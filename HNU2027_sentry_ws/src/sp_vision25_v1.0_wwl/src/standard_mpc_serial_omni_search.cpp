#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/nav_serial_gimbal.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | Print command-line usage}"
  "{@config-path   | | YAML config path }";

using namespace std::chrono_literals;

namespace
{
struct SearchObservation
{
  bool valid{false};
  float delta_yaw{0.0F};
  float delta_pitch{0.0F};
  float target_yaw{0.0F};
  float target_pitch{0.0F};
  std::string camera_name;
  std::chrono::steady_clock::time_point timestamp{};
};

struct ControlUpdate
{
  std::optional<auto_aim::Target> target;
  std::optional<SearchObservation> search;
  std::chrono::steady_clock::time_point timestamp;
};

struct UsbPreviewFrame
{
  bool valid{false};
  cv::Mat image;
  std::string camera_name;
  std::string status_text;
  bool found{false};
};

struct OmniCameraConfig
{
  std::string device;
  std::string name;
  float yaw_center_deg{0.0F};
};

struct SearchDebugCounters
{
  size_t frames{0};
  size_t raw_armors{0};
  size_t valid_armors{0};
  size_t wrong_color_or_invalid{0};
};

float readFloat(const YAML::Node & yaml, const std::string & key, float default_value)
{
  return yaml[key] ? yaml[key].as<float>() : default_value;
}

std::string readString(const YAML::Node & yaml, const std::string & key, const std::string & value)
{
  return yaml[key] ? yaml[key].as<std::string>() : value;
}

bool readBool(const YAML::Node & yaml, const std::string & key, bool default_value)
{
  return yaml[key] ? yaml[key].as<bool>() : default_value;
}

std::string stripDevPrefix(std::string device)
{
  const std::string prefix = "/dev/";
  if (device.rfind(prefix, 0) == 0) device.erase(0, prefix.size());
  return device;
}

std::vector<OmniCameraConfig> loadOmniCameras(const YAML::Node & yaml)
{
  std::vector<OmniCameraConfig> cameras;
  if (yaml["omni_search_cameras"]) {
    for (const auto & node : yaml["omni_search_cameras"]) {
      OmniCameraConfig cfg;
      cfg.device = stripDevPrefix(readString(node, "device", ""));
      cfg.name = readString(node, "name", cfg.device);
      cfg.yaw_center_deg = readFloat(node, "yaw_center_deg", 0.0F);
      if (!cfg.device.empty()) cameras.push_back(cfg);
    }
  }

  if (cameras.empty()) {
    cameras.push_back({"video0", "left", -60.0F});
    cameras.push_back({"video2", "right", 60.0F});
  }
  return cameras;
}

bool isEnemy(auto_aim::Color color, const std::string & enemy_color)
{
  if (enemy_color == "red") return color == auto_aim::Color::red;
  if (enemy_color == "blue") return color == auto_aim::Color::blue;
  return true;
}

std::optional<SearchObservation> detectSearchTarget(
  auto_aim::YOLO & yolo, const cv::Mat & image, const OmniCameraConfig & cfg,
  const std::string & enemy_color, float fov_h_deg, float fov_v_deg,
  std::chrono::steady_clock::time_point timestamp, SearchDebugCounters * debug = nullptr)
{
  auto armors = yolo.detect(image);
  const auto raw_count = armors.size();
  if (debug) {
    debug->raw_armors += raw_count;
  }
  armors.remove_if([&](const auto_aim::Armor & armor) {
    return !isEnemy(armor.color, enemy_color) || armor.name == auto_aim::ArmorName::not_armor;
  });
  if (debug) {
    const auto valid_count = armors.size();
    debug->valid_armors += valid_count;
    debug->wrong_color_or_invalid += raw_count - valid_count;
  }
  if (armors.empty()) return std::nullopt;

  armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
    return a.confidence > b.confidence;
  });

  const auto & armor = armors.front();
  SearchObservation observation;
  observation.valid = true;
  observation.camera_name = cfg.name;
  observation.timestamp = timestamp;
  observation.delta_yaw =
    (cfg.yaw_center_deg + (0.5F - armor.center_norm.x) * fov_h_deg) / 57.2957795F;
  observation.delta_pitch = ((armor.center_norm.y - 0.5F) * fov_v_deg) / 57.2957795F;
  return observation;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  const auto yaml = tools::load(config_path);
  const auto target_timeout = std::chrono::milliseconds(
    yaml["nav_serial_target_timeout_ms"] ?
      yaml["nav_serial_target_timeout_ms"].as<int>() : 200);
  const auto search_timeout = std::chrono::milliseconds(
    yaml["omni_search_timeout_ms"] ? yaml["omni_search_timeout_ms"].as<int>() : 300);
  const auto search_hold_timeout = std::chrono::milliseconds(
    yaml["omni_search_hold_ms"] ? yaml["omni_search_hold_ms"].as<int>() : 3000);
  const bool omni_enabled = readBool(yaml, "omni_search_enabled", true);
  const float omni_fov_h_deg = readFloat(yaml, "omni_search_fov_h_deg", 60.0F);
  const float omni_fov_v_deg = readFloat(yaml, "omni_search_fov_v_deg", 45.0F);
  const float search_yaw_limit_deg = readFloat(yaml, "omni_search_yaw_limit_deg", 90.0F);
  const float search_pitch_limit_deg = readFloat(yaml, "omni_search_pitch_limit_deg", 20.0F);
  const bool show_usb_preview = readBool(yaml, "omni_search_show_usb", true);
  const std::string enemy_color = readString(yaml, "enemy_color", "red");
  const auto omni_cameras = loadOmniCameras(yaml);

  if (target_timeout.count() <= 0 || search_timeout.count() <= 0 || search_hold_timeout.count() <= 0) {
    tools::logger()->warn("target/search/hold timeout must be positive");
    return 1;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;
  io::NavSerialGimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  std::mutex search_mutex;
  SearchObservation latest_search;
  std::mutex usb_preview_mutex;
  UsbPreviewFrame latest_usb_preview;
  std::vector<std::thread> omni_threads;
  std::atomic<bool> quit{false};
  std::exception_ptr omni_error;

  if (omni_enabled) {
    for (const auto & cfg : omni_cameras) {
      omni_threads.emplace_back([&, cfg]() {
        try {
          io::USBCamera usb_camera(cfg.device, config_path);
          auto_aim::YOLO local_yolo(config_path, false);
          auto diagnostic_time = std::chrono::steady_clock::now();
          SearchDebugCounters debug;
          size_t empty_frames = 0;
          tools::logger()->info(
            "[OmniSearch] {} on /dev/{} yaw_center={:.1f} fov=({:.1f},{:.1f})",
            cfg.name, cfg.device, cfg.yaw_center_deg, omni_fov_h_deg, omni_fov_v_deg);
          while (!quit.load(std::memory_order_acquire)) {
            cv::Mat image;
            std::chrono::steady_clock::time_point timestamp;
            usb_camera.read(image, timestamp);
            if (image.empty()) {
              ++empty_frames;
              std::this_thread::sleep_for(20ms);
              const auto now = std::chrono::steady_clock::now();
              if (std::chrono::duration<double>(now - diagnostic_time).count() >= 1.0) {
                tools::logger()->warn(
                  "[OmniSearch:{}] no USB frame yet, empty_reads={}", cfg.name, empty_frames);
                empty_frames = 0;
                diagnostic_time = now;
              }
              continue;
            }
            ++debug.frames;
            auto observation = detectSearchTarget(
              local_yolo, image, cfg, enemy_color, omni_fov_h_deg, omni_fov_v_deg, timestamp,
              &debug);
            const auto now = std::chrono::steady_clock::now();
            if (observation.has_value()) {
              const auto state = gimbal.state();
              observation->timestamp = now;
              observation->target_yaw = tools::limit_rad(state.yaw + observation->delta_yaw);
              observation->target_pitch = tools::limit_rad(state.pitch + observation->delta_pitch);
            }
            if (show_usb_preview) {
              const auto mean = cv::mean(image);
              const double brightness = (mean[0] + mean[1] + mean[2]) / 3.0;
              UsbPreviewFrame preview_frame;
              preview_frame.valid = true;
              preview_frame.image = image.clone();
              preview_frame.camera_name = cfg.name;
              preview_frame.found = observation.has_value();
              preview_frame.status_text =
                observation.has_value() ?
                  cv::format(
                    "%s FOUND dyaw:%.1f dpitch:%.1f brightness:%.1f", cfg.name.c_str(),
                    observation->delta_yaw * 57.2957795F,
                    observation->delta_pitch * 57.2957795F, brightness) :
                  cv::format("%s no target brightness:%.1f", cfg.name.c_str(), brightness);
              {
                std::lock_guard<std::mutex> lock(usb_preview_mutex);
                latest_usb_preview = std::move(preview_frame);
              }
            }
            if (std::chrono::duration<double>(now - diagnostic_time).count() >= 1.0) {
              const auto mean = cv::mean(image);
              const double brightness = (mean[0] + mean[1] + mean[2]) / 3.0;
              tools::logger()->info(
                "[OmniSearch:{}] usb_fps={:.1f}, brightness={:.1f}, raw_armors={}, "
                "valid_armors={}, filtered={}",
                cfg.name,
                debug.frames / std::chrono::duration<double>(now - diagnostic_time).count(),
                brightness, debug.raw_armors, debug.valid_armors, debug.wrong_color_or_invalid);
              debug = {};
              empty_frames = 0;
              diagnostic_time = now;
            }
            if (!observation.has_value()) continue;
            {
              std::lock_guard<std::mutex> lock(search_mutex);
              latest_search = observation.value();
            }
          }
        } catch (...) {
          omni_error = std::current_exception();
          quit.store(true, std::memory_order_release);
        }
      });
    }
  }

  tools::ThreadSafeQueue<ControlUpdate, true> control_queue(1);
  control_queue.push({std::nullopt, std::nullopt, std::chrono::steady_clock::now()});
  std::exception_ptr plan_error;

  auto plan_thread = std::thread([&]() {
    try {
      bool target_was_stale = false;
      bool search_was_stale = false;
      while (!quit.load(std::memory_order_acquire)) {
        const auto update = control_queue.front();
        const auto now = std::chrono::steady_clock::now();
        const bool target_is_stale =
          now - update.timestamp > target_timeout;
        const bool search_is_stale =
          !update.search.has_value() || now - update.search->timestamp > search_timeout;
        const bool search_hold_is_valid =
          update.search.has_value() && now - update.search->timestamp <= search_hold_timeout;

        if (target_is_stale && update.target.has_value() && !target_was_stale) {
          tools::logger()->warn(
            "[SelfAim] Target input is older than {} ms; using omni search if available",
            target_timeout.count());
        }
        target_was_stale = target_is_stale && update.target.has_value();
        search_was_stale = search_is_stale && update.search.has_value();

        if (update.target.has_value() && !target_is_stale) {
          const auto plan = planner.plan(update.target, gimbal.state().bullet_speed);
          gimbal.send(
            plan.control, false, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
            plan.pitch_vel, plan.pitch_acc);
        } else if (update.search.has_value() && search_hold_is_valid) {
          const float yaw_offset =
            std::clamp(update.search->delta_yaw, -search_yaw_limit_deg / 57.2957795F,
                       search_yaw_limit_deg / 57.2957795F);
          const float pitch_offset =
            std::clamp(update.search->delta_pitch, -search_pitch_limit_deg / 57.2957795F,
                       search_pitch_limit_deg / 57.2957795F);
          const float target_yaw =
            tools::limit_rad(update.search->target_yaw - update.search->delta_yaw + yaw_offset);
          const float target_pitch = tools::limit_rad(
            update.search->target_pitch - update.search->delta_pitch + pitch_offset);
          gimbal.send(true, false, target_yaw, 0.0F, 0.0F, target_pitch, 0.0F, 0.0F);
        } else {
          if (search_was_stale) {
            tools::logger()->debug("[OmniSearch] search observation expired");
          }
          gimbal.send(false, false, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        }
        std::this_thread::sleep_for(10ms);
      }
    } catch (...) {
      plan_error = std::current_exception();
      quit.store(true, std::memory_order_release);
    }
  });

  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp;
  auto diagnostic_time = std::chrono::steady_clock::now();
  size_t diagnostic_frames = 0;
  double brightness = 0.0;
  std::exception_ptr processing_error;
  try {
    while (!exiter.exit() && !quit.load(std::memory_order_acquire)) {
      camera.read(image, timestamp);

      std::optional<SearchObservation> search;
      {
        std::lock_guard<std::mutex> lock(search_mutex);
        if (latest_search.valid) search = latest_search;
      }

      if (image.empty()) {
        control_queue.push({std::nullopt, search, std::chrono::steady_clock::now()});
        const auto key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q' || key == 'Q') break;
        continue;
      }

      const auto orientation = gimbal.q(timestamp);
      recorder.record(image, orientation, timestamp);
      solver.set_R_gimbal2world(orientation);

      auto armors = yolo.detect(image);
      const auto targets = tracker.track(armors, timestamp);
      if (targets.empty())
        control_queue.push({std::nullopt, search, std::chrono::steady_clock::now()});
      else
        control_queue.push({targets.front(), std::nullopt, std::chrono::steady_clock::now()});

      const auto mean = cv::mean(image);
      brightness = (mean[0] + mean[1] + mean[2]) / 3.0;

      cv::Mat preview;
      cv::resize(image, preview, {}, 0.5, 0.5);
      const auto preview_now = std::chrono::steady_clock::now();
      const int omni_age_ms =
        search.has_value() ?
          static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
              preview_now - search->timestamp)
              .count()) :
          -1;
      const bool omni_active = search.has_value() && omni_age_ms <= search_timeout.count();
      const bool omni_hold = search.has_value() && omni_age_ms <= search_hold_timeout.count();
      const std::string control_text =
        !targets.empty() ? " ctrl:main" :
                           (omni_active ? " ctrl:omni" : (omni_hold ? " ctrl:hold" : " ctrl:none"));
      const std::string omni_text =
        search.has_value() ?
          cv::format(
            " omni:%s %s age:%dms dyaw:%.1f dpitch:%.1f", search->camera_name.c_str(),
            omni_active ? "active" : (omni_hold ? "hold" : "stale"), omni_age_ms,
            search->delta_yaw * 57.2957795F, search->delta_pitch * 57.2957795F) :
          " omni:none";
      cv::putText(
        preview,
        cv::format("brightness: %.1f  armors: %zu  targets: %zu%s%s", brightness, armors.size(),
                   targets.size(), control_text.c_str(), omni_text.c_str()),
        {16, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2, cv::LINE_AA);
      cv::imshow("SP Vision Camera + Omni Search", preview);

      if (show_usb_preview) {
        UsbPreviewFrame usb_preview_frame;
        {
          std::lock_guard<std::mutex> lock(usb_preview_mutex);
          if (latest_usb_preview.valid) {
            usb_preview_frame.valid = true;
            usb_preview_frame.image = latest_usb_preview.image.clone();
            usb_preview_frame.camera_name = latest_usb_preview.camera_name;
            usb_preview_frame.status_text = latest_usb_preview.status_text;
            usb_preview_frame.found = latest_usb_preview.found;
          }
        }
        if (usb_preview_frame.valid && !usb_preview_frame.image.empty()) {
          cv::Mat usb_preview;
          cv::resize(usb_preview_frame.image, usb_preview, {}, 0.75, 0.75);
          cv::putText(
            usb_preview, usb_preview_frame.status_text, {16, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
            usb_preview_frame.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), 2,
            cv::LINE_AA);
          cv::imshow("SP Vision Omni USB " + usb_preview_frame.camera_name, usb_preview);
        }
      }

      const auto key = cv::waitKey(1) & 0xFF;
      if (key == 27 || key == 'q' || key == 'Q') break;

      ++diagnostic_frames;
      const auto now = std::chrono::steady_clock::now();
      const auto diagnostic_interval =
        std::chrono::duration<double>(now - diagnostic_time).count();
      if (diagnostic_interval >= 1.0) {
        const auto gimbal_state = gimbal.state();
        tools::logger()->info(
          "[SelfAimOmni] camera_fps={:.1f}, brightness={:.1f}, armors={}, targets={}, "
          "imu_yaw={:.3f}, imu_pitch={:.3f}, control={}, omni={}, omni_age_ms={}",
          diagnostic_frames / diagnostic_interval, brightness, armors.size(), targets.size(),
          gimbal_state.yaw, gimbal_state.pitch,
          !targets.empty() ? "main" : (omni_active ? "omni" : (omni_hold ? "hold" : "none")),
          search.has_value() ? search->camera_name : "none", omni_age_ms);
        diagnostic_frames = 0;
        diagnostic_time = now;
      }
    }
  } catch (...) {
    processing_error = std::current_exception();
  }

  quit.store(true, std::memory_order_release);
  if (plan_thread.joinable()) plan_thread.join();
  for (auto & thread : omni_threads) {
    if (thread.joinable()) thread.join();
  }
  if (!gimbal.stopAndWait(250ms)) {
    tools::logger()->warn(
      "[SelfAimOmni] Serial did not confirm a safe zero frame before cleanup");
  }
  cv::destroyAllWindows();

  const auto error = plan_error ? plan_error : (processing_error ? processing_error : omni_error);
  if (error) {
    try {
      std::rethrow_exception(error);
    } catch (const std::exception & e) {
      tools::logger()->error("[SelfAimOmni] Stopped after an exception: {}", e.what());
    } catch (...) {
      tools::logger()->error("[SelfAimOmni] Stopped after an unknown exception");
    }
    return 1;
  }
  return 0;
}
