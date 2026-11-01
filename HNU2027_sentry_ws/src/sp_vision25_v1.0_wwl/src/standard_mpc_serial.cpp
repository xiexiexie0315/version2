#include <atomic>
#include <chrono>
#include <exception>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/nav_serial_gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | Print command-line usage}"
  "{@config-path   | | YAML config path }";

using namespace std::chrono_literals;

namespace
{
struct TargetUpdate
{
  std::optional<auto_aim::Target> target;
  std::chrono::steady_clock::time_point timestamp;
};
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
  tools::logger()->info("[SelfAim] cwd={}", std::filesystem::current_path().string());
  tools::logger()->info("[SelfAim] config={}", std::filesystem::absolute(config_path).string());
  tools::logger()->info(
    "[SelfAim] yolo_name={}, yolov5_model_path={}, classify_model={}, enemy_color={}, "
    "exposure_ms={}, gain={}, use_traditional={}, threshold={}",
    yaml["yolo_name"].as<std::string>(), yaml["yolov5_model_path"].as<std::string>(),
    yaml["classify_model"].as<std::string>(), yaml["enemy_color"].as<std::string>(),
    yaml["exposure_ms"].as<double>(), yaml["gain"].as<double>(),
    yaml["use_traditional"].as<bool>(), yaml["threshold"].as<double>());
  const auto target_timeout = std::chrono::milliseconds(
    yaml["nav_serial_target_timeout_ms"] ?
      yaml["nav_serial_target_timeout_ms"].as<int>() : 200);
  if (target_timeout.count() <= 0) {
    tools::logger()->warn("nav_serial_target_timeout_ms must be positive");
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

  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
  std::atomic<bool> quit{false};
  std::exception_ptr plan_error;

  auto plan_thread = std::thread([&]() {
    try {
      bool target_was_stale = false;
      while (!quit) {
        const auto update = target_queue.front();
        const bool target_is_stale =
          std::chrono::steady_clock::now() - update.timestamp > target_timeout;
        if (target_is_stale && update.target.has_value() && !target_was_stale) {
          tools::logger()->warn(
            "[SelfAim] Target input is older than {} ms; sending a safe zero command",
            target_timeout.count());
        }
        target_was_stale = target_is_stale && update.target.has_value();
        const auto target = target_is_stale ? std::nullopt : update.target;
        const auto plan = planner.plan(target, gimbal.state().bullet_speed);
        gimbal.send(
          plan.control, false, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
          plan.pitch_vel, plan.pitch_acc);
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
      if (image.empty()) {
        target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
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
        target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
      else
        target_queue.push({targets.front(), std::chrono::steady_clock::now()});

      const auto mean = cv::mean(image);
      brightness = (mean[0] + mean[1] + mean[2]) / 3.0;

      cv::Mat preview;
      cv::resize(image, preview, {}, 0.5, 0.5);
      cv::putText(
        preview, cv::format("brightness: %.1f  armors: %zu  targets: %zu", brightness,
                            armors.size(), targets.size()),
        {16, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2, cv::LINE_AA);
      cv::imshow("SP Vision Camera", preview);
      const auto key = cv::waitKey(1) & 0xFF;
      if (key == 27 || key == 'q' || key == 'Q') break;

      ++diagnostic_frames;
      const auto now = std::chrono::steady_clock::now();
      const auto diagnostic_interval =
        std::chrono::duration<double>(now - diagnostic_time).count();
      if (diagnostic_interval >= 1.0) {
        const auto gimbal_state = gimbal.state();
        tools::logger()->info(
          "[SelfAim] camera_fps={:.1f}, brightness={:.1f}, armors={}, targets={}, "
          "imu_yaw={:.3f}, imu_pitch={:.3f}",
          diagnostic_frames / diagnostic_interval, brightness, armors.size(), targets.size(),
          gimbal_state.yaw, gimbal_state.pitch);
        diagnostic_frames = 0;
        diagnostic_time = now;
      }
    }
  } catch (...) {
    processing_error = std::current_exception();
  }

  quit.store(true, std::memory_order_release);
  if (plan_thread.joinable()) plan_thread.join();
  if (!gimbal.stopAndWait(250ms)) {
    tools::logger()->warn(
      "[SelfAim] Serial did not confirm a safe zero frame before cleanup");
  }
  cv::destroyAllWindows();

  const auto error = plan_error ? plan_error : processing_error;
  if (error) {
    try {
      std::rethrow_exception(error);
    } catch (const std::exception & e) {
      tools::logger()->error("[SelfAim] Stopped after an exception: {}", e.what());
    } catch (...) {
      tools::logger()->error("[SelfAim] Stopped after an unknown exception");
    }
    return 1;
  }
  return 0;
}
