// Simulation-only transport watchdog. Native VelocityControl has no command expiry.
#include <chrono>
#include <mutex>
#include <string>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
namespace hnu_gazebo_adapter {
class DriveWatchdog : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity&,
      const std::shared_ptr<const sdf::Element>& sdf,
      ignition::gazebo::EntityComponentManager&,
      ignition::gazebo::EventManager&) override {
    const auto topic = sdf->Get<std::string>("topic");
    publisher_ = node_.Advertise<ignition::msgs::Twist>(topic);
    node_.Subscribe(topic, &DriveWatchdog::Command, this);
    last_ = std::chrono::steady_clock::now();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo&,
                 ignition::gazebo::EntityComponentManager&) override {
    bool expired = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expired = std::chrono::steady_clock::now() - last_ > std::chrono::milliseconds(400);
    }
    if (expired) publisher_.Publish(ignition::msgs::Twist());
  }
 private:
  void Command(const ignition::msgs::Twist&) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_ = std::chrono::steady_clock::now();
  }
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher publisher_;
  std::chrono::steady_clock::time_point last_;
  std::mutex mutex_;
};
}
IGNITION_ADD_PLUGIN(hnu_gazebo_adapter::DriveWatchdog, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure, ignition::gazebo::ISystemPreUpdate)
