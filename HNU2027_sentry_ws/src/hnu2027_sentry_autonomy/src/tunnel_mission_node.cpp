#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "hnu_sentry_interfaces/action/traverse_tunnel.hpp"
#include "hnu_sentry_interfaces/msg/traverse_control.hpp"
#include "hnu_sentry_interfaces/msg/lower_controller_state.hpp"
#include "hnu_sentry_interfaces/msg/tunnel_observation.hpp"
#include "hnu_tunnel_core/traverse_state_machine.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace {
using Phase=hnu_tunnel_core::TraversePhase;
using Nav=nav2_msgs::action::NavigateToPose;
using Traverse=hnu_sentry_interfaces::action::TraverseTunnel;
using State=hnu_sentry_interfaces::msg::LowerControllerState;
using Observation=hnu_sentry_interfaces::msg::TunnelObservation;
using Request=hnu_sentry_interfaces::msg::TraverseControl;
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double bounded(double a,double limit){return std::clamp(a,-limit,limit);}
double steady(){return std::chrono::duration<double>(
  std::chrono::steady_clock::now().time_since_epoch()).count();}
class Mission : public rclcpp::Node {
 public:
 Mission():Node("tunnel_mission"),tf_(get_clock()),listener_(tf_) {
  tunnel_id_=declare_parameter<std::string>("tunnel_id","blue_spawn_tunnel");
  center_x_=declare_parameter("center_x",-2.5);
  center_y_=declare_parameter("center_y",3.5);
  lateral_=declare_parameter("folded_center_offset_y",0.059);
  entry_x_=declare_parameter("entry_x",-3.2);exit_x_=declare_parameter("exit_x",-1.8);
  width_=declare_parameter("folded_width",0.646);height_=declare_parameter("folded_height",0.223);
  obs_timeout_=declare_parameter("observation_timeout",0.5);
  nav_=rclcpp_action::create_client<Nav>(this,"navigate_to_pose");
  lifecycle_=create_client<lifecycle_msgs::srv::GetState>("/bt_navigator/get_state");
  clear_global_=create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
  clear_local_=create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();});
  grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",
    rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      grid_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();});

  request_pub_=create_publisher<Request>("/tunnel/control_request",1);
  velocity_pub_=create_publisher<geometry_msgs::msg::TwistStamped>("/tunnel/cmd_vel",1);
  status_pub_=create_publisher<std_msgs::msg::String>("/mission/status",10);
  state_sub_=create_subscription<State>("/robot/controller_state",1,[this](State::ConstSharedPtr m){
    state_=*m;state_at_=steady();
  });
  obs_sub_=create_subscription<Observation>("/tunnel/observation",10,[this](Observation::ConstSharedPtr m){
    // Replayed measurements do not refresh the observation watchdog.
    const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
    if(stamp>obs_stamp_){observation_=*m;obs_at_=steady();obs_stamp_=stamp;}
  });
  goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/mission/goal",1,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
      if(active_){RCLCPP_WARN(get_logger(),"Mission already active");return;}
      if(m->header.frame_id!="map" || !std::isfinite(m->pose.position.x) ||
         !std::isfinite(m->pose.position.y)){RCLCPP_ERROR(get_logger(),"Goal must be finite in map");return;}
      if(m->pose.position.x<=exit_x_){RCLCPP_ERROR(get_logger(),"This entry-to-exit route requires a goal beyond exit");return;}
      final_goal_=*m;has_final_=true;start();
    });
  action_=rclcpp_action::create_server<Traverse>(this,"traverse_tunnel",
    [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Traverse::Goal> g){
      return (!active_ && g->tunnel_id==tunnel_id_ && g->direction==0) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Traverse>>){
      cancel_requested_=true;return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Traverse>> handle){
      handle_=handle;has_final_=false;start();});
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
 }
 private:
 void phase(Phase p){if(phase_!=p){phase_=p;++request_id_;
   if(p==Phase::kComplete)handoff_started_at_=steady();
 }}
 void start(){
  if(!state_.valid || !state_.unfolded || !state_.stopped || state_.fault){
    reason_="start_requires_measured_unfolded_stopped_robot";
    if(handle_){auto r=std::make_shared<Traverse::Result>();r->success=false;r->reason=reason_;
      r->terminal_phase="rejected";handle_->abort(r);handle_.reset();}
    return;
  }
  if(!locate() || x_>entry_x_+0.15){
    reason_="robot_not_on_configured_entry_side";
    if(handle_){auto r=std::make_shared<Traverse::Result>();r->success=false;
      r->terminal_phase="rejected";r->reason=reason_;handle_->abort(r);handle_.reset();}
    return;
  }
  ++clear_generation_;clear_started_=false;clear_count_=0;clear_done_stamp_=0;required_scan_stamp_=0;
  hnu_tunnel_core::TraverseTimeouts t;t.approach_s=60;t.detect_s=8;t.align_s=20;
  t.fold_s=15;t.verify_s=8;t.traverse_s=30;t.unfold_s=15;
  fsm_=std::make_unique<hnu_tunnel_core::TraverseStateMachine>(t);
  cancel_requested_=false;active_=true;aborted_=false;post_=false;finishing_=false;nav_sent_=false;
  nav_done_=false;nav_failed_=false;started_=steady();
  phase(fsm_->start(steady()).phase);reason_="approach";
 }
 bool locate(){
  try{
    auto tr=tf_.lookupTransform("map","base_link",tf2::TimePointZero);
    double age=(now()-rclcpp::Time(tr.header.stamp,get_clock()->get_clock_type())).seconds();
    if(age>0.75)return false;
    x_=tr.transform.translation.x;y_=tr.transform.translation.y;
    auto q=tr.transform.rotation;
    yaw_=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    return std::isfinite(x_)&&std::isfinite(y_)&&std::isfinite(yaw_);
  }catch(const tf2::TransformException&){return false;}
 }
 void navigation(double x,double y){
  if(nav_sent_ || !nav_active_ || !nav_->action_server_is_ready())return;
  nav_sent_=true;nav_done_=false;nav_failed_=false;
  Nav::Goal goal;goal.pose.header.frame_id="map";goal.pose.header.stamp=now();
  goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;goal.pose.pose.orientation.w=1;
  auto epoch=++nav_epoch_;
  rclcpp_action::Client<Nav>::SendGoalOptions opts;
  opts.goal_response_callback=[this,epoch](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr g){
    if(epoch!=nav_epoch_){if(g)nav_->async_cancel_goal(g);return;}
    nav_handle_=g;if(!g)nav_failed_=true;
  };
  opts.result_callback=[this,epoch](const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult& r){
    if(epoch!=nav_epoch_)return;
    nav_handle_.reset();
    nav_done_=r.code==rclcpp_action::ResultCode::SUCCEEDED;
    nav_failed_=!nav_done_;
  };
  nav_->async_send_goal(goal,opts);
 }
 void cancel_nav(){
  ++nav_epoch_;
  if(nav_handle_){try{nav_->async_cancel_goal(nav_handle_);}catch(const rclcpp_action::exceptions::UnknownGoalHandleError&){}}
  nav_handle_.reset();nav_sent_=false;
 }
 void abort(const std::string& why){
  if(aborted_)return;
  reason_=why;aborted_=true;active_=false;cancel_nav();phase(Phase::kAbort);
  if(handle_){
    auto r=std::make_shared<Traverse::Result>();r->success=false;r->terminal_phase="abort";r->reason=why;
    if(handle_->is_canceling())handle_->canceled(r);else handle_->abort(r);
    handle_.reset();
  }
 }
 bool observed()const{
  return steady()-obs_at_<=obs_timeout_ && observation_.detected &&
    observation_.geometry_valid && observation_.obstacle_free &&
    observation_.width_m>=width_+0.04 && observation_.clearance_m>=height_+0.01;
 }
 bool navigation_handoff_ready(){
  if(steady()-handoff_started_at_>8){
    abort("navigation_costmap_refresh_timeout");return false;
  }
  if(!clear_started_){
    if(!clear_global_->service_is_ready() || !clear_local_->service_is_ready())return false;
    clear_started_=true;
    const auto generation=clear_generation_;
    auto callback=[this,generation](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture f){
      if(generation!=clear_generation_)return;
      (void)f.get();
      if(++clear_count_==2)clear_done_stamp_=now().nanoseconds();
    };
    clear_global_->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>(),callback);
    clear_local_->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>(),callback);
  }
  if(clear_count_==2 && scan_stamp_>clear_done_stamp_ && required_scan_stamp_==0)
    required_scan_stamp_=scan_stamp_;
  return required_scan_stamp_>0 && grid_stamp_>=required_scan_stamp_;
 }
 void tick(){
  if(steady()-lifecycle_check_at_>0.5 && !lifecycle_pending_ && lifecycle_->service_is_ready()){
    lifecycle_check_at_=steady();lifecycle_pending_=true;
    lifecycle_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f){
        nav_active_=f.get()->current_state.id==3;lifecycle_pending_=false;
      });
  }
  geometry_msgs::msg::TwistStamped velocity;
  velocity.header.stamp=now();velocity.header.frame_id="base_link";
  const bool localized=locate();
  const bool fresh=state_.valid && steady()-state_at_<0.5;
  if(active_){
    if(cancel_requested_)abort("cancel_requested");
    else if(steady()-started_>150)abort("mission_timeout");
    else if(!fresh || state_.fault)abort("controller_feedback_invalid");
    else if(!localized)abort("localization_transform_unavailable");
    else if(nav_failed_)abort("nav2_goal_failed");
    else if(phase_==Phase::kTraverse && !state_.folded)abort("fold_state_lost_during_traverse");
    else if(post_){
      navigation(final_goal_.pose.position.x,final_goal_.pose.position.y);
      if(nav_done_)finishing_=true;
      if(finishing_ && state_.stopped){
        active_=false;reason_="succeeded";phase(Phase::kNormal);
      }else reason_=finishing_?"stopping_at_goal":"post_tunnel_navigation";
    }else{
      if(phase_==Phase::kApproach)navigation(entry_x_,center_y_);
      hnu_tunnel_core::TraverseInput in;
      in.at_staging_pose=nav_done_&&std::hypot(x_-entry_x_,y_-center_y_)<0.12;
      in.require_restore=true;
      in.restore_completed=state_.unfolded && state_.centered && state_.stopped &&
        std::abs(y_-center_y_)<0.012 && std::abs(wrap(yaw_))<0.025;
      in.observation_safe=observed();
      in.observation_available=steady()-obs_at_<=obs_timeout_ &&
        observation_.detected && observation_.geometry_valid;
      in.aligned=state_.centered && state_.stopped &&
        std::abs(wrap(yaw_))<0.025 && std::abs(y_-center_y_-lateral_)<0.012;
      const bool ack=state_.accepted_request_id==request_id_;
      in.fold_target=state_.folded;
      in.fold_completed=ack&&(phase_==Phase::kUnfold?state_.unfolded:state_.folded);
      in.fold_fault=state_.fault;
      // Center beyond roof edge plus the whole collision footprint and margin.
      in.exited_tunnel=x_>=exit_x_ && std::abs(y_-center_y_-lateral_)<0.08;
      auto out=fsm_->step(in,steady());
      phase(out.phase);reason_=out.reason;
      if(phase_==Phase::kAbort || phase_==Phase::kFallback)abort(out.reason);
      if(phase_==Phase::kAlign){
        const double lateral_error=center_y_+lateral_-y_;
        const double heading_error=wrap(-yaw_);
        const double world_vy=std::abs(heading_error)<0.1 && std::abs(lateral_error)>0.012?bounded(0.8*lateral_error,0.06):0;
        velocity.twist.linear.x=std::sin(yaw_)*world_vy;
        velocity.twist.linear.y=std::cos(yaw_)*world_vy;
        velocity.twist.angular.z=std::abs(heading_error)>0.025?bounded(0.8*heading_error,0.3):0;
      }else if(phase_==Phase::kRestore){
        const double lateral_error=center_y_-y_;
        const double heading_error=wrap(-yaw_);
        const double vy=std::abs(lateral_error)>0.012?bounded(.8*lateral_error,.06):0;
        velocity.twist.linear.x=std::sin(yaw_)*vy;
        velocity.twist.linear.y=std::cos(yaw_)*vy;
        velocity.twist.angular.z=std::abs(heading_error)>.025?bounded(.8*heading_error,.25):0;
      }else if(phase_==Phase::kTraverse && out.allow_traverse){
        if(!state_.folded)abort("fold_state_lost_during_traverse");
        else{
          const double world_vy=bounded(0.8*(center_y_+lateral_-y_),0.05);
          velocity.twist.linear.x=std::cos(yaw_)*0.10+std::sin(yaw_)*world_vy;
          velocity.twist.linear.y=-std::sin(yaw_)*0.10+std::cos(yaw_)*world_vy;
          velocity.twist.angular.z=bounded(0.8*wrap(-yaw_),0.25);
        }
      }else if(phase_==Phase::kComplete){
        if(!navigation_handoff_ready()){
          if(!aborted_)reason_="refreshing_navigation_costmaps";
        }else if(has_final_){
          post_=true;nav_sent_=false;nav_done_=false;nav_failed_=false;
        }else if(state_.unfolded&&state_.gimbal_ready){
          active_=false;reason_="succeeded";
          if(handle_){auto r=std::make_shared<Traverse::Result>();r->success=true;
            r->terminal_phase="complete";r->reason="measured_exit_and_unfold";
            handle_->succeed(r);handle_.reset();}
        }
      }
    }
  }
  if(aborted_)velocity.twist=geometry_msgs::msg::Twist();
  Request req;req.header.stamp=now();req.request_id=request_id_;
  req.phase=static_cast<uint8_t>(phase_);request_pub_->publish(req);
  velocity_pub_->publish(velocity);
  std_msgs::msg::String status;
  std::ostringstream os;
  os<<"{\"active\":"<<(active_?"true":"false")<<",\"phase\":\""
    <<hnu_tunnel_core::to_string(phase_)<<"\",\"reason\":\""<<reason_
    <<"\",\"localized\":"<<(localized?"true":"false")<<",\"x\":"<<x_
    <<",\"y\":"<<y_<<",\"yaw\":"<<yaw_<<",\"observation_safe\":"
    <<(observed()?"true":"false")<<",\"width\":"<<observation_.width_m
    <<",\"clearance\":"<<observation_.clearance_m
    <<",\"observation_reason\":\""<<observation_.reason.substr(0,observation_.reason.find(':'))
    <<"\",\"folded\":"<<(state_.folded?"true":"false")
    <<",\"unfolded\":"<<(state_.unfolded?"true":"false")
    <<",\"stopped\":"<<(state_.stopped?"true":"false")
    <<",\"controller_mode\":"<<static_cast<int>(state_.chassis_mode)
    <<",\"actual_wz\":"<<state_.velocity.angular.z<<"}";
  status.data=os.str();status_pub_->publish(status);
  if(handle_){auto f=std::make_shared<Traverse::Feedback>();
    f->phase=hnu_tunnel_core::to_string(phase_);f->detail=reason_;handle_->publish_feedback(f);}
 }
 tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
 rclcpp_action::Client<Nav>::SharedPtr nav_;
 rclcpp_action::ClientGoalHandle<Nav>::SharedPtr nav_handle_;
 rclcpp_action::Server<Traverse>::SharedPtr action_;
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Traverse>> handle_;
 rclcpp::Publisher<Request>::SharedPtr request_pub_;
 rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
 rclcpp::Subscription<State>::SharedPtr state_sub_;
 rclcpp::Subscription<Observation>::SharedPtr obs_sub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::unique_ptr<hnu_tunnel_core::TraverseStateMachine> fsm_;
 bool cancel_requested_{false};
 rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_;
 bool nav_active_{false},lifecycle_pending_{false};
 double lifecycle_check_at_{-1e9};
 rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_,clear_local_;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
 bool clear_started_{false};int clear_count_{0};uint64_t clear_generation_{0};
 int64_t scan_stamp_{0},grid_stamp_{0},clear_done_stamp_{0},required_scan_stamp_{0};
 double handoff_started_at_{0};
 State state_;Observation observation_;geometry_msgs::msg::PoseStamped final_goal_;
 Phase phase_{Phase::kNormal};uint64_t request_id_{1},nav_epoch_{0};
 bool active_{false},aborted_{false},post_{false},finishing_{false},has_final_{false};
 bool nav_sent_{false},nav_done_{false},nav_failed_{false};
 double state_at_{-1e9},obs_at_{-1e9},started_{0},x_{0},y_{0},yaw_{0};
 int64_t obs_stamp_{-1};std::string tunnel_id_,reason_{"idle"};
 double center_x_,center_y_,lateral_,entry_x_,exit_x_,width_,height_,obs_timeout_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Mission>());
rclcpp::shutdown();return 0;}
