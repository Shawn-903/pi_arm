#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "pi_arm_interfaces/action/direct_move.hpp"
#include "pi_arm_interfaces/action/move_j.hpp"
#include "pi_arm_interfaces/action/move_js.hpp"
#include "pi_arm_interfaces/action/move_l.hpp"
#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/msg/robot_state.hpp"
#include "pi_arm_interfaces/msg/task_state.hpp"
#include "pi_arm_interfaces/srv/manage_motor.hpp"
#include "pi_arm_interfaces/srv/preview_motion.hpp"
#include "pi_arm_interfaces/srv/set_motion_armed.hpp"
#include "pi_arm_interfaces/srv/set_teaching.hpp"
#include "pi_arm_interfaces/srv/stop_motion.hpp"
#include "pi_arm_manager/management_gate.hpp"
#include "pi_arm_manager/arrival_check.hpp"
#include "pi_arm_manager/direct_move_validation.hpp"
#include "pi_arm_transmission/config_loader.hpp"
#include "pi_arm_transmission/cable_kinematics.hpp"
#include "hardware_interface/component_parser.hpp"
#include "pi_arm_manager/motion_stop_reason.hpp"
#include "pi_arm_manager/state_aggregator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

namespace pi_arm_manager
{
namespace
{

// MoveL / goal-reject TCP caps (SI). Keep numerically aligned with
// pi_arm_moveit_config/config/pilz_cartesian_limits.yaml used by move_group Pilz.
constexpr double kMaxTransVelMps = 0.1;              // 100 mm/s
constexpr double kMaxTransAccMps2 = 0.2;             // 200 mm/s²
constexpr double kMaxRotVelRadps = 0.17453292519943295;      // 10 deg/s
// Pilz-style derived rotational acceleration for goal validation / logging only.
constexpr double kMaxRotAccRadps2 = kMaxTransAccMps2 / kMaxTransVelMps * kMaxRotVelRadps;

constexpr const char * kOmplPipeline = "ompl";
constexpr const char * kPilzPipeline = "pilz_industrial_motion_planner";
constexpr const char * kOmplPlanner = "RRTConnectkConfigDefault";
constexpr const char * kPilzLinPlanner = "LIN";

double clamp_scaling(double value)
{
  // Leave headroom under URDF limits so TOTP/JTC interpolation does not
  // command velocities that barely exceed max_velocity.
  return std::clamp(value, 0.001, 0.95);
}

void validate_timed_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    throw std::runtime_error("planned trajectory is empty");
  }
  double previous_time = -1.0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double t = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
    if (!std::isfinite(t) || t + 1e-9 < previous_time) {
      throw std::runtime_error(
              "planned trajectory has invalid time_from_start at index " + std::to_string(i));
    }
    if (trajectory.points[i].positions.size() != trajectory.joint_names.size()) {
      throw std::runtime_error(
              "planned trajectory point/joint size mismatch at index " + std::to_string(i));
    }
    previous_time = t;
  }
}

}  // namespace

class ManagerNode : public rclcpp::Node
{
public:
  using MoveJ = pi_arm_interfaces::action::MoveJ;
  using MoveJs = pi_arm_interfaces::action::MoveJs;
  using MoveL = pi_arm_interfaces::action::MoveL;
  using DirectMove = pi_arm_interfaces::action::DirectMove;
  using Follow = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
  using GoalHandleMoveJs = rclcpp_action::ServerGoalHandle<MoveJs>;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;
  using GoalHandleDirect = rclcpp_action::ServerGoalHandle<DirectMove>;

  ManagerNode()
  : Node("pi_arm_manager"),
    task_claimed_(false),
    stop_requested_(false),
    stop_reason_(MotionStopReason::NONE),
    next_task_id_(1),
    still_frames_(0),
    linear_velocity_limit_(kMaxTransVelMps),
    linear_acceleration_limit_(kMaxTransAccMps2),
    angular_velocity_limit_(kMaxRotVelRadps),
    angular_acceleration_limit_(kMaxRotAccRadps2)
  {
    hardware_timeout_ = declare_parameter("hardware_timeout_sec", 1.0);
    operation_timeout_ = declare_parameter("operation_timeout_sec", 5.0);
    motion_timeout_ = declare_parameter("motion_timeout_sec", 120.0);
    still_velocity_ = declare_parameter(
      "still_velocity_rad_s", 0.00017453292519943296);
    goal_position_tolerance_ = declare_parameter("goal_position_tolerance_deg", 0.2) * M_PI / 180.0;
    arrival_stable_time_ = declare_parameter("arrival_stable_time_sec", 0.5);
    arrival_timeout_ = declare_parameter("arrival_timeout_sec", 10.0);
    if (!std::isfinite(goal_position_tolerance_) || goal_position_tolerance_ <= 0.0 ||
      !std::isfinite(arrival_stable_time_) || arrival_stable_time_ <= 0.0 ||
      !std::isfinite(arrival_timeout_) || arrival_timeout_ <= arrival_stable_time_) {
      throw std::runtime_error("invalid arrival verification parameters");
    }
    state_rate_ = declare_parameter("state_publish_rate_hz", 10.0);
    follow_action_name_ = declare_parameter(
      "follow_joint_trajectory_action",
      std::string("/joint_trajectory_controller/follow_joint_trajectory"));

    task_.status = pi_arm_interfaces::msg::TaskState::IDLE;
    task_.message = "idle";
    hardware_sub_ = create_subscription<pi_arm_interfaces::msg::HardwareState>(
      declare_parameter("hardware_state_topic", std::string("/pi_arm/hardware_state")), rclcpp::SensorDataQoS(),
      std::bind(&ManagerNode::hardware_callback, this, std::placeholders::_1));
    state_pub_ = create_publisher<pi_arm_interfaces::msg::RobotState>(
      "/pi_arm/state", rclcpp::QoS(10).reliable().transient_local());

    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Reentrant group so worker threads can wait_for() action futures without
    // starving against the default MutuallyExclusive group (timers/subscriptions).
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    enable_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/enable_motor", rmw_qos_profile_services_default, service_group_);
    disable_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/disable_motor", rmw_qos_profile_services_default, service_group_);
    reset_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/reset_motor", rmw_qos_profile_services_default, service_group_);
    motion_arm_client_ = create_client<pi_arm_interfaces::srv::SetMotionArmed>(
      "/pi_arm/hardware/set_motion_armed", rmw_qos_profile_services_default, service_group_);
    reboot_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/reboot_motor", rmw_qos_profile_services_default, service_group_);
    teaching_client_ = create_client<pi_arm_interfaces::srv::SetTeaching>(
      "/pi_arm/hardware/set_teaching", rmw_qos_profile_services_default, service_group_);
    teaching_service_ = create_service<pi_arm_interfaces::srv::SetTeaching>(
      "/pi_arm/set_teaching",
      [this](const std::shared_ptr<pi_arm_interfaces::srv::SetTeaching::Request> request,
        std::shared_ptr<pi_arm_interfaces::srv::SetTeaching::Response> response) {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        pi_arm_interfaces::msg::HardwareState h;
        pi_arm_interfaces::msg::TaskState t;
        {std::lock_guard<std::mutex> lock(state_mutex_); h = hardware_; t = task_;
          if ((now() - last_hardware_time_).seconds() > hardware_timeout_) {h.connected = false;}}
        const auto decision = decide_motor_recovery(h, aggregate_state(h, t),
          task_claimed_.load() || preview_active_.load() || h.axis_test_active,
          raw_still_frames_.load() >= 3, 0);
        if (!decision.allowed || h.commissioning || h.motion_stop_pending || h.motion_stop_failed) {
          response->success = false; response->code = 2001;
          response->message = "drag teaching requires idle, fresh stationary feedback and confirmed stop";
          return;
        }
        if (!teaching_client_->wait_for_service(std::chrono::duration<double>(operation_timeout_))) {
          response->success = false; response->code = 2002; response->message = "teaching service unavailable"; return;
        }
        auto future = teaching_client_->async_send_request(request);
        if (future.wait_for(std::chrono::duration<double>(operation_timeout_)) != std::future_status::ready) {
          response->success = false; response->code = 2002; response->message = "teaching service timeout; inspect feedback"; return;
        }
        *response = *future.get();
      });
    follow_client_ = rclcpp_action::create_client<Follow>(
      this, follow_action_name_, action_group_);

    enable_service_ = make_management_service(
      "/pi_arm/enable_motor", enable_client_, Operation::ENABLE);
    disable_service_ = make_management_service(
      "/pi_arm/disable_motor", disable_client_, Operation::DISABLE);
    reset_service_ = make_management_service(
      "/pi_arm/reset_motor", reset_client_, Operation::RESET);
    reboot_service_ = make_management_service(
      "/pi_arm/reboot_motor", reboot_client_, Operation::REBOOT);
    stop_service_ = create_service<pi_arm_interfaces::srv::StopMotion>(
      "/pi_arm/stop_motion",
      std::bind(
        &ManagerNode::stop_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, state_rate_));
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ManagerNode::publish_state, this));

    RCLCPP_INFO(
      get_logger(),
      "MoveL TCP caps (compile-time): max_trans_vel=%.6f m/s max_trans_acc=%.6f m/s2 "
      "max_rot_vel=%.6f rad/s derived_rot_acc=%.6f rad/s2",
      linear_velocity_limit_, linear_acceleration_limit_,
      angular_velocity_limit_, angular_acceleration_limit_);
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "pi_arm");
    move_group_->setEndEffectorLink("tool0");
    load_joint_limits();
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>(
      "/compute_ik", rmw_qos_profile_services_default, service_group_);
    preview_service_ = create_service<pi_arm_interfaces::srv::PreviewMotion>(
      "/pi_arm/preview_motion",
      std::bind(&ManagerNode::preview_motion, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

    movej_server_ = rclcpp_action::create_server<MoveJ>(
      this, "/pi_arm/movej",
      std::bind(&ManagerNode::goal_movej, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movej, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movej, this, std::placeholders::_1));
    movejs_server_ = rclcpp_action::create_server<MoveJs>(
      this, "/pi_arm/movejs",
      std::bind(&ManagerNode::goal_movejs, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movejs, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movejs, this, std::placeholders::_1));
    movel_server_ = rclcpp_action::create_server<MoveL>(
      this, "/pi_arm/movel",
      std::bind(&ManagerNode::goal_movel, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movel, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movel, this, std::placeholders::_1));
    direct_server_ = rclcpp_action::create_server<DirectMove>(
      this, "/pi_arm/direct_move",
      std::bind(&ManagerNode::goal_direct, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_direct, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_direct, this, std::placeholders::_1));
  }

private:
  enum class Operation {ENABLE, DISABLE, RESET, REBOOT};

  static bool valid_output_speed(double speed)
  {
    return std::isfinite(speed) && (speed == 0.0 || (speed >= 0.002908882086657216 && speed <= kMaxRotVelRadps));
  }

  static double resolved_output_speed(double speed)
  {
    if (!valid_output_speed(speed)) {throw std::runtime_error("invalid output speed ceiling");}
    return speed == 0.0 ? 0.08726646259971647 : speed;
  }

  std::vector<double> direct_targets(const DirectMove::Goal & request) const
  {
    if (!request.output_space) {return request.positions;}
    const std::vector<std::string> names{
      "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
    if (request.joint_names != names || request.positions.size() != 6U) {
      throw std::runtime_error("output target requires six canonical axes");
    }
    pi_arm_transmission::Vector6 output{}, joint{};
    std::copy(request.positions.begin(), request.positions.end(), output.begin());
    if (!cable_->check_output_position_limits(output) ||
      !cable_->output_to_joint_position(output, joint)) {
      throw std::runtime_error("output target outside limits or inverse domain");
    }
    std::vector<double> result(joint.begin(), joint.end());
    validate_direct_positions(joint_limits_, names, result, "output target inverse");
    return result;
  }

  pi_arm_interfaces::msg::HardwareState preview_start()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if ((now() - last_hardware_time_).seconds() > hardware_timeout_ ||
      !hardware_.connected || !hardware_.kinematics_valid ||
      hardware_.joint_names.size() != 6 || hardware_.positions.size() != 6 ||
      hardware_.fresh.size() != 6 || hardware_.motor_fresh.size() != 6 ||
      !std::all_of(hardware_.fresh.begin(), hardware_.fresh.end(), [](bool v) {return v;}) ||
      !std::all_of(hardware_.motor_fresh.begin(), hardware_.motor_fresh.end(), [](bool v) {return v;})) {
      throw std::runtime_error("preview requires fresh complete joint feedback");
    }
    if (hardware_.velocities.size() != 6 ||
      !std::all_of(hardware_.velocities.begin(), hardware_.velocities.end(),
        [this](double v) {return std::isfinite(v) && std::abs(v) <= still_velocity_;})) {
      throw std::runtime_error("preview requires stationary feedback");
    }
    validate_direct_positions(joint_limits_, hardware_.joint_names, hardware_.positions, "preview start", true);
    return hardware_;
  }

  void preview_motion(
    const std::shared_ptr<pi_arm_interfaces::srv::PreviewMotion::Request> request,
    std::shared_ptr<pi_arm_interfaces::srv::PreviewMotion::Response> response)
  {
    // Sharing MoveGroupInterface is safe because previews and motion claims are
    // mutually exclusive. Do not hold command_mutex_ while waiting on MoveIt.
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (task_claimed_.load() || preview_active_.exchange(true)) {
        response->code = 2003;
        response->message = "motion or preview already active";
        return;
      }
    }
    struct Release {std::atomic_bool & flag; ~Release() {flag.store(false);}} release{preview_active_};
    try {
      const auto start = preview_start();
      response->joint_names = start.joint_names;
      response->start_positions = start.positions;
      auto & trajectory = response->trajectory;
      trajectory.joint_names = start.joint_names;
      const auto & mode = request->mode;
      if (mode != "joint" && mode != "output" && mode != "ik" && mode != "lin") {
        throw std::runtime_error("unknown preview mode");
      }
      const auto output_speed = resolved_output_speed(request->max_output_velocity);
      if (mode == "joint" || mode == "output") {
        DirectMove::Goal goal;
        goal.joint_names = start.joint_names;
        goal.positions = request->positions;
        goal.output_space = mode == "output";
        response->target_positions = direct_targets(goal);
        validate_direct_positions(joint_limits_, start.joint_names, response->target_positions, "preview target");
        if (!valid_joint_goal(start.joint_names, response->target_positions, request->max_velocity)) {
          throw std::runtime_error("invalid joint speed");
        }
        double duration = 0.1;
        for (size_t i = 0; i < 6; ++i) {
          duration = std::max(duration,
            std::abs(response->target_positions[i] - start.positions[i]) / request->max_velocity);
        }
        trajectory_msgs::msg::JointTrajectoryPoint end;
        end.positions = response->target_positions;
        end.time_from_start = rclcpp::Duration::from_seconds(duration);
        trajectory.points.push_back(end);
        response->message = "DirectMove joint interpolation; no collision planning";
      } else {
        const auto & p = request->target;
        const auto & q = p.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
          !std::isfinite(p.position.z) || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-5) {
          throw std::runtime_error("invalid base_link/tool0 target pose");
        }
        if (mode == "ik") {
          auto ik = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
          ik->ik_request.group_name = "pi_arm";
          ik->ik_request.ik_link_name = "tool0";
          ik->ik_request.avoid_collisions = true;
          ik->ik_request.pose_stamped.header.frame_id = "base_link";
          ik->ik_request.pose_stamped.pose = p;
          ik->ik_request.robot_state.joint_state.name = start.joint_names;
          ik->ik_request.robot_state.joint_state.position = start.positions;
          for (size_t i = 0; i < 6; ++i) {
            const auto & bound = joint_limits_.at(start.joint_names[i]);
            ik->ik_request.robot_state.joint_state.position[i] =
              std::clamp(start.positions[i], bound.lower, bound.upper);
          }
          ik->ik_request.timeout.sec = 2;
          if (!ik_client_->wait_for_service(1s)) {throw std::runtime_error("IK service unavailable");}
          auto future = ik_client_->async_send_request(ik);
          if (future.wait_for(4s) != std::future_status::ready) {
            ik_client_->remove_pending_request(future);
            throw std::runtime_error("IK timeout");
          }
          const auto result = future.get();
          if (result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            throw std::runtime_error("IK failed: " + std::to_string(result->error_code.val));
          }
          const auto & joints = result->solution.joint_state;
          for (const auto & name : start.joint_names) {
            const auto found = std::find(joints.name.begin(), joints.name.end(), name);
            if (found == joints.name.end()) {throw std::runtime_error("incomplete IK result");}
            response->target_positions.push_back(joints.position.at(std::distance(joints.name.begin(), found)));
          }
          response->message = "IK endpoint with scene collision check; not a path";
        } else {
          if (!std::isfinite(request->max_linear_velocity) || request->max_linear_velocity <= 0 ||
            request->max_linear_velocity > linear_velocity_limit_ ||
            !std::isfinite(request->max_linear_acceleration) || request->max_linear_acceleration <= 0 ||
            request->max_linear_acceleration > linear_acceleration_limit_ ||
            !std::isfinite(request->max_angular_velocity) || request->max_angular_velocity <= 0 ||
            request->max_angular_velocity > angular_velocity_limit_) {
            throw std::runtime_error("invalid Cartesian speed/acceleration");
          }
          move_group_->clearPoseTargets();
          move_group_->setPlanningPipelineId(kPilzPipeline);
          move_group_->setPlannerId(kPilzLinPlanner);
          move_group_->setPoseReferenceFrame("base_link");
          move_group_->setPoseTarget(p, "tool0");
          move_group_->setMaxVelocityScalingFactor(clamp_scaling(std::min(
              request->max_linear_velocity / linear_velocity_limit_,
              request->max_angular_velocity / angular_velocity_limit_)));
          move_group_->setMaxAccelerationScalingFactor(clamp_scaling(
              request->max_linear_acceleration / linear_acceleration_limit_));
          prepare_planning_start();
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          const auto code = move_group_->plan(plan);
          move_group_->clearPoseTargets();
          if (code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("LIN planning failed: " + std::to_string(code.val));
          }
          trajectory = plan.trajectory.joint_trajectory;
          if (trajectory.points.empty()) {throw std::runtime_error("empty LIN trajectory");}
          response->target_positions = trajectory.points.back().positions;
          response->message = "Pilz LIN path; execution will replan from current feedback";
        }
      }
      validate_direct_positions(joint_limits_, start.joint_names, response->target_positions, "preview inverse");
      pi_arm_transmission::Vector6 target{}, output{};
      std::copy(response->target_positions.begin(), response->target_positions.end(), target.begin());
      if (!cable_->joint_to_output_position(target, output) || !cable_->check_output_position_limits(output)) {
        throw std::runtime_error("preview output limit exceeded");
      }
      response->output_positions.assign(output.begin(), output.end());
      moveit::core::RobotState model(move_group_->getRobotModel());
      model.setToDefaultValues();
      const auto fk = [&model, &start](const std::vector<double> & positions) {
          for (size_t i = 0; i < 6; ++i) {model.setVariablePosition(start.joint_names[i], positions[i]);}
          model.update();
          const auto & transform = model.getGlobalLinkTransform("tool0");
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.translation().x();
          pose.position.y = transform.translation().y();
          pose.position.z = transform.translation().z();
          Eigen::Quaterniond q(transform.rotation());
          pose.orientation.x = q.x(); pose.orientation.y = q.y();
          pose.orientation.z = q.z(); pose.orientation.w = q.w();
          return pose;
        };
      response->target_pose = fk(response->target_positions);
      if (!trajectory.points.empty()) {
        prepare_trajectory(trajectory, output_speed);
        const auto original = trajectory.points;
        trajectory.points.clear();
        const double duration = rclcpp::Duration(original.back().time_from_start).seconds();
        // Sample the exact position-linear JTC segments for display, not a new plan.
        size_t segment = 0;
        for (int sample = 0; sample <= 200; ++sample) {
          const double t = duration * sample / 200.0;
          while (segment + 1 < original.size() &&
            rclcpp::Duration(original[segment].time_from_start).seconds() < t) {++segment;}
          const double before_t = segment == 0 ? 0.0 :
            rclcpp::Duration(original[segment - 1].time_from_start).seconds();
          const auto & before = segment == 0 ? start.positions : original[segment - 1].positions;
          const double after_t = rclcpp::Duration(original[segment].time_from_start).seconds();
          const double u = std::clamp((t - before_t) / std::max(1e-9, after_t - before_t), 0.0, 1.0);
          trajectory_msgs::msg::JointTrajectoryPoint point;
          for (size_t i = 0; i < 6; ++i) {
            point.positions.push_back(before[i] + u * (original[segment].positions[i] - before[i]));
          }
          point.time_from_start = rclcpp::Duration::from_seconds(t);
          response->tcp_path.push_back(fk(point.positions).position);
          trajectory.points.push_back(point);
        }
      }
      const auto end = preview_start();
      if (end.joint_names != start.joint_names) {throw std::runtime_error("preview feedback order changed");}
      for (size_t i = 0; i < 6; ++i) {
        if (std::abs(end.positions[i] - start.positions[i]) > goal_position_tolerance_) {
          throw std::runtime_error("preview start changed; calculate again");
        }
      }
      response->success = true;
    } catch (const std::exception & error) {
      *response = pi_arm_interfaces::srv::PreviewMotion::Response();
      response->code = 2002;
      response->message = error.what();
    }
  }

  void prepare_planning_start()
  {
    std::vector<std::string> names;
    std::vector<double> positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      names = hardware_.joint_names;
      positions = hardware_.positions;
    }
    validate_direct_positions(joint_limits_, names, positions, "planning start", true);
    moveit::core::RobotState start(move_group_->getRobotModel());
    start.setToDefaultValues();
    for (size_t i = 0; i < names.size(); ++i) {
      const auto & bound = joint_limits_.at(names[i]);
      start.setVariablePosition(names[i], std::clamp(positions[i], bound.lower, bound.upper));
    }
    start.update();
    move_group_->setStartState(start);
  }

  void prepare_trajectory(trajectory_msgs::msg::JointTrajectory & trajectory, double output_speed)
  {
    validate_timed_trajectory(trajectory);
    std::vector<double> previous;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (trajectory.joint_names != hardware_.joint_names) {
        throw std::runtime_error("trajectory requires canonical complete joint order");
      }
      previous = hardware_.positions;
    }
    validate_direct_positions(joint_limits_, trajectory.joint_names, previous, "trajectory start", true);
    pi_arm_transmission::Matrix6 gain{};
    cable_->jacobian({}, gain);
    double old_time = 0.0, new_time = 0.0;
    for (auto & point : trajectory.points) {
      validate_direct_positions(joint_limits_, trajectory.joint_names, point.positions, "trajectory target");
      pi_arm_transmission::Vector6 target{}, output{};
      std::copy(point.positions.begin(), point.positions.end(), target.begin());
      if (!cable_->joint_to_output_position(target, output) ||
        !cable_->check_output_position_limits(output)) {
        throw std::runtime_error("trajectory output limit exceeded");
      }
      const double time = rclcpp::Duration(point.time_from_start).seconds();
      double duration = std::max(0.001, time - old_time);
      for (size_t i = 0; i < 6; ++i) {
        double distance_bound = 0.0;
        for (size_t j = 0; j < 6; ++j) {
          distance_bound += std::abs(gain[i][j]) * std::abs(point.positions[j] - previous[j]);
        }
        const double encoded_speed = std::min(65535.0, std::floor(
            output_speed * 180.0 / M_PI * protocol_ratios_[i] + 1e-10));
        if (encoded_speed < 1.0) {throw std::runtime_error("output speed below protocol resolution");}
        const double effective_speed = encoded_speed / protocol_ratios_[i] * M_PI / 180.0;
        duration = std::max(duration, distance_bound / effective_speed);
        duration = std::max(duration, std::abs(point.positions[i] - previous[i]) /
          joint_limits_.at(trajectory.joint_names[i]).velocity);
      }
      // Position-only waypoints select linear JTC interpolation: no spline
      // overshoot into the feedback allowance. |J(0)| bounds cable speed on
      // each linear segment, including the chord axes.
      point.velocities.clear();
      point.accelerations.clear();
      point.effort.clear();
      new_time += duration;
      if (!std::isfinite(new_time)) {throw std::runtime_error("invalid trajectory duration");}
      point.time_from_start = rclcpp::Duration::from_seconds(new_time);
      old_time = time;
      previous = point.positions;
    }
    trajectory.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  // Reads position/velocity bounds from the robot model (URDF) and acceleration
  // bounds from the robot_description_planning joint limits that the launch
  // file loads from joint_limits.yaml. Called once before the action servers
  // are created; joint_limits_ is read-only afterwards.
  void load_joint_limits()
  {
    const auto controls = hardware_interface::parse_control_resources_from_urdf(
      get_parameter("robot_description").as_string());
    if (controls.size() != 1U) {throw std::runtime_error("expected one hardware configuration");}
    const auto & control = controls.front();
    if (control.joints.size() != 6U) {throw std::runtime_error("expected six actuator mappings");}
    for (size_t i = 0; i < 6; ++i) {
      protocol_ratios_[i] = std::stod(control.joints[i].parameters.at("protocol_angle_ratio"));
      if (!std::isfinite(protocol_ratios_[i]) || protocol_ratios_[i] <= 0.0) {
        throw std::runtime_error("invalid protocol angle ratio");
      }
    }
    cable_ = std::make_unique<pi_arm_transmission::CableKinematics>(
      pi_arm_transmission::load_cable_kinematics_config(
        control.hardware_parameters.at("cable_kinematics_file")));
    const auto robot_model = move_group_->getRobotModel();
    joint_limits_.clear();
    max_joint_velocity_limit_ = 0.0;
    max_joint_acceleration_limit_ = 0.0;
    for (const auto & name : move_group_->getActiveJoints()) {
      const auto * joint_model = robot_model->getJointModel(name);
      if (joint_model == nullptr || joint_model->getVariableBounds().empty()) {
        throw std::runtime_error("robot model has no bounds for joint " + name);
      }
      const auto & bounds = joint_model->getVariableBounds().front();
      if (!bounds.position_bounded_ || !bounds.velocity_bounded_ ||
        bounds.max_velocity_ <= 0.0)
      {
        throw std::runtime_error(
                "URDF must provide position and velocity limits for joint " + name);
      }
      if (!bounds.acceleration_bounded_ || bounds.max_acceleration_ <= 0.0) {
        throw std::runtime_error(
                "joint " + name + " has no acceleration limit; load joint_limits.yaml "
                "under the robot_description_planning namespace");
      }
      JointLimits limits;
      limits.lower = bounds.min_position_;
      limits.upper = bounds.max_position_;
      limits.velocity = bounds.max_velocity_;
      limits.acceleration = bounds.max_acceleration_;
      for (const auto & joint : control.joints) {
        if (joint.name == name && joint.parameters.count("feedback_position_tolerance_rad")) {
          limits.feedback_tolerance = std::stod(joint.parameters.at("feedback_position_tolerance_rad"));
          if (!std::isfinite(limits.feedback_tolerance) || limits.feedback_tolerance < 0.0) {
            throw std::runtime_error("invalid feedback tolerance");
          }
        }
      }
      joint_limits_.emplace(name, limits);
      max_joint_velocity_limit_ = std::max(max_joint_velocity_limit_, limits.velocity);
      max_joint_acceleration_limit_ = std::max(max_joint_acceleration_limit_, limits.acceleration);
    }
    if (joint_limits_.empty()) {
      throw std::runtime_error("planning group pi_arm has no active joints");
    }
  }

  double min_velocity_limit(const std::vector<std::string> & names) const
  {
    double limit = max_joint_velocity_limit_;
    for (const auto & name : names) {
      const auto found = joint_limits_.find(name);
      if (found == joint_limits_.end()) {
        return 0.0;
      }
      limit = std::min(limit, found->second.velocity);
    }
    return limit;
  }

  double min_acceleration_limit(const std::vector<std::string> & names) const
  {
    double limit = max_joint_acceleration_limit_;
    for (const auto & name : names) {
      const auto found = joint_limits_.find(name);
      if (found == joint_limits_.end()) {
        return 0.0;
      }
      limit = std::min(limit, found->second.acceleration);
    }
    return limit;
  }

  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr make_management_service(
    const std::string & name,
    const rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr & client,
    Operation operation)
  {
    return create_service<pi_arm_interfaces::srv::ManageMotor>(
      name,
      [this, client, operation, name](
        const std::shared_ptr<pi_arm_interfaces::srv::ManageMotor::Request> request,
        std::shared_ptr<pi_arm_interfaces::srv::ManageMotor::Response> response)
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        pi_arm_interfaces::msg::HardwareState hardware_snapshot;
        pi_arm_interfaces::msg::TaskState task_snapshot;
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          hardware_snapshot = hardware_;
          task_snapshot = task_;
          if ((now() - last_hardware_time_).seconds() > hardware_timeout_) {
            hardware_snapshot.connected = false;
          }
        }
        const auto state = aggregate_state(hardware_snapshot, task_snapshot);
        const bool still = (operation != Operation::ENABLE ?
          raw_still_frames_.load() : still_frames_.load()) >= 3;
        const bool task_claimed = task_claimed_.load();
        bool allowed = false;
        int rejection_code = 2001;
        std::string rejection_message = "operation rejected by robot state";
        if (operation == Operation::ENABLE || operation == Operation::DISABLE) {
          const auto decision = decide_motor_power_operation(
            operation == Operation::ENABLE ?
            MotorPowerOperation::ENABLE : MotorPowerOperation::DISABLE,
            hardware_snapshot, state, task_claimed, still, request->motor_id);
          if (decision.already_satisfied) {
            response->success = true;
            response->code = decision.code;
            response->message = decision.message;
            RCLCPP_INFO(
              get_logger(), "service %s idempotent: motor_id=%d message=%s",
              name.c_str(), request->motor_id, response->message.c_str());
            return;
          }
          allowed = decision.allowed;
          rejection_code = decision.code;
          rejection_message = decision.message;
        } else {
          const auto decision = decide_motor_recovery(hardware_snapshot, state,
              task_claimed || preview_active_.load(), still, request->motor_id);
          allowed = decision.allowed;
          rejection_code = decision.code;
          rejection_message = decision.message;
          if (operation == Operation::REBOOT && request->motor_id <= 0) {
            allowed = false;
            rejection_code = 1004;
            rejection_message = "reboot requires one explicit motor_id";
          }
        }
        if (!allowed) {
          response->success = false;
          response->code = rejection_code;
          response->message = rejection_message;
          RCLCPP_WARN(
            get_logger(),
            "service %s REJECTED: motor_id=%d state=%s still=%d task_claimed=%d",
            name.c_str(), request->motor_id, state_name(state),
            still ? 1 : 0, task_claimed ? 1 : 0);
          return;
        }
        if (!client->wait_for_service(std::chrono::duration<double>(operation_timeout_))) {
          response->success = false;
          response->code = 2002;
          response->message = "hardware service unavailable";
          RCLCPP_ERROR(
            get_logger(), "service %s FAILED: hardware backend unavailable", name.c_str());
          return;
        }
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::duration<double>(operation_timeout_)) !=
          std::future_status::ready)
        {
          response->success = false;
          response->code = 2002;
          response->message = "hardware service timeout";
          RCLCPP_ERROR(
            get_logger(), "service %s FAILED: hardware backend timeout", name.c_str());
          return;
        }
        *response = *future.get();
        if (response->success) {
          RCLCPP_INFO(
            get_logger(), "service %s ok: motor_id=%d message=%s",
            name.c_str(), request->motor_id, response->message.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "service %s backend failed: motor_id=%d code=%d message=%s",
            name.c_str(), request->motor_id, response->code, response->message.c_str());
        }
      },
      rmw_qos_profile_services_default, service_group_);
  }

  std::uint64_t active_task_id()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return task_.task_id;
  }

  bool set_motion_armed(
    const bool armed, std::uint64_t lease_id, const std::string & stage,
    double max_output_velocity = 0.0, bool hold_position = false)
  {
    // Serialize arm/release RPCs, including scope guards racing with Stop.
    std::lock_guard<std::mutex> gate_lock(motion_gate_mutex_);
    if (armed && stop_requested_.load()) {return false;}
    if (!armed && stop_requested_.load()) {
      if (holds_position_on_stop(stop_reason_.load()) && lease_id != 0U) {
        lease_id = 0U;
        hold_position = true;
      } else if (!holds_position_on_stop(stop_reason_.load()) &&
        stop_reason_.load() != MotionStopReason::NONE)
      {
        lease_id = 0U;
        hold_position = false;
      }
    }
    if (!armed && lease_id == 0U) {stop_hold_confirmed_.store(false);}
    if (!motion_arm_client_->wait_for_service(
        std::chrono::duration<double>(operation_timeout_)))
    {
      RCLCPP_ERROR(
        get_logger(), "motion gate service unavailable at %s", stage.c_str());
      return false;
    }
    auto request = std::make_shared<pi_arm_interfaces::srv::SetMotionArmed::Request>();
    request->armed = armed;
    request->hold_position = hold_position;
    request->max_output_velocity = max_output_velocity;
    request->lease_id = lease_id;
    auto future = motion_arm_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(operation_timeout_)) !=
      std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "motion gate timeout at %s", stage.c_str());
      return false;
    }
    const auto response = future.get();
    if (!response->success || response->armed != armed) {
      RCLCPP_ERROR(
        get_logger(), "motion gate rejected at %s: success=%d armed=%d message=%s",
        stage.c_str(), response->success ? 1 : 0, response->armed ? 1 : 0,
        response->message.c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "motion gate armed=%d lease=%llu at %s: %s",
      armed ? 1 : 0, static_cast<unsigned long long>(lease_id), stage.c_str(),
      response->message.c_str());
    if (!armed && lease_id == 0U && hold_position) {stop_hold_confirmed_.store(true);}
    return true;
  }

  void hardware_callback(const pi_arm_interfaces::msg::HardwareState::SharedPtr message)
  {
    const bool complete_motion_state =
      !message->joint_names.empty() &&
      message->velocities.size() == message->joint_names.size() &&
      message->moving.size() == message->joint_names.size() &&
      message->fresh.size() == message->joint_names.size() &&
      std::all_of(message->fresh.begin(), message->fresh.end(), [](bool v) {return v;});
    const bool no_motion = complete_motion_state &&
      !std::any_of(message->moving.begin(), message->moving.end(), [](bool v) {return v;}) &&
      std::all_of(
      message->velocities.begin(), message->velocities.end(),
      [this](double v) {return std::isfinite(v) && std::abs(v) <= still_velocity_;});
    still_frames_.store(no_motion ? std::min(3, still_frames_.load() + 1) : 0);
    // Joint feedback can be invalid when cable inversion fails. Power-off
    // relies on the independent, fresh motor motion flags in that case.
    const bool raw_still = message->connected && complete_raw_motor_state(*message) &&
      std::all_of(message->motor_fresh.begin(), message->motor_fresh.end(),
      [](bool fresh) {return fresh;}) &&
      std::none_of(message->motor_moving.begin(), message->motor_moving.end(),
      [](bool moving) {return moving;});
    raw_still_frames_.store(raw_still ? std::min(3, raw_still_frames_.load() + 1) : 0);
    MotionStopReason hardware_stop_reason = MotionStopReason::NONE;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_ = *message;
      last_hardware_time_ = now();
      ++hardware_sequence_;
      if (task_claimed_.load() && task_.joint_names == hardware_.joint_names &&
        task_.target_positions.size() == hardware_.positions.size()) {
        task_.position_errors.clear();
        for (size_t i = 0; i < task_.target_positions.size(); ++i) {
          task_.position_errors.push_back(task_.target_positions[i] - hardware_.positions[i]);
        }
      }
      const auto state = aggregate_state(hardware_, task_);
      if (task_claimed_.load()) {
        hardware_stop_reason = motion_stop_reason_for_robot_state(state);
      }
    }
    if (hardware_stop_reason != MotionStopReason::NONE) {
      RCLCPP_WARN(
        get_logger(),
        "stopping active task due to hardware state change: %s",
        motion_stop_reason_name(hardware_stop_reason));
      request_stop(hardware_stop_reason);
    }
  }

  uint8_t current_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto hardware = hardware_;
    if ((now() - last_hardware_time_).seconds() > hardware_timeout_) {
      hardware.connected = false;
    }
    const auto state = aggregate_state(hardware, task_);
    // Only hold RUNNING for settling while a motion claim is active. After
    // FAILED/CANCELED/SUCCEEDED the claim is released; still_frames must not
    // fake RUNNING or subsequent goals are blocked as "busy".
    if (state == pi_arm_interfaces::msg::RobotState::READY &&
      task_claimed_.load() && still_frames_.load() < 3)
    {
      return pi_arm_interfaces::msg::RobotState::RUNNING;
    }
    return state;
  }

  void publish_state()
  {
    pi_arm_interfaces::msg::RobotState message;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      message.hardware = hardware_;
      stale = (now() - last_hardware_time_).seconds() > hardware_timeout_;
      message.task = task_;
    }
    message.stamp = now();
    auto state_input = message.hardware;
    if (stale) {
      state_input.connected = false;
    }
    message.state = aggregate_state(state_input, message.task);
    if (message.state == pi_arm_interfaces::msg::RobotState::READY &&
      task_claimed_.load() && still_frames_.load() < 3)
    {
      message.state = pi_arm_interfaces::msg::RobotState::RUNNING;
    }
    message.state_name = state_name(message.state);
    state_pub_->publish(message);
  }

  bool claim_goal(const std::string & command)
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (preview_active_.load()) {
      RCLCPP_WARN(get_logger(), "motion rejected while read-only preview is calculating");
      return false;
    }
    const auto state = current_state();
    if (state != pi_arm_interfaces::msg::RobotState::READY) {
      RCLCPP_WARN(
        get_logger(), "%s REJECTED: robot not READY (state=%s still_frames=%d task_claimed=%d)",
        command.c_str(), state_name(state), still_frames_.load(),
        task_claimed_.load() ? 1 : 0);
      return false;
    }
    bool expected = false;
    if (!task_claimed_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(
        get_logger(), "%s REJECTED: another task is already claimed", command.c_str());
      return false;
    }
    begin_task(command);
    uint64_t task_id = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_id = task_.task_id;
    }
    RCLCPP_INFO(
      get_logger(), "%s ACCEPTED: task_id=%lu",
      command.c_str(), static_cast<unsigned long>(task_id));
    return true;
  }

  void begin_task(const std::string & command)
  {
    stop_requested_.store(false);
    stop_hold_confirmed_.store(false);
    stop_reason_.store(MotionStopReason::NONE);
    still_frames_.store(0);
    std::lock_guard<std::mutex> lock(state_mutex_);
    task_.task_id = next_task_id_.fetch_add(1);
    task_.command = command;
    task_.status = pi_arm_interfaces::msg::TaskState::RUNNING;
    task_.progress = 0.0F;
    task_.code = 0;
    task_.message = "running";
    task_.joint_names.clear();
    task_.target_positions.clear();
    task_.position_errors.clear();
    task_.goal_position_tolerance = goal_position_tolerance_;
    task_.arrival_state = "planning";
  }

  void finish_task(uint8_t status, int32_t code, const std::string & message)
  {
    uint64_t task_id = 0;
    std::string command;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_id = task_.task_id;
      command = task_.command;
      task_.status = status;
      task_.progress = status == pi_arm_interfaces::msg::TaskState::SUCCEEDED ? 1.0F : task_.progress;
      task_.code = code;
      task_.message = message;
      if (status == pi_arm_interfaces::msg::TaskState::CANCELED) {task_.arrival_state = "canceled";}
      else if (status == pi_arm_interfaces::msg::TaskState::FAILED &&
        task_.arrival_state != "not_reached") {task_.arrival_state = "failed";}
    }
    task_claimed_.store(false);
    if (status == pi_arm_interfaces::msg::TaskState::SUCCEEDED) {
      RCLCPP_INFO(
        get_logger(), "task finished ok: id=%lu cmd=%s",
        static_cast<unsigned long>(task_id), command.c_str());
    } else if (status == pi_arm_interfaces::msg::TaskState::CANCELED) {
      RCLCPP_WARN(
        get_logger(), "task canceled: id=%lu cmd=%s message=%s",
        static_cast<unsigned long>(task_id), command.c_str(), message.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "task failed: id=%lu cmd=%s code=%d message=%s",
        static_cast<unsigned long>(task_id), command.c_str(), code, message.c_str());
    }
  }

  bool wait_still()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(motion_timeout_));
    auto next_log = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (stop_requested_.load()) {
        return false;
      }
      if (still_frames_.load() >= 3) {
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_log) {
        RCLCPP_INFO(
          get_logger(), "wait_still: still_frames=%d remain=%.1fs",
          still_frames_.load(),
          std::chrono::duration<double>(deadline - now).count());
        next_log = now + 1s;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool wait_arrival(const std::string & stage)
  {
    ArrivalWindow window(arrival_stable_time_);
    const auto start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.arrival_state = stage;
    }
    while (rclcpp::ok() && !stop_requested_.load()) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= arrival_timeout_) {break;}
      ArrivalCheck check;
      std::uint64_t sequence;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const bool fresh = (now() - last_hardware_time_).seconds() <= hardware_timeout_ &&
          hardware_.connected && hardware_.kinematics_valid &&
          hardware_.fresh.size() == task_.joint_names.size() &&
          std::all_of(hardware_.fresh.begin(), hardware_.fresh.end(), [](bool v) {return v;}) &&
          hardware_.motor_fresh.size() == task_.joint_names.size() &&
          std::all_of(hardware_.motor_fresh.begin(), hardware_.motor_fresh.end(), [](bool v) {return v;});
        check = check_arrival(task_.joint_names, task_.target_positions, hardware_.joint_names,
          hardware_.positions, hardware_.velocities, fresh, goal_position_tolerance_, still_velocity_);
        sequence = hardware_sequence_;
        task_.position_errors = check.errors;
      }
      if (window.update(check.valid && check.reached, elapsed, sequence)) {return true;}
      std::this_thread::sleep_for(20ms);
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    task_.arrival_state = "not_reached";
    return false;
  }

  std::string arrival_error(const std::string & stage)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::ostringstream text;
    text << "GOAL_NOT_REACHED (" << stage << "), tolerance=" << goal_position_tolerance_ * 180.0 / M_PI << "deg; errors:";
    if (task_.position_errors.empty()) {text << " feedback unavailable";}
    for (size_t i = 0; i < task_.position_errors.size(); ++i) {
      text << " J" << i + 1 << "=" << task_.position_errors[i] * 180.0 / M_PI << "deg";
    }
    return text.str();
  }

  bool valid_joint_goal(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    double velocity) const
  {
    if (names.empty() || names.size() != positions.size() ||
      !std::isfinite(velocity) || velocity <= 0.0 ||
      !std::all_of(positions.begin(), positions.end(), [](double v) {return std::isfinite(v);}))
    {
      return false;
    }
    for (size_t i = 0; i < names.size(); ++i) {
      const auto found = joint_limits_.find(names[i]);
      if (names[i].empty() ||
        found == joint_limits_.end() ||
        positions[i] < found->second.lower ||
        positions[i] > found->second.upper ||
        velocity > found->second.velocity + 1e-6 ||
        std::find(names.begin() + static_cast<std::ptrdiff_t>(i + 1), names.end(), names[i]) !=
        names.end())
      {
        return false;
      }
    }
    return true;
  }

  bool valid_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    const auto & names = trajectory.joint_names;
    if (names.empty() || trajectory.points.empty() || trajectory.points.size() > 50000U) {
      return false;
    }
    std::vector<double> previous_positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.joint_names.empty() ||
        hardware_.positions.size() != hardware_.joint_names.size())
      {
        return false;
      }
      previous_positions.reserve(names.size());
      for (const auto & name : names) {
        const auto found = std::find(
          hardware_.joint_names.begin(), hardware_.joint_names.end(), name);
        if (found == hardware_.joint_names.end()) {
          return false;
        }
        previous_positions.push_back(
          hardware_.positions[static_cast<std::size_t>(
              std::distance(hardware_.joint_names.begin(), found))]);
      }
    }
    const double start_velocity_limit = min_velocity_limit(names);
    if (start_velocity_limit <= 0.0) {return false;}
    for (size_t i = 0; i < names.size(); ++i) {
      const auto & bound = joint_limits_.at(names[i]);
      if (!std::isfinite(previous_positions[i]) ||
        previous_positions[i] < bound.lower - bound.feedback_tolerance ||
        previous_positions[i] > bound.upper + bound.feedback_tolerance) {return false;}
    }
    std::vector<double> velocity_limits(names.size(), 0.0);
    std::vector<double> acceleration_limits(names.size(), 0.0);
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto & limits = joint_limits_.at(names[index]);
      velocity_limits[index] = limits.velocity;
      acceleration_limits[index] = limits.acceleration;
    }

    double previous_time = 0.0;
    std::vector<double> previous_segment_velocity(names.size(), 0.0);
    bool have_segment_velocity = false;
    for (const auto & point : trajectory.points) {
      const double time =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;
      const double dt = time - previous_time;
      if (dt <= 0.0 ||
        !valid_joint_goal(names, point.positions, start_velocity_limit))
      {
        return false;
      }
      if ((!point.velocities.empty() && point.velocities.size() != names.size()) ||
        (!point.accelerations.empty() && point.accelerations.size() != names.size()))
      {
        return false;
      }
      std::vector<double> segment_velocity(names.size(), 0.0);
      for (std::size_t index = 0; index < names.size(); ++index) {
        segment_velocity[index] =
          (point.positions[index] - previous_positions[index]) / dt;
        if (std::abs(segment_velocity[index]) > velocity_limits[index] + 1e-6 ||
          (!point.velocities.empty() &&
          (!std::isfinite(point.velocities[index]) ||
          std::abs(point.velocities[index]) > velocity_limits[index] + 1e-6)) ||
          (!point.accelerations.empty() &&
          (!std::isfinite(point.accelerations[index]) ||
          std::abs(point.accelerations[index]) > acceleration_limits[index] + 1e-6)))
        {
          return false;
        }
        if (have_segment_velocity &&
          std::abs(segment_velocity[index] - previous_segment_velocity[index]) / dt >
          acceleration_limits[index] + 1e-6)
        {
          return false;
        }
      }
      previous_positions = point.positions;
      previous_segment_velocity = std::move(segment_velocity);
      have_segment_velocity = true;
      previous_time = time;
    }
    return true;
  }

  rclcpp_action::GoalResponse goal_movej(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJ::Goal> goal)
  {
    if (!valid_output_speed(goal->max_output_velocity) ||
      !valid_joint_goal(goal->joint_names, goal->positions, goal->max_velocity) ||
      !std::isfinite(goal->max_acceleration) || goal->max_acceleration <= 0.0 ||
      goal->max_acceleration > min_acceleration_limit(goal->joint_names) + 1e-6)
    {
      RCLCPP_WARN(
        get_logger(),
        "movej REJECTED: invalid goal (joints=%zu max_v=%.6f max_a=%.6f a_limit=%.6f)",
        goal->joint_names.size(), goal->max_velocity, goal->max_acceleration,
        min_acceleration_limit(goal->joint_names));
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movej") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_movejs(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJs::Goal> goal)
  {
    if (!valid_output_speed(goal->max_output_velocity) || !valid_trajectory(goal->trajectory)) {
      RCLCPP_WARN(
        get_logger(),
        "movejs REJECTED: invalid trajectory (joints=%zu points=%zu)",
        goal->trajectory.joint_names.size(), goal->trajectory.points.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movejs") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_movel(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveL::Goal> goal)
  {
    const auto & position = goal->target.pose.position;
    const auto & orientation = goal->target.pose.orientation;
    const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    const char * reason = nullptr;
    if (!valid_output_speed(goal->max_output_velocity)) {
      reason = "invalid output speed limit";
    } else if (!std::isfinite(goal->max_linear_velocity) || goal->max_linear_velocity <= 0.0 ||
      goal->max_linear_velocity > linear_velocity_limit_ + 1e-6)
    {
      reason = "invalid or out-of-range max_linear_velocity";
    } else if (!std::isfinite(goal->max_linear_acceleration) ||
      goal->max_linear_acceleration <= 0.0 ||
      goal->max_linear_acceleration > linear_acceleration_limit_ + 1e-6)
    {
      reason = "invalid or out-of-range max_linear_acceleration";
    } else if (!std::isfinite(goal->max_angular_velocity) || goal->max_angular_velocity <= 0.0 ||
      goal->max_angular_velocity > angular_velocity_limit_ + 1e-6)
    {
      reason = "invalid or out-of-range max_angular_velocity";
    } else if (!std::isfinite(goal->max_angular_acceleration) ||
      goal->max_angular_acceleration <= 0.0 ||
      goal->max_angular_acceleration > angular_acceleration_limit_ + 1e-6)
    {
      reason = "invalid or out-of-range max_angular_acceleration";
    } else if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      std::abs(quaternion_norm - 1.0) > 1e-3)
    {
      reason = "invalid target pose / quaternion";
    }
    if (reason != nullptr) {
      RCLCPP_WARN(
        get_logger(),
        "movel REJECTED: %s target=(%.4f,%.4f,%.4f) lin_v=%.6f lin_a=%.6f "
        "ang_v=%.6f ang_a=%.6f quat_norm=%.6f limits lin_v=%.6f lin_a=%.6f ang_v=%.6f ang_a=%.6f",
        reason, position.x, position.y, position.z,
        goal->max_linear_velocity, goal->max_linear_acceleration,
        goal->max_angular_velocity, goal->max_angular_acceleration, quaternion_norm,
        linear_velocity_limit_, linear_acceleration_limit_,
        angular_velocity_limit_, angular_acceleration_limit_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movel") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_direct(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const DirectMove::Goal> goal)
  {
    std::vector<double> targets;
    try {targets = direct_targets(*goal);} catch (const std::exception &) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!valid_output_speed(goal->max_output_velocity) ||
      !valid_joint_goal(goal->joint_names, targets, goal->max_velocity)) {
      RCLCPP_WARN(
        get_logger(),
        "direct_move REJECTED: invalid goal (joints=%zu max_v=%.6f)",
        goal->joint_names.size(), goal->max_velocity);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("direct_move") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse cancel_common(const std::shared_ptr<GoalHandleT>)
  {
    RCLCPP_WARN(get_logger(), "motion cancel requested");
    request_stop(MotionStopReason::ACTION_CANCEL);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse cancel_movej(const std::shared_ptr<GoalHandleMoveJ> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_movejs(const std::shared_ptr<GoalHandleMoveJs> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_movel(const std::shared_ptr<GoalHandleMoveL> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_direct(const std::shared_ptr<GoalHandleDirect> goal)
  {
    return cancel_common(goal);
  }

  void accept_movej(const std::shared_ptr<GoalHandleMoveJ> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movej(goal);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(self->get_logger(), "movej exception: %s", error.what());
          self->complete_action<MoveJ>(goal, false, "movej", error.what());
        }
      }).detach();
  }
  void accept_movejs(const std::shared_ptr<GoalHandleMoveJs> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movejs(goal);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(self->get_logger(), "movejs exception: %s", error.what());
          self->complete_action<MoveJs>(goal, false, "movejs", error.what());
        }
      }).detach();
  }
  void accept_movel(const std::shared_ptr<GoalHandleMoveL> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movel(goal);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(self->get_logger(), "movel exception: %s", error.what());
          self->complete_action<MoveL>(goal, false, "movel", error.what());
        }
      }).detach();
  }
  void accept_direct(const std::shared_ptr<GoalHandleDirect> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_direct(goal);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(self->get_logger(), "direct_move exception: %s", error.what());
          self->complete_action<DirectMove>(goal, false, "direct_move", error.what());
        }
      }).detach();
  }

  void execute_movej(const std::shared_ptr<GoalHandleMoveJ> handle)
  {
    publish_feedback<MoveJ>(handle, 0.0F, "running");
    // Dual-pipeline safety: movel switches to Pilz; pin OMPL for joint-space motion.
    move_group_->setPlanningPipelineId(kOmplPipeline);
    move_group_->setPlannerId(kOmplPlanner);
    RCLCPP_INFO(
      get_logger(), "movej using pipeline=%s planner=%s", kOmplPipeline, kOmplPlanner);
    const auto goal = handle->get_goal();
    std::map<std::string, double> targets;
    for (size_t i = 0; i < goal->joint_names.size(); ++i) {
      targets[goal->joint_names[i]] = goal->positions[i];
    }
    move_group_->setMaxVelocityScalingFactor(
      clamp_scaling(goal->max_velocity / std::max(1e-9, max_joint_velocity_limit_)));
    move_group_->setMaxAccelerationScalingFactor(
      clamp_scaling(goal->max_acceleration / std::max(1e-9, max_joint_acceleration_limit_)));
    if (!move_group_->setJointValueTarget(targets)) {
      throw std::runtime_error("movej target is invalid for planning group pi_arm");
    }
    prepare_planning_start();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("movej planning failed");
    }
    Follow::Goal follow;
    follow.trajectory = plan.trajectory.joint_trajectory;
    complete_follow_action<MoveJ>(handle, follow, "movej");
  }

  void execute_movel(const std::shared_ptr<GoalHandleMoveL> handle)
  {
    publish_feedback<MoveL>(handle, 0.0F, "running");
    const auto goal = handle->get_goal();
    const std::string frame =
      goal->target.header.frame_id.empty() ? "base_link" : goal->target.header.frame_id;
    const auto & pose = goal->target.pose;
    RCLCPP_INFO(
      get_logger(),
      "movel start: frame=%s target xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f) "
      "req lin_v=%.6f m/s lin_a=%.6f m/s2 ang_v=%.6f rad/s ang_a=%.6f rad/s2",
      frame.c_str(), pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
      goal->max_linear_velocity, goal->max_linear_acceleration,
      goal->max_angular_velocity, goal->max_angular_acceleration);
    RCLCPP_INFO(
      get_logger(),
      "movel angular_acc request ignored for timing; "
      "Pilz derived rot_acc=(max_trans_acc/max_trans_vel)*max_rot_vel=%.6f rad/s2 "
      "(request ang_a=%.6f rad/s2)",
      angular_acceleration_limit_, goal->max_angular_acceleration);

    const double velocity_scaling = clamp_scaling(
      std::min(
        goal->max_linear_velocity / linear_velocity_limit_,
        goal->max_angular_velocity / angular_velocity_limit_));
    const double acceleration_scaling = clamp_scaling(
      goal->max_linear_acceleration / linear_acceleration_limit_);
    RCLCPP_INFO(
      get_logger(),
      "movel scaling: vel=%.4f (effective lin=%.6f m/s ang=%.6f rad/s) "
      "acc=%.4f (effective lin_a=%.6f m/s2) limits vel=%.6f ang=%.6f acc=%.6f",
      velocity_scaling,
      velocity_scaling * linear_velocity_limit_,
      velocity_scaling * angular_velocity_limit_,
      acceleration_scaling,
      acceleration_scaling * linear_acceleration_limit_,
      linear_velocity_limit_, angular_velocity_limit_, linear_acceleration_limit_);

    move_group_->setPlanningPipelineId(kPilzPipeline);
    move_group_->setPlannerId(kPilzLinPlanner);
    move_group_->setPoseReferenceFrame(frame);
    if (!move_group_->setPoseTarget(goal->target.pose, "tool0")) {
      throw std::runtime_error("movel setPoseTarget failed for link tool0");
    }
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    RCLCPP_INFO(
      get_logger(), "movel planning pipeline=%s planner=%s", kPilzPipeline, kPilzLinPlanner);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    prepare_planning_start();
    const auto plan_code = move_group_->plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "movel Pilz LIN plan failed: error_code=%d", plan_code.val);
      throw std::runtime_error(
              "movel Pilz LIN plan failed: error_code=" + std::to_string(plan_code.val));
    }

    Follow::Goal follow_goal;
    follow_goal.trajectory = plan.trajectory.joint_trajectory;
    // stamp=0 means "execute as soon as received" for joint_trajectory_controller.
    follow_goal.trajectory.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    validate_timed_trajectory(follow_goal.trajectory);
    log_trajectory_summary("movel", follow_goal.trajectory);
    publish_feedback<MoveL>(handle, 0.3F, "sending_follow");
    // Do not use move_group_->execute(): unreliable under MultiThreadedExecutor from a
    // worker thread. Send the Pilz-timed trajectory to JTC, same as movejs/direct_move.
    complete_follow_action<MoveL>(handle, follow_goal, "movel");
  }

  void log_trajectory_summary(
    const std::string & tag, const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    std::string joints;
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if (i > 0) {
        joints += ",";
      }
      joints += trajectory.joint_names[i];
    }
    const auto & first = trajectory.points.front();
    const auto & last = trajectory.points.back();
    const double duration = rclcpp::Duration(last.time_from_start).seconds();
    RCLCPP_INFO(
      get_logger(),
      "%s trajectory: joints=[%s] points=%zu duration=%.3fs "
      "first_t=%.3fs last_t=%.3fs first_has_vel=%d last_has_vel=%d "
      "stamp_sec=%d stamp_nanosec=%u",
      tag.c_str(), joints.c_str(), trajectory.points.size(), duration,
      rclcpp::Duration(first.time_from_start).seconds(), duration,
      first.velocities.empty() ? 0 : 1, last.velocities.empty() ? 0 : 1,
      trajectory.header.stamp.sec, trajectory.header.stamp.nanosec);
    if (!first.positions.empty() && !last.positions.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "%s trajectory ends: first_q0=%.4f last_q0=%.4f delta_q0=%.4f",
        tag.c_str(), first.positions.front(), last.positions.front(),
        last.positions.front() - first.positions.front());
    }
  }

  void execute_movejs(const std::shared_ptr<GoalHandleMoveJs> handle)
  {
    publish_feedback<MoveJs>(handle, 0.0F, "running");
    Follow::Goal goal;
    goal.trajectory = handle->get_goal()->trajectory;
    complete_follow_action<MoveJs>(handle, goal, "movejs");
  }

  void execute_direct(const std::shared_ptr<GoalHandleDirect> handle)
  {
    publish_feedback<DirectMove>(handle, 0.0F, "running");
    const auto request = handle->get_goal();
    const auto targets = direct_targets(*request);
    Follow::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    double duration = 0.1;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.joint_names.empty() ||
        hardware_.positions.size() != hardware_.joint_names.size())
      {
        throw std::runtime_error("hardware joint positions unavailable");
      }
      goal.trajectory.joint_names = hardware_.joint_names;
      point.positions = hardware_.positions;
      validate_direct_positions(joint_limits_, goal.trajectory.joint_names, point.positions, "start/hold", true);
      const auto measured_start = point.positions;
      for (size_t i = 0; i < point.positions.size(); ++i) {
        const auto & bounds = joint_limits_.at(goal.trajectory.joint_names[i]);
        point.positions[i] = std::clamp(point.positions[i], bounds.lower, bounds.upper);
      }
      for (size_t i = 0; i < request->joint_names.size(); ++i) {
        const auto found = std::find(
          hardware_.joint_names.begin(), hardware_.joint_names.end(), request->joint_names[i]);
        if (found == hardware_.joint_names.end()) {
          throw std::runtime_error("direct_move contains unknown joint");
        }
        const auto index = static_cast<size_t>(
          std::distance(hardware_.joint_names.begin(), found));
        duration = std::max(
          duration, std::abs(targets[i] - point.positions[index]) /
          request->max_velocity);
        point.positions[index] = targets[i];
      }
      for (size_t i = 0; i < point.positions.size(); ++i) {
        duration = std::max(duration, std::abs(point.positions[i] - measured_start[i]) /
          request->max_velocity);
      }
    }
    validate_direct_positions(joint_limits_, goal.trajectory.joint_names, point.positions, "target/hold");
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(point);
    complete_follow_action<DirectMove>(handle, goal, "direct_move");
  }

  template<typename ActionT, typename GoalHandleT>
  void publish_feedback(
    const std::shared_ptr<GoalHandleT> handle, float progress, const std::string & message)
  {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.progress = progress;
      feedback->task_id = task_.task_id;
    }
    feedback->progress = progress;
    feedback->message = message;
    handle->publish_feedback(feedback);
  }

  // Wait on an rclcpp future with periodic logs so a hang is visible in journal.
  template<typename FutureT>
  std::future_status wait_future_logged(
    FutureT & future, double timeout_sec, const std::string & stage)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));
    auto next_log = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        RCLCPP_ERROR(
          get_logger(), "follow wait TIMEOUT at stage=%s after %.1fs",
          stage.c_str(), timeout_sec);
        return std::future_status::timeout;
      }
      if (now >= next_log) {
        const double remain = std::chrono::duration<double>(deadline - now).count();
        RCLCPP_INFO(
          get_logger(), "follow waiting stage=%s remain=%.1fs still_frames=%d stop=%d",
          stage.c_str(), remain, still_frames_.load(), stop_requested_.load() ? 1 : 0);
        next_log = now + 1s;
      }
      const auto slice = std::min(
        std::chrono::duration<double>(0.2),
        std::chrono::duration<double>(deadline - now));
      const auto status = future.wait_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(slice));
      if (status == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "follow wait READY at stage=%s", stage.c_str());
        return status;
      }
      if (stop_requested_.load()) {
        RCLCPP_WARN(get_logger(), "follow wait interrupted by stop at stage=%s", stage.c_str());
        return std::future_status::timeout;
      }
    }
    return std::future_status::timeout;
  }

  template<typename ActionT, typename GoalHandleT>
  void complete_follow_action(
    const std::shared_ptr<GoalHandleT> handle, Follow::Goal goal,
    const std::string & command)
  {
    const double output_speed = resolved_output_speed(handle->get_goal()->max_output_velocity);
    prepare_trajectory(goal.trajectory, output_speed);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.joint_names = goal.trajectory.joint_names;
      task_.target_positions = goal.trajectory.points.back().positions;
      task_.arrival_state = "tracking";
    }
    // Explicit action tolerances cannot silently inherit a zero/disabled controller default.
    goal.goal_tolerance.clear();
    for (const auto & name : goal.trajectory.joint_names) {
      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = name;
      tolerance.position = goal_position_tolerance_;
      tolerance.velocity = still_velocity_;
      goal.goal_tolerance.push_back(tolerance);
    }
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(arrival_timeout_);
    const auto lease_id = active_task_id();
    if (!set_motion_armed(true, lease_id, command + "/accepted", output_speed)) {
      if (stop_requested_.load() || handle->is_canceling()) {
        complete_action<ActionT>(handle, true, command, "stopped before arming");
        return;
      }
      throw std::runtime_error("failed to arm motion");
    }
    // Release the lease even if a future or result handler throws.
    bool completed = false;
    const auto release = [this, lease_id, &completed](int *) {
      if (!completed) {set_motion_armed(false, lease_id, "trajectory/scope_exit");}
    };
    int lease_token = 0;
    std::unique_ptr<int, decltype(release)> lease_guard(&lease_token, release);
    const double result_timeout = !goal.trajectory.points.empty() ?
      direct_result_timeout(
      rclcpp::Duration(goal.trajectory.points.back().time_from_start).seconds(),
      motion_timeout_, arrival_timeout_ + operation_timeout_ + arrival_stable_time_ + 1.0) : motion_timeout_;
    bool direct_disarmed = false;
    bool stop_confirmed = true;
    const auto disarm_direct = [this, lease_id, &direct_disarmed, &stop_confirmed](const std::string & stage) {
        if (!direct_disarmed) {
          stop_confirmed = set_motion_armed(false, lease_id, stage);
          direct_disarmed = true;
        }
      };
    RCLCPP_INFO(
      get_logger(),
      "%s follow: waiting for action server '%s' (timeout=%.1fs)",
      command.c_str(), follow_action_name_.c_str(), operation_timeout_);
    if (!follow_client_->wait_for_action_server(std::chrono::duration<double>(operation_timeout_))) {
      RCLCPP_ERROR(
        get_logger(), "%s follow: action server unavailable: %s",
        command.c_str(), follow_action_name_.c_str());
      disarm_direct("direct_move/follow_server_unavailable");
      complete_action<ActionT>(handle, false, command, "trajectory controller unavailable");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s follow: sending goal points=%zu",
      command.c_str(), goal.trajectory.points.size());
    rclcpp_action::Client<Follow>::SendGoalOptions send_options;
    send_options.goal_response_callback = [this, lease_id](const auto & accepted) {
        if (accepted && (stop_requested_.load() || !task_claimed_.load() ||
          active_task_id() != lease_id))
        {
          follow_client_->async_cancel_goal(accepted);
        }
      };
    auto send_future = follow_client_->async_send_goal(goal, send_options);
    if (wait_future_logged(send_future, operation_timeout_, command + "/send_goal") !=
      std::future_status::ready)
    {
      disarm_direct("direct_move/follow_goal_timeout");
      complete_action<ActionT>(handle, false, command, "trajectory goal timeout");
      return;
    }
    auto follow_handle = send_future.get();
    if (!follow_handle) {
      RCLCPP_ERROR(get_logger(), "%s follow: goal REJECTED by controller", command.c_str());
      disarm_direct("direct_move/follow_goal_rejected");
      complete_action<ActionT>(handle, false, command, "trajectory goal rejected");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s follow: goal ACCEPTED, waiting for result (timeout=%.1fs)",
      command.c_str(), result_timeout);
    publish_feedback<ActionT>(handle, 0.5F, "follow_executing");
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      active_follow_goal_ = follow_handle;
    }
    // The controller may accept after Stop already closed the hardware gate.
    // Cancel that late goal too; never allow it to survive into the next task.
    if (stop_requested_.load()) {follow_client_->async_cancel_goal(follow_handle);}
    auto result_future = follow_client_->async_get_result(follow_handle);
    const auto status = wait_future_logged(
      result_future, result_timeout, command + "/get_result");
    bool backend_ok = false;
    std::string detail = command;
    if (status == std::future_status::ready) {
      const auto wrapped = result_future.get();
      const int error_code = wrapped.result ? wrapped.result->error_code : -1;
      backend_ok = wrapped.result &&
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped.result->error_code == Follow::Result::SUCCESSFUL;
      detail = wrapped.result ? wrapped.result->error_string : "trajectory returned no result";
      if (wrapped.result && wrapped.result->error_code == Follow::Result::GOAL_TOLERANCE_VIOLATED) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          task_.arrival_state = "not_reached";
        }
        detail = arrival_error("controller timeout");
      }
      RCLCPP_INFO(
        get_logger(),
        "%s follow: result ready ok=%d result_code=%d error_code=%d detail=%s",
        command.c_str(), backend_ok ? 1 : 0, static_cast<int>(wrapped.code),
        error_code, detail.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s follow: result TIMEOUT, canceling goal", command.c_str());
      follow_client_->async_cancel_goal(follow_handle);
      detail = "trajectory result timeout";
    }
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      active_follow_goal_.reset();
    }
    // Keep the final position command active until fresh feedback has remained
    // within position AND velocity tolerances. A controller success alone is insufficient.
    if (backend_ok && !stop_requested_.load() && !wait_arrival("settling")) {
      backend_ok = false;
      detail = arrival_error("before stop");
    }
    // Successful arrival retains the final A4 target. User cancellation holds
    // current encoder positions; faults/timeouts retain the fail-closed stop.
    if (!backend_ok || stop_requested_.load() || handle->is_canceling()) {
      disarm_direct("direct_move/follow_result");
    }
    if (!stop_confirmed) {backend_ok = false; detail = "motor stop unconfirmed";}
    completed = complete_action<ActionT>(handle, backend_ok, command, detail);
  }

  template<typename ActionT, typename GoalHandleT>
  bool complete_action(
    const std::shared_ptr<GoalHandleT> handle, bool backend_ok,
    const std::string & command, const std::string & detail = "")
  {
    RCLCPP_INFO(
      get_logger(),
      "%s complete_action: backend_ok=%d detail=%s stop=%d canceling=%d still_frames=%d",
      command.c_str(), backend_ok ? 1 : 0, detail.c_str(),
      stop_requested_.load() ? 1 : 0, handle->is_canceling() ? 1 : 0, still_frames_.load());
    auto result = std::make_shared<typename ActionT::Result>();
    if (handle->is_canceling() || stop_requested_.load()) {
      auto reason = stop_reason_.load();
      if (reason == MotionStopReason::NONE) {
        reason = handle->is_canceling() ? MotionStopReason::ACTION_CANCEL :
          MotionStopReason::STOP_SERVICE;
      }
      // Await/establish hold even when the action result wins the race with
      // the service callback. Repeated holds are idempotent in the backend.
      if (holds_position_on_stop(reason)) {
        set_motion_armed(false, 0U, "canceled/current_position_hold", 0.0, true);
      }
      auto outcome = motion_stop_outcome(reason);
      if (holds_position_on_stop(reason) && !stop_hold_confirmed_.load()) {
        outcome = {false, 3006, pi_arm_interfaces::msg::TaskState::FAILED,
          "HOLD UNCONFIRMED: stop position could not be held; check feedback and retry Stop"};
      }
      result->success = outcome.success;
      result->code = outcome.code;
      result->message = outcome.message;
      finish_task(outcome.task_status, outcome.code, result->message);
      RCLCPP_WARN(
        get_logger(), "%s terminated by %s: code=%d message=%s",
        command.c_str(), motion_stop_reason_name(reason), result->code, result->message.c_str());
      if (reason == MotionStopReason::ACTION_CANCEL && handle->is_canceling()) {
        handle->canceled(result);
      } else {
        handle->abort(result);
      }
      return false;
    }
    if (!backend_ok) {
      result->success = false;
      result->code = 3001;
      result->message = detail.empty() ? command + " backend failed" : detail;
      RCLCPP_ERROR(
        get_logger(), "%s aborted: code=%d message=%s",
        command.c_str(), result->code, result->message.c_str());
      finish_task(pi_arm_interfaces::msg::TaskState::FAILED, result->code, result->message);
      handle->abort(result);
      return false;
    }
    publish_feedback<ActionT>(handle, 0.9F, "verifying_position_hold");
    if (!wait_arrival("verifying_hold")) {
      RCLCPP_ERROR(
        get_logger(),
        "%s wait_still FAILED still_frames=%d stop=%d",
        command.c_str(), still_frames_.load(), stop_requested_.load() ? 1 : 0);
      if (stop_requested_.load() || handle->is_canceling()) {
        complete_action<ActionT>(handle, true, command, "motion stopped");
        return false;
      }
      result->success = false;
      result->code = 3003;
      result->message = arrival_error("while holding");
      finish_task(pi_arm_interfaces::msg::TaskState::FAILED, result->code, result->message);
      handle->abort(result);
      return false;
    }
    if (!set_motion_armed(false, active_task_id(), "trajectory/hold", 0.0, true)) {
      result->success = false;
      result->code = 3003;
      result->message = "failed to retain position hold";
      finish_task(pi_arm_interfaces::msg::TaskState::FAILED, result->code, result->message);
      handle->abort(result);
      return false;
    }
    if (stop_requested_.load() || handle->is_canceling()) {
      return complete_action<ActionT>(handle, true, command, "motion stopped");
    }
    RCLCPP_INFO(get_logger(), "%s succeeded; holding position", command.c_str());
    result->success = true;
    result->code = 0;
    result->message = "ok";
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.arrival_state = "reached";
    }
    publish_feedback<ActionT>(handle, 1.0F, "completed");
    finish_task(pi_arm_interfaces::msg::TaskState::SUCCEEDED, 0, "ok");
    handle->succeed(result);
    return true;
  }

  void record_stop_reason(const MotionStopReason reason)
  {
    auto current = stop_reason_.load();
    while (motion_stop_priority(reason) > motion_stop_priority(current) &&
      !stop_reason_.compare_exchange_weak(current, reason))
    {
      // compare_exchange updates current; retry only while the new reason has
      // greater diagnostic priority than the reason recorded by another thread.
    }
  }

  bool request_stop(const MotionStopReason reason)
  {
    // Block new claims until the hardware stop/hold and cancellation finish.
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (!task_claimed_.load()) {stop_reason_.store(MotionStopReason::NONE);}
    record_stop_reason(reason);
    const bool already_stopping = stop_requested_.exchange(true);
    const bool hold = holds_position_on_stop(stop_reason_.load());
    const bool stopped = set_motion_armed(false, 0U, "stop_or_cancel", 0.0, hold);
    // A later hardware fault must override an earlier successful user hold.
    if (already_stopping && task_claimed_.load()) {return stopped;}
    if (move_group_) {
      move_group_->stop();
    }
    rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_goal;
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      follow_goal = active_follow_goal_;
    }
    if (follow_goal) {
      auto cancel_future = follow_client_->async_cancel_goal(follow_goal);
      if (cancel_future.wait_for(std::chrono::duration<double>(operation_timeout_)) !=
        std::future_status::ready)
      {
        RCLCPP_WARN(get_logger(), "Timed out waiting for trajectory cancellation");
      }
    }
    return stopped;
  }

  void stop_callback(
    const std::shared_ptr<pi_arm_interfaces::srv::StopMotion::Request>,
    std::shared_ptr<pi_arm_interfaces::srv::StopMotion::Response> response)
  {
    const bool stopped = request_stop(MotionStopReason::STOP_SERVICE);
    response->success = stopped;
    response->code = stopped ? 0 : 2002;
    response->message = "HOLD UNCONFIRMED: check feedback and retry Stop";
    if (stopped) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.teaching_active) {
        response->message = "drag teaching remains active; position loop released";
      } else if (!hardware_.motor_enabled.empty() &&
        std::none_of(hardware_.motor_enabled.begin(), hardware_.motor_enabled.end(), [](bool v) {return v;}))
      {
        response->message = "motors remain disabled; no position hold enabled";
      } else if (stop_hold_confirmed_.load()) {
        response->message = "motion stopped; current position hold confirmed";
      } else {
        response->success = false;
        response->code = 2002;
        response->message = "fault stop acknowledged; position hold NOT confirmed";
      }
    }
  }

  double hardware_timeout_;
  double operation_timeout_;
  double motion_timeout_;
  double still_velocity_;
  double goal_position_tolerance_;
  double arrival_stable_time_;
  double arrival_timeout_;
  std::uint64_t hardware_sequence_{0};
  std::map<std::string, JointLimits> joint_limits_;
  std::unique_ptr<pi_arm_transmission::CableKinematics> cable_;
  pi_arm_transmission::Vector6 protocol_ratios_{};
  double max_joint_velocity_limit_{0.0};
  double max_joint_acceleration_limit_{0.0};
  double linear_velocity_limit_;
  double linear_acceleration_limit_;
  double angular_velocity_limit_;
  double angular_acceleration_limit_;
  double state_rate_;
  std::string follow_action_name_;

  std::mutex state_mutex_;
  std::mutex follow_mutex_;
  std::mutex command_mutex_;
  std::mutex motion_gate_mutex_;
  std::atomic_bool stop_hold_confirmed_{false};
  std::atomic_bool preview_active_{false};
  rclcpp::Service<pi_arm_interfaces::srv::PreviewMotion>::SharedPtr preview_service_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  pi_arm_interfaces::msg::HardwareState hardware_;
  pi_arm_interfaces::msg::TaskState task_;
  rclcpp::Time last_hardware_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool task_claimed_;
  std::atomic_bool stop_requested_;
  std::atomic<MotionStopReason> stop_reason_;
  std::atomic_uint64_t next_task_id_;
  std::atomic_int still_frames_;
  std::atomic_int raw_still_frames_{0};

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr active_follow_goal_;
  rclcpp::Subscription<pi_arm_interfaces::msg::HardwareState>::SharedPtr hardware_sub_;
  rclcpp::Publisher<pi_arm_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr enable_client_;
  rclcpp::Client<pi_arm_interfaces::srv::SetTeaching>::SharedPtr teaching_client_;
  rclcpp::Service<pi_arm_interfaces::srv::SetTeaching>::SharedPtr teaching_service_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr disable_client_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reset_client_;
  rclcpp::Client<pi_arm_interfaces::srv::SetMotionArmed>::SharedPtr motion_arm_client_;

  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr enable_service_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr disable_service_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reset_service_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reboot_client_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reboot_service_;

  rclcpp::Service<pi_arm_interfaces::srv::StopMotion>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp_action::Server<MoveJ>::SharedPtr movej_server_;
  rclcpp_action::Server<MoveJs>::SharedPtr movejs_server_;
  rclcpp_action::Server<MoveL>::SharedPtr movel_server_;
  rclcpp_action::Server<DirectMove>::SharedPtr direct_server_;
};

}  // namespace pi_arm_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pi_arm_manager::ManagerNode>();
  try {
    node->initialize();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Manager initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), std::max(4U, std::thread::hardware_concurrency()));
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
