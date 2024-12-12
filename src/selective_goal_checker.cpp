#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <limits>
#include <vector>
#include <seletive_goal_checker/selective_goal_checker.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace selective_goal_checker
{
    SelectiveGoalChecker::SelectiveGoalChecker() : xy_goal_tolerance_(0.25),
        yaw_goal_tolerance_(0.25),
        stateful_(true),
        check_xy_(true),
        xy_goal_tolerance_sq_(0.0625)
    {
        RCLCPP_INFO(rclcpp::get_logger("SelectiveGoalChecker started."), "SelectiveGoalChecker created");
    }

    void SelectiveGoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
    {
    plugin_name_ = plugin_name;
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".stateful", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".check_xy_dist", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".check_x_only", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node,
        plugin_name + ".check_v", rclcpp::ParameterValue(false));

    node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
    node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter(plugin_name + ".stateful", stateful_);
    node->get_parameter(plugin_name + ".check_xy_dist", check_xy_dist_);
    node->get_parameter(plugin_name + ".check_x_only", check_x_);
    node->get_parameter(plugin_name + ".check_v", check_v_);

    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
    std::cout << "check x only: " << check_x_ << std::endl;
    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&SelectiveGoalChecker::dynamicParametersCallback, this, _1));
    }

    void SelectiveGoalChecker::reset()
    {
        check_xy_ = true;
    }

    bool SelectiveGoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &)
    {
    // std::cout << "query pose x: " << query_pose.position.x << " query pose y: "
    //                 << query_pose.position.y << std::endl;
    if (check_xy_ && check_xy_dist_) {
        double dx = query_pose.position.x - goal_pose.position.x,
        dy = query_pose.position.y - goal_pose.position.y;
        if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
        return false;
        }
        // We are within the window
        // If we are stateful, change the state.
        if (stateful_) {
        check_xy_ = false;
        }
        
    }else if(check_x_){
        double yaw_robot = getYawFromPose(query_pose);
        double yaw_goal = getYawFromPose(goal_pose);

        Eigen::Matrix4d T_map_robot,T_map_goal,T_robot_goal;
        T_map_robot << cos(yaw_robot),-sin(yaw_robot),0,query_pose.position.x,
                       sin(yaw_robot),cos(yaw_robot),0,query_pose.position.y,
                        0,0,1,0,
                        0,0,0,1;
        T_map_goal << cos(yaw_goal),-sin(yaw_goal),0,goal_pose.position.x,
                       sin(yaw_goal),cos(yaw_goal),0,goal_pose.position.y,
                        0,0,1,0,
                        0,0,0,1;
        T_robot_goal = T_map_robot.inverse() * T_map_goal;

        double dx = abs(T_robot_goal(0,3));

        // std::cout << "dx: " << dx << std::endl;
        if (dx > xy_goal_tolerance_) {
            return false;
        }
    }
    if(check_v_){
        // TODO: add velocity check code here.
    }
    double dyaw = angles::shortest_angular_distance(
        tf2::getYaw(query_pose.orientation),
        tf2::getYaw(goal_pose.orientation));
    return fabs(dyaw) < yaw_goal_tolerance_;
    }

    bool SelectiveGoalChecker::getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance)
    {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
    }

    double SelectiveGoalChecker::getYawFromPose(const geometry_msgs::msg::Pose & pose)
    {
        // 从Pose中获取四元数
        const auto & orientation = pose.orientation;

        // 创建tf2的四元数对象
        tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);

        // 使用tf2::getYaw()获取yaw角度
        double yaw = tf2::getYaw(quat);

        return yaw;
    }

    rcl_interfaces::msg::SetParametersResult
    SelectiveGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
    rcl_interfaces::msg::SetParametersResult result;
    for (auto & parameter : parameters) {
        const auto & type = parameter.get_type();
        const auto & name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
        if (name == plugin_name_ + ".xy_goal_tolerance") {
            xy_goal_tolerance_ = parameter.as_double();
            xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
        } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
            yaw_goal_tolerance_ = parameter.as_double();
        }
        } else if (type == ParameterType::PARAMETER_BOOL) {
        if (name == plugin_name_ + ".stateful") {
            stateful_ = parameter.as_bool();
        }
        }
    }
    result.successful = true;
    return result;
    }
}



PLUGINLIB_EXPORT_CLASS(selective_goal_checker::SelectiveGoalChecker, nav2_core::GoalChecker)