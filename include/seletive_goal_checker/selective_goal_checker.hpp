#ifndef __SELECTIVE_GOAL_CHECKER_HPP__
#define __SELECTIVE_GOAL_CHECKER_HPP__

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace selective_goal_checker
{
    class SelectiveGoalChecker : public nav2_core::GoalChecker
    {
        public:
            SelectiveGoalChecker();
            void initialize(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                const std::string & plugin_name,
                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
            void reset() override;
            bool isGoalReached(
                const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
                const geometry_msgs::msg::Twist & velocity) override;
            bool getTolerances(
                geometry_msgs::msg::Pose & pose_tolerance,
                geometry_msgs::msg::Twist & vel_tolerance) override;
            double getYawFromPose(const geometry_msgs::msg::Pose & pose);
        protected:
            double xy_goal_tolerance_, yaw_goal_tolerance_,x_tolerance_,y_tolerance_;
            bool stateful_, check_xy_ ,check_x_ , check_v_, check_xy_dist_;
            // Cached squared xy_goal_tolerance_
            double xy_goal_tolerance_sq_;
            // Dynamic parameters handler
            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
            std::string plugin_name_;

            /**
            * @brief Callback executed when a paramter change is detected
            * @param parameters list of changed parameters
            */
            rcl_interfaces::msg::SetParametersResult
            dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    };
}


#endif