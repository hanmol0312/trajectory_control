#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "trajectory_control/spline.hpp"
#include "trajectory_control/trajectory.hpp"
#include "trajectory_control/ppt_controller.hpp"

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("pure_pursuit_controller")
    {
        // Load patameters
        declare_parameter("lookahead", 0.5);
        declare_parameter("max_velocity", 0.3);
        declare_parameter("goal_position_tolerance", 0.1);
        declare_parameter("goal_heading_tolerance", 0.1);
        declare_parameter("k_theta", 0.1);
        declare_parameter("curvature_gain", 0.1);
        declare_parameter("omega_max", 0.1);
        declare_parameter("k1", 0.1);
        declare_parameter("k2", 0.1);
        declare_parameter("k3", 0.1);


        declare_parameter<std::vector<double>>(
            "waypoints",
            {0.0, 0.0,
            1.0, 2.0,
            3.0, 3.0,
            5.0, 2.0,
            6.0, 0.0});


        double lookahead = get_parameter("lookahead").as_double();
        double k_theta = get_parameter("k_theta").as_double();
        double omega_max = get_parameter("omega_max").as_double();
        double max_velocity = get_parameter("max_velocity").as_double();
        double curvature_gain = get_parameter("curvature_gain").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(),"lookahead:"<<lookahead);
        double k1 = get_parameter("k1").as_double();
        double k2 = get_parameter("k2").as_double();
        double k3 = get_parameter("k3").as_double();
        std::vector<double> wp_flat =
            get_parameter("waypoints").as_double_array();


        if (wp_flat.size() % 2 != 0 || wp_flat.size() < 4)
        {
            RCLCPP_ERROR(get_logger(),
                "Waypoints must contain pairs of x,y values.");
            throw std::runtime_error("Invalid waypoint format");
        }

        std::vector<double> wx;
        std::vector<double> wy;
        // Flatten waypoints
        for (size_t i = 0; i < wp_flat.size(); i += 2)
        {
            wx.push_back(wp_flat[i]);
            wy.push_back(wp_flat[i + 1]);
        }

        goal_position_tolerance_ =
            get_parameter("goal_position_tolerance").as_double();

        goal_heading_tolerance_ =
            get_parameter("goal_heading_tolerance").as_double();


        controller_ = std::make_shared<PurePursuitController>(
            max_velocity,
            omega_max,
            k1,   // k1
            k2,   // k2
            k3);  // k3

        // Publishers and subscribers
        traj_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);
        actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/actual_path", 10);
        cross_track_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/cross_track_error", 10);

        heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/heading_error", 10);
         lateral_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/lateral_error", 10);

        actual_path_.header.frame_id = "odom";
        planned_path.header.frame_id = "odom";
        Spline2D spline(wx, wy);        // Generates spline functions


        // First generate trajectory given the spline
        TrajectoryGenerator traj_generator;
        trajectory_ = traj_generator.generate(spline,max_velocity, 0.05);
        goal_x_ = trajectory_.back().x;
        goal_y_ = trajectory_.back().y;
        goal_theta_ = trajectory_.back().theta;
        


        for (const auto &p : trajectory_)
        {
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.orientation.w = 1.0;
            planned_path.poses.push_back(pose);
        }

        RCLCPP_INFO_STREAM(this->get_logger(),"plan_size:"<<planned_path.poses.size());

        // Controller takes in the generated trajectory.
        controller_->setTrajectory(trajectory_);


        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ControllerNode::odomCallback,
                      this, std::placeholders::_1));
    }

private:

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (trajectory_.empty())
            return;

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf2::getYaw(msg->pose.pose.orientation);

        // ---------------------------------------
        // Initialize trajectory start time
        // ---------------------------------------
        if (!start_time_initialized_)
        {
            start_time_ = this->now();
            last_time_ = start_time_;
            effective_time_ = 0.0;        
            start_time_initialized_ = true;
        }

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Compute tracking error magnitude
        double dx_err = goal_x_ - x;
        double dy_err = goal_y_ - y;
        double error_norm = std::hypot(dx_err, dy_err);

        // Time scaling factor
        double alpha = 0.5;  // tune
        double time_scale = 1.0 / (1.0 + alpha * error_norm);

        // Update effective time
        effective_time_ += dt * time_scale;

        double current_time = effective_time_;

        double total_traj_time = trajectory_.back().t;

        // ---------------------------------------
        // If trajectory time finished → switch to goal alignment
        // ---------------------------------------
        if (current_time >= total_traj_time)
        {
            double dx_goal = goal_x_ - x;
            double dy_goal = goal_y_ - y;
            double position_error = std::hypot(dx_goal, dy_goal);

            double heading_error =
                std::atan2(
                    std::sin(goal_theta_ - theta),
                    std::cos(goal_theta_ - theta));

            if (position_error < goal_position_tolerance_)
            {
                if (std::abs(heading_error) > goal_heading_tolerance_)
                {
                    geometry_msgs::msg::Twist cmd;
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 1.5 * heading_error;
                    cmd_pub_->publish(cmd);
                }
                else
                {
                    geometry_msgs::msg::Twist stop_cmd;
                    stop_cmd.linear.x = 0.0;
                    stop_cmd.angular.z = 0.0;
                    cmd_pub_->publish(stop_cmd);

                    goal_reached_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Goal reached and aligned!");
                }
            }
            else
            {
                // If time finished but not at goal,
                // continue tracking final point
                double v, omega;
                controller_->computeCommand(
                    x, y, theta,
                    total_traj_time,
                    v, omega);

                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = v;
                cmd.angular.z = omega;
                cmd_pub_->publish(cmd);
            }

            return;
        }
        // ---------------------------------------
        // Compute tracking error w.r.t. reference
        // ---------------------------------------

        size_t target_index = trajectory_.size() - 1;

        for (size_t i = 0; i < trajectory_.size(); ++i)
        {
            if (trajectory_[i].t >= current_time)
            {
                target_index = i;
                break;
            }
        }

        double x_d = trajectory_[target_index].x;
        double y_d = trajectory_[target_index].y;
        double theta_d = trajectory_[target_index].theta;

        double dx = x_d - x;
        double dy = y_d - y;

        // Robot frame errors
        double x_r = std::cos(theta) * dx +
                    std::sin(theta) * dy;

        double y_r = -std::sin(theta) * dx +
                    std::cos(theta) * dy;

        double theta_error =
            std::atan2(
                std::sin(theta_d - theta),
                std::cos(theta_d - theta));

        double euclidean_error = std::hypot(dx, dy);
        std_msgs::msg::Float64 lat_msg;
        lat_msg.data = y_r;
        lateral_error_pub_->publish(lat_msg);

        std_msgs::msg::Float64 head_msg;
        head_msg.data = theta_error;
        heading_error_pub_->publish(head_msg);

        std_msgs::msg::Float64 pos_msg;
        pos_msg.data = euclidean_error;
        cross_track_pub_->publish(pos_msg);
        // ---------------------------------------
        // Normal trajectory tracking phase
        // ---------------------------------------
        double v, omega;

        controller_->computeCommand(
            x,
            y,
            theta,
            current_time,
            v,
            omega);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = omega;

        cmd_pub_->publish(cmd);

        // ---------------------------------------
        // Publish actual path for visualization
        // ---------------------------------------
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = msg->pose.pose.orientation;

        actual_path_.poses.push_back(pose);
        publish_paths();
    }

    void publish_paths(){
        
        actual_path_pub_->publish(actual_path_);
        traj_pub_->publish(planned_path);

    }

    std::shared_ptr<PurePursuitController> controller_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_error_pub_;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;

    nav_msgs::msg::Path actual_path_;
    nav_msgs::msg::Path planned_path;
    std::vector<TrajPoint> trajectory_;
    double effective_time_ = 0.0;
    rclcpp::Time last_time_;
    double goal_position_tolerance_ = 0.1;   // meters
    double goal_heading_tolerance_ = 0.1;    // radians (~6 degrees)4
    double goal_x_, goal_y_, goal_theta_;
    rclcpp::Time start_time_;
    bool start_time_initialized_ = false;
    bool goal_reached_ = false;
    

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
