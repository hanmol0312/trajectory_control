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
            lookahead, max_velocity,curvature_gain,k_theta,omega_max);

        // Publishers and subscribers
        traj_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);
        actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/actual_path", 10);
        cross_track_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/cross_track_error", 10);

        heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/heading_error", 10);

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
        double x = msg->pose.pose.position.x;          // get current locations 
        double y = msg->pose.pose.position.y;
        double theta = tf2::getYaw(msg->pose.pose.orientation);
        double dx_goal = goal_x_ - x;                  // difference between present x and goal x similarly for y   
        double dy_goal = goal_y_ - y;

        double position_error = std::hypot(dx_goal, dy_goal);

        double heading_error = std::atan2(             // heading error 
            std::sin(goal_theta_ - theta),
            std::cos(goal_theta_ - theta));

        if (position_error < goal_position_tolerance_)          // checking whether goal reached along with allignement
        {
            if (std::abs(heading_error) > goal_heading_tolerance_)
            {
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.angular.z = 1.0 * heading_error;  // simple P control
                cmd_pub_->publish(cmd);
            }
            else
            {
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_pub_->publish(stop_cmd);

                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached and aligned!");
            }
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = msg->pose.pose.orientation;

        actual_path_.poses.push_back(pose);
        publish_paths();
        double min_dist = 1e9;
        size_t closest_index = 0;

        for (size_t i = 0; i < trajectory_.size(); ++i)
        {
            double dx = trajectory_[i].x - x;
            double dy = trajectory_[i].y - y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = i;
            }
        }

        double cross_track_error = min_dist;                 // calculation of cross track error

        std_msgs::msg::Float64 cross_msg;
        cross_msg.data = cross_track_error;
        cross_track_pub_->publish(cross_msg);

        double desired_theta = trajectory_[closest_index].theta;

        double path_heading_error =                      // calculation of path heading error
            std::atan2(
                std::sin(desired_theta - theta),
                std::cos(desired_theta - theta));

        std_msgs::msg::Float64 heading_msg;
        heading_msg.data = path_heading_error;
        heading_error_pub_->publish(heading_msg);

        double v, omega;

        controller_->computeCommand(x, y, theta, v, omega);        // controller given v and omega

        
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = omega;

        cmd_pub_->publish(cmd);
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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;

    nav_msgs::msg::Path actual_path_;
    nav_msgs::msg::Path planned_path;
    std::vector<TrajPoint> trajectory_;

    double goal_position_tolerance_ = 0.1;   // meters
    double goal_heading_tolerance_ = 0.1;    // radians (~6 degrees)4
    double goal_x_, goal_y_, goal_theta_;

    bool goal_reached_ = false;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
