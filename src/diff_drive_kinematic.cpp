#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class DiffDriveNode: public rclcpp::Node
{
    public: 
        DiffDriveNode() : Node("diff_drive_node")
        {   
            
            cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
                std::bind(&DiffDriveNode::cmd_callback, this, _1)
            );
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            last_time_ = this->now();
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&DiffDriveNode::update, this));
        }

    private:
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {   
            RCLCPP_INFO_STREAM(this->get_logger(), "x =" << msg->linear.x << " w=" << w_);
            RCLCPP_INFO_STREAM(this->get_logger(), "w_ =" << msg->angular.z<< " w=" << w_);
            v_ = msg->linear.x;
            w_ = msg->angular.z;
            RCLCPP_INFO_STREAM(this->get_logger(), "CMD_VEL RECEIVED -> v=" << v_ << " w=" << w_);
        }

        void update()
        {
            auto now = this->now();
            double dt = (now - last_time_).seconds();
            last_time_ = now;
            dt = 0.020;
            x_ += v_ * cos(theta_) * dt;
            y_ += v_ * sin(theta_) * dt;
            theta_ += w_ * dt;


            publish_odom(now);
            publishTF(now);
        }

        void publish_odom(const rclcpp::Time &time)
        {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;

            odom.pose.pose.orientation.z = sin(theta_ / 2.0);
            odom.pose.pose.orientation.w = cos(theta_ / 2.0);

            odom.twist.twist.linear.x = v_;
            odom.twist.twist.angular.z = w_;

            odom_pub_->publish(odom);
        }
        
        void publishTF(const rclcpp::Time &time)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = time;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "base_link";

            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;

            tf.transform.rotation.z = sin(theta_ / 2.0);
            tf.transform.rotation.w = cos(theta_ / 2.0);

            tf_broadcaster_->sendTransform(tf);
        }


        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        double v_ = 0.0;
        double w_ = 0.0;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_time_;

        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist> > cmd_sub_;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry> > odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveNode>());
    rclcpp::shutdown();
    return 0;
}