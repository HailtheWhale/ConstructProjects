#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "my_rb1_ros/Rotate.h"
#include <tf/tf.h>
#include <list>

class Rb1RotateService
{
  
    public:

        // ROS Objects
        ros::NodeHandle nh_;
        // ROS Services
        ros::ServiceServer my_service;
        // ROS Publishers
        ros::Publisher vel_pub;
        // ROS Subscribers 
        ros::Subscriber odom_sub;
        // ROS Messages
        geometry_msgs::Twist vel_msg;

        // Containers
        double yaw_current;

        // csts 
        double pi = 3.1415926535;
        float max_rotation_spd = 0.35;
        // PD Gains 
        float Kp = 0.6;
        float Kd = 0.1;
        // Within how many radians to stop. 
        double tol = 0.01;

        Rb1RotateService()
        {
            // Service
            my_service = nh_.advertiseService("/rotate_robot", &Rb1RotateService::rotate_callback, this);
            ROS_INFO("Service READY");
            // Sub Pub
            odom_sub = nh_.subscribe("/odom", 1000, &Rb1RotateService::odomCallback,this);
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }
        
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) 
        {
        // Convert from quat to euler. Return the yaw angle.
        double ox,oy,oz,ow;
        ox = msg->pose.pose.orientation.x;
        oy = msg->pose.pose.orientation.y;
        oz = msg->pose.pose.orientation.z;
        ow = msg->pose.pose.orientation.w;
        tf::Quaternion quat(ox,oy,oz,ow);
        tf::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        yaw_current = yaw;
        }
        
        bool rotate_callback(my_rb1_ros::Rotate::Request  &req,
                            my_rb1_ros::Rotate::Response &res)
        {
            // For timing
            ros::Rate loop_rate(1);

            // Make sure stationary. 
            vel_msg.linear.x = 0.0;
            double yaw_prev = yaw_current;

            // Convert from degrees to radians 
            double req_rad = req.degrees*pi/180;
            ROS_INFO("Service REQUEST. Rotate %f rads.",req_rad);

            // How many rads have been transversed
            double angle_sum = 0.0;
            // for Kd term. 
            float err_prev = 0.0;
            // Before adding on the d term. 
            bool init = false;
            
            // Init speed 
            vel_msg.angular.z = 0.05;
            do {
                // Make velocity based on PD control
                float err = req_rad-angle_sum;
                if (init){
                    vel_msg.angular.z = (err*Kp+Kd*(err-err_prev));
                    // Clamp the val to max speed. 
                    if (vel_msg.angular.z > max_rotation_spd){
                        vel_msg.angular.z = max_rotation_spd;
                    } else if (vel_msg.angular.z < -max_rotation_spd){
                        vel_msg.angular.z = -max_rotation_spd;
                        }
                }
                vel_pub.publish(vel_msg);


                double angle_changed = yaw_current-yaw_prev;
                // ROS_INFO("ANGLE CHANGE %f. TOTAL %f. Want %f. Speed %f.",angle_changed,angle_sum,req_rad,vel_msg.angular.z);
                // Add/ subtract 2pi to the diff if going past the angle wrap singularity
                if (angle_changed<-pi){
                    angle_changed+=2*pi;
                } else if (angle_changed>pi){
                    angle_changed-=2*pi;
                }
                angle_sum+=angle_changed;

                // Break loop if within tolerance. 
                if (angle_sum > req_rad - tol and angle_sum < req_rad + tol){
                    break;
                }
                // Update vals 
                init = true;
                err_prev=err;
                yaw_prev=yaw_current;
                loop_rate.sleep(); // Maintain refresh rate. 
            } while (true);

            // Stop moving
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
            // ROS_INFO("Service COMPLETE! Rotated %f rads. Wanted %f rads.",angle_sum,req_rad);
            ROS_INFO("Service COMPLETE!");

            res.result = true;

            return true;
        }
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rb1_rotate_service_node");
  
  Rb1RotateService rb1RotateService;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}