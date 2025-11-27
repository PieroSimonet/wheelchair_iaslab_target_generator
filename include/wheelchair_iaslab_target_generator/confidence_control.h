#ifndef PROX_CONFIDENCECONTROL_HPP
#define PROX_CONFIDENCECONTROL_HPP

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>

#include <cmath>

namespace wheelchair_iaslab_target_generator {

    class confidence_control {
        public:
            confidence_control(void);
            virtual ~confidence_control(void);
            
            void run(void);
        
        protected:
            void odom_callback(const nav_msgs::Odometry msg);
            void goal_callback(const move_base_msgs::MoveBaseActionGoal msg);
            void check_control_state();
            
        private:
            bool setup_listeners();
            bool setup_services();
        
        private:
            ros::NodeHandle	nh_;
            
            bool goal_reached_ = true;
            bool running = false;

            std_srvs::Empty empty;

            std::string odom_topic_;
            std::string goal_topic_;
      	
            ros::Subscriber	odometry_sub_;
            ros::Subscriber	goal_sub_;
            ros::ServiceClient field_stard_srv, field_stop_srv, request_new_target_srv;

            std_msgs::Bool goalreached_msg_;
            ros::Publisher goalreached_pub_;

            nav_msgs::Odometry current_position;
            geometry_msgs::PoseStamped current_goal;

            double short_distance;
            double long_distance;

            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;


    };
} // namespace wheelchair_iaslab_target_generator

#endif
