#ifndef PROX_PATHTOFIEL_HPP
#define PROX_PATHTOFIEL_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <proximity_grid/ProximityGridMsg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>

namespace wheelchair_iaslab_target_generator {
    class path_to_field {
        public:
            path_to_field();
            virtual ~path_to_field();

            void run();

        protected:
            void setup_listeners();
            void setup_services();
            void setup_publishers();
            void initialize();

        private:
            void segment_path();
            void generate_attactor();
            void setup_grid();

            void odom_callback(const nav_msgs::Odometry msg);
            void path_callback(const nav_msgs::Path msg);
            void goal_callback(const move_base_msgs::MoveBaseActionGoal msg);
          
        private:
            ros::NodeHandle nh_;

            std::string odom_topic_, path_topic_, goal_topic_;
            std::string attractor_topic_;

            ros::Subscriber path_sub_, odom_sub_, goal_sub_;
            ros::Publisher  attractor_pub_;

            double angle_min_, angle_max_, angle_increment_;
            double range_min_, range_max_;
            double max_distance_, delta_distance_;
            int    rate_;

            std::string frame_id_, odom_frame_, base_frame;

            nav_msgs::Path path_;
            nav_msgs::Odometry current_odom_;
            geometry_msgs::PoseStamped current_goal_;
            proximity_grid::ProximityGridMsg attractor_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener* tf_listener;
            
    };
} // namespace wheelchair_iaslab_target_generator


#endif
