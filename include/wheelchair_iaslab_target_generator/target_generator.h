#ifndef PROX_TARGET_GENERETOR_HPP
#define PROX_TARGET_GENERETOR_HPP

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace wheelchair_iaslab_target_generator {
    class target_generator {
        public:
            target_generator(void);
            virtual ~target_generator(void);

            void run();

        protected:
            void setup_listeners();
            void setup_services();
            void setup_publishers();
            void initialize();

            // void generate_target(); // TODO: -> Mettere questo come plugin

        private:
            void bci_callback(const rosneuro_msgs::NeuroOutput msg);
            // void traversibility_callback(const rosneuro::BCI msg);
            // void errp_callback(const rosneuro::BCI msg);
            // void service_callback(const rosneuro::BCI msg);

            // void publish_target();

        private:
            ros::NodeHandle	nh_;

            bool is_discrete = false;

            int rate_;

            rosneuro_msgs::NeuroOutput bci_state_;

            std::string bci_topic_;
            std::string odom_topic_;
            
            ros::Subscriber bci_sub_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener* tf_listener_;

    };
} // namespace wheelchair_iaslab_target_generator


#endif
