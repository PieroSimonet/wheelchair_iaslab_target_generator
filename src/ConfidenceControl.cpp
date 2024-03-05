#include "wheelchair_iaslab_target_generator/confidence_control.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "confidence_control");
    wheelchair_iaslab_target_generator::confidence_control* cf = new wheelchair_iaslab_target_generator::confidence_control();
    cf->run();
    return 0;
}
