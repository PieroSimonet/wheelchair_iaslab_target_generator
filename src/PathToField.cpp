#include "wheelchair_iaslab_target_generator/path_to_field.h"

int main(int argc, char** argv) {
    ros::init(argc, argv,"path_to_field");
    wheelchair_iaslab_target_generator::path_to_field* ptf = new wheelchair_iaslab_target_generator::path_to_field();
    ptf->run();
    return 0;
}
