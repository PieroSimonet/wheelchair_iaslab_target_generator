#include "prox/path_to_field.h"

int main(int argc, char** argv) {
    ros::init(argc, argv,"path_to_field");
    prox::path_to_field* ptf = new prox::path_to_field();
    ptf->run();
    return 0;
}