#include "prox/confidence_control.h"

int main() {
    prox::confidence_control* cf = new prox::confidence_control();
    cf->run();
    return 0;
}