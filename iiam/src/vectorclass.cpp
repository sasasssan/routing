#include "iiam/iiam.h"
#include "iiam/calculation.h"

VectorClass::VectorClass() { }
          
VectorClass::VectorClass(float angle) {
    this->angle = FixRadian_0to2Pi(angle);
    possible_shooting = true;
}
          
VectorClass::~VectorClass() { }
     
void VectorClass::notPossibleShooting() {
    possible_shooting = false;
    return;
}
          
float VectorClass::getAngle() {
    return angle;
}

bool VectorClass::isPossibleShooting() {
    return possible_shooting;
}
