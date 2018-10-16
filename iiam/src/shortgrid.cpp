#include "iiam/iiam.h"

ShootingGrid::ShootingGrid(int x, int y) {

    this->x = x;
    this->y = y;
    angle = 0.0;
    shootedgrid = false;
    
}

ShootingGrid::ShootingGrid(int x, int y, float angle) {

    this->x = x;
    this->y = y;
    this->angle = angle;
    shootedgrid = false;
    
}

ShootingGrid::ShootingGrid(int x, int y, float angle, float angleObs2sGrid) {

    this->x = x;
    this->y = y;
    this->angle = angle;
    this->angleObs2sGrid = angleObs2sGrid;
    shootedgrid = false;
    
}

ShootingGrid::~ShootingGrid() { }

int ShootingGrid::getX() {
    return x;
}
    
int ShootingGrid::getY() {
    return y;
}

int ShootingGrid::getID() {
    return id;
}
      
float ShootingGrid::getAngle() {
    return angle;
}
          
float ShootingGrid::getAngleObs2sGrid() {
    return angleObs2sGrid;
}

bool ShootingGrid::getShootedGrid() {
    return shootedgrid;
}

void ShootingGrid::setShootedGrid() {
    shootedgrid = true;
    return;
}

void ShootingGrid::setShootedGrid(bool type) {
    shootedgrid = type;
    return;
}

