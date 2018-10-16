#include "iiam/iiam.h"
#include "iiam/calculation.h"

GridClass::GridClass() { }

GridClass::GridClass(int x,int y) {

    this->x = x;
    this->y = y;
    cell_type = -1;
    movable_cell = true;
    process_cell = false;
    observation_point = false;
    obs_count = 0;
    shooted_vector_number = 0;
}	  

GridClass::~GridClass(void){ }

void GridClass::setCellType(short int type) {
    cell_type = type;
    return;      
}

void GridClass::setMovableCell(bool type) {
    movable_cell = type;
    return;
}

void GridClass::setCautionCell() {
    movable_cell = false;
    return;
}

void GridClass::setProcessCell(bool process) {
    process_cell = process;
    return;
}

void GridClass::setObsCount(int count) {
    obs_count = count;
    return;
}

void GridClass::setShootedVectorNumber(int count) {
    shooted_vector_number = count;
    return;
}

void GridClass::setObservationPoint() {
    observation_point = true;
    return;
}

void GridClass::updateObsCount() {
    obs_count++;
    return;
}

void GridClass::updateShootedVectorNumber() {
    shooted_vector_number++;
    return;
}

void GridClass::pushbackShootingVector(float vector) {
    shooting_vector.push_back(VectorClass(vector));
    return;     
}

void GridClass::pushbacksGrid(int x, int y, float angle, float angleObs2sgrid) {
    sGrid.push_back(new ShootingGrid(x, y, angle, angleObs2sgrid));
    return;
}

void GridClass::erasesGrid(int number){
    sGrid.erase(sGrid.begin() + number);
    return;
}

void GridClass::clearsGrid() {
    sGrid.clear();
    return;
}

int GridClass::getX() {
    return x;
}
      
int GridClass::getY() {
    return y;
}

VectorClass GridClass::getShootingVector(int number) {
    return shooting_vector[number];
}

int GridClass::getShootingVectorNumber() {
    return shooting_vector.size();
}

ShootingGrid *GridClass::getsGrid(int number) {
    return sGrid[number];
}

int GridClass::getsGridNumber() {
    return sGrid.size();
}

int GridClass::getObsCount() {
    return obs_count;
}

int GridClass::getShootedVectorNumber() {
    return shooted_vector_number;
}

short int GridClass::getCellType() {
    return cell_type;
}

bool GridClass::isProcessCell() {
    return process_cell;
}

bool GridClass::isMovableCell() {
    return movable_cell;
}

bool GridClass::isObservationPoint() {
    return observation_point;
}

//指定した撮影ベクトルがすでに付加されている撮影ベクトルと似ているかどうか
bool GridClass::isSimilarVector(float src_vector, float cos_interval){

    std::vector<VectorClass>::iterator vector=shooting_vector.begin();     
    for( ; vector != shooting_vector.end(); ++vector){
        if(cos(CalcRadianGap(src_vector, (*vector).getAngle())) > cos_interval){
            return true;
        }
    }
          
    return false;
}
      
bool GridClass::hasShootingVector() {
    if(shooting_vector.size() > 0) return true;
    return false;
}

