#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>

#include "iiam/iiam.h"
#include "iiam/calculation.h"

bool obs_count_des(GridClass* left, GridClass* right);

iiam::iiam(float resolution) { Init(resolution); }

iiam::~iiam() { }

void iiam::MakeIIAM(const nav_msgs::OccupancyGrid &map, geometry_msgs::Polygon *observation_points) { 
    
    iiammap.clear();
    
    width = map.info.width;
    height = map.info.height;
    resolution = map.info.resolution;
    origin = map.info.origin;
        
    for(int y = height - 1; y >= 0; y--) {
        for(int x = 0; x < width; x++) {
            int index = XYtoP(x, y);
            iiammap.push_back(new GridClass(x, y));
            if(iiammap[index]->getCellType() != map.data[index]) {
                iiammap[index]->setCellType(map.data[index]); //セルの種類を登録
            }
        }
    }

    //occupied cellに対するロボット接近不可領域と撮影ベクトルの定義
    std::vector<GridClass*>::iterator current_cell = iiammap.begin();
    for( ; current_cell != iiammap.end(); ++current_cell) {
        if((*current_cell)->getCellType() == Occupied || (*current_cell)->getCellType() == Object) {
            int x = (*current_cell)->getX();
            int y = (*current_cell)->getY();
            int p = XYtoP(x, y);
            SetCautionArea(x, y); //ロボットの接近不可領域を設定
            if((*current_cell)->getCellType() == Object) {
                SetShootingVector(p); //撮影ベクトルを付加
                if(iiammap[p]->hasShootingVector()) {
                    ShootingArea(p); //撮影ベクトル範囲内のfree cellの情報を更新
                }
            }
        } else if((*current_cell)->getCellType() == Unknown) { 
            int x = (*current_cell)->getX();
            int y = (*current_cell)->getY();
            SetCautionArea(x, y); 
        }
    }
    
    GetObservationPoint(observation_points);

}

void iiam::GetObservationPoint(geometry_msgs::Polygon *observation_points) { 

    //ロボットが移動可能なfree cellを格納
    std::vector<GridClass*> temp;
    for(int i=0; i < iiammap.size(); i++) {
        if(iiammap[i]->getCellType() == Free && iiammap[i]->isMovableCell() && iiammap[i]->isProcessCell()) {
            iiammap[i]->setProcessCell(false);
            temp.push_back(new GridClass(iiammap[i]->getX(), iiammap[i]->getY()));
            (*(temp.end()-1))->setObsCount(iiammap[i]->getObsCount());
            (*(temp.end()-1))->setShootedVectorNumber(iiammap[i]->getShootedVectorNumber());
            for(int j = 0; j < iiammap[i]->getsGridNumber(); j++) {
                (*(temp.end()-1))->pushbacksGrid(iiammap[i]->getsGrid(j)->getX(), iiammap[i]->getsGrid(j)->getY(),
                                                 iiammap[i]->getsGrid(j)->getAngle(), iiammap[i]->getsGrid(j)->getAngleObs2sGrid());
            }
        }
    }

    int loop = 0;
    std::vector<ShootingGrid*> temp_shooting_girds;
    while(ros::ok()) {
        bool register_vector = false;
        std::sort(temp.begin(), temp.end(), obs_count_des); 
        for(int i=0; i < temp.size(); i++) {
            int x = temp[i]->getX();
            int y = temp[i]->getY();
            if(ShootingExcessJudgment(temp[i], temp_shooting_girds)) {
                if(!register_vector) register_vector = true;
                iiammap[XYtoP(x,y)]->setObservationPoint(); //観測点に追加
                iiammap[XYtoP(x,y)]->clearsGrid();
                geometry_msgs::Point32 point;
                point.x = origin.position.x + x * resolution + resolution / 2.0;
                point.y = origin.position.y + (height - y - 1) * resolution + resolution / 2.0;
                point.z = 0.0;
                (*observation_points).points.push_back(point);
                for(int j=0; j < temp[i]->getsGridNumber(); j++) {
                    shooted_grid.push_back(new ShootingGrid(temp[i]->getsGrid(j)->getX(),
                                                            temp[i]->getsGrid(j)->getY(),
                                                            temp[i]->getsGrid(j)->getAngle()));
                    iiammap[XYtoP(x,y)]->pushbacksGrid(temp[i]->getsGrid(j)->getX(), 
                                                       temp[i]->getsGrid(j)->getY(), 
                                                       temp[i]->getsGrid(j)->getAngle(),
                                                       0.0);
                    temp_shooting_girds.push_back(new ShootingGrid(temp[i]->getsGrid(j)->getX(), 
                                                                   temp[i]->getsGrid(j)->getY(),
                                                                   temp[i]->getsGrid(j)->getAngle()));
                }
            }

            UpdateShootedGridNumber(temp[i]);

        }
        
        loop++;
        int size = shooted_grid.size();
        ROS_INFO("loop: %d shooted_grid.size()=%d", loop, size);
        if(!register_vector) break;
        
    }

}

inline int iiam::PtoX(int p) { return p - (height - PtoY(p) - 1) * width; }

inline int iiam::PtoY(int p) { return height - p / width - 1; }
    
inline int iiam::XYtoP(int x,int y) { return x + (height - y - 1) * width; }
  
inline bool iiam::XYCheck(int x,int y){ return (x >= 0 && y >= 0 && x < width && y < height); }

void iiam::Init(float resolution) {

    request_resolution = 0.7;
    camera_width = 1280;
    camera_height = 960;
    camera_angle = 68.9;
    map_resolution = (int)(resolution * 1000.0);
    robot_size = 800;
    VectorForPoint = 17;
    chain_search = 5;
    areadensity = 100;   
    shooting_vector_number = 0;

    SetParameter();

}

int iiam::isGridAround(int p,float vector){
    
    int X = PtoX(p), Y = PtoY(p);
    
    if(X < 1 || Y < 1 || X+1 >= width || Y+1 >= height) return -1;
    
    vector=FixRadian_0to2Pi(vector);
    if(-_pi/8 <= vector && vector <= _pi/8 && (iiammap[XYtoP(X+1,Y)]->getCellType() != Free)) return XYtoP(X+1,Y);
    else if(_pi/8 <= vector && vector <= 3*_pi/8 && (iiammap[XYtoP(X+1,Y+1)]->getCellType() != Free)) return XYtoP(X+1,Y+1);
    else if(3*_pi/8 <= vector && vector <= 5*_pi/8 && (iiammap[XYtoP(X  ,Y+1)]->getCellType() != Free)) return XYtoP(X,Y+1);
    else if(5*_pi/8 <= vector && vector <= 7*_pi/8 && (iiammap[XYtoP(X-1,Y+1)]->getCellType() != Free)) return XYtoP(X-1,Y+1);
    else if(7*_pi/8 <= vector && vector <= 9*_pi/8 && (iiammap[XYtoP(X-1,Y)]->getCellType() != Free)) return XYtoP(X-1,Y);
    else if(9*_pi/8 <= vector && vector <= 11*_pi/8 && (iiammap[XYtoP(X-1,Y-1)]->getCellType() != Free)) return XYtoP(X-1,Y-1);
    else if(11*_pi/8 <= vector && vector <= 13*_pi/8 && (iiammap[XYtoP(X,Y-1)]->getCellType() != Free)) return XYtoP(X,Y-1);
    else if(13*_pi/8 <= vector && vector <= 15*_pi/8 && (iiammap[XYtoP(X+1,Y-1)]->getCellType() != Free)) return XYtoP(X+1,Y-1);
    else if(15*_pi/8 <= vector && vector <= 17*_pi/8 && (iiammap[XYtoP(X+1,Y)]->getCellType() != Free)) return XYtoP(X+1,Y);
        
    return -1;

}

void iiam::SetParameter() { 

    robot_area = (int)(float)(robot_size/map_resolution);
    alpha = camera_angle * _pi / 180.0; //カメラの画角(rad)
    beta = alpha * request_resolution * (float)map_resolution / (float)camera_width; //要求解像度を満たすのに最低限必要な角度(rad)
        
    float l = (float)robot_size;
    float lambda = (float)map_resolution / 2.0;
    float a = powf(l, 4.0) + 2.0 * powf(l, 2.0) * powf(lambda, 2.0) + powf(lambda, 4.0); // a = l^4 + 2 * l^2 * λ^2 + λ^4
    float b = a * powf(sinf(beta), 2.0) - 4 * powf(l, 2.0) * powf(lambda, 2.0); //b = a * sin^2(beta) - 4 * l^2 * λ^2
    float c = 4 * (powf(sinf(beta), 2.0) - 1.0) * powf(l, 2.0) * powf(lambda, 2.0); //c = 4 * (sin^2(beta) - 1) * l^2 * λ^2
    float theta = asinf(sqrtf(b / c)); // θ = asin(√(b / c)) (rad)
    int least_x = _pi / theta + 1; //最低限必要な撮影ベクトルの本数

    //表面形状の誤差を考慮して最低限必要な撮影ベクトルの本数を算出      
    for(int j = least_x + 1; j < VectorForPoint; j++) {
        float r = GetShootingArea(_pi/j);
        float error_resolution = GetResolution(r, 0.0, 2*_pi/j);
        ROS_INFO("j = %d, r = %f error_resolution = %f", j, r, error_resolution);
        if(error_resolution / request_resolution > 0.8 && r > robot_size / map_resolution) {
            VectorNum = j;
            break;
        }
    }
        
    SupportVectorInterval = 2.0 * _pi / VectorNum; //撮影ベクトルを付加する間隔
        
    ROS_INFO("VectorNum = %d, SupportVectorInterval = %f", VectorNum, SupportVectorInterval);
    
    return;

}

//x,y中心で半径をロボットの大きさの半径とする円の内部をロボットが進入できないエリアとして登録する
void iiam::SetCautionArea(int x, int y) {
    
    for(int com_x = x - robot_area; com_x <= x + robot_area; com_x++){
        for(int com_y = y-robot_area; com_y <= y + robot_area; com_y++){
            if(round_f(CalcDistancePointToPoint(x,com_x,y,com_y)) < robot_area + 1 && XYCheck(com_x,com_y)){
                iiammap[XYtoP(com_x, com_y)]->setCautionCell();
            }
        }
    }
    
    return;
    
}

//点セルに撮影ベクトルを付加
void iiam::SetPointVector(int p) {
   
    float vector_angle = 0.0;
    for(int i = 0; i < VectorNum; i++) {
        vector_angle += SupportVectorInterval * i;
        if(iiammap[p]->isSimilarVector(vector_angle,0.5) == false && isGridAround(p,vector_angle) == -1) {
            iiammap[p]->pushbackShootingVector(vector_angle);
        }
    }
    
    return;
    
}

//端点セルに撮影ベクトルを付加
void iiam::SetEdgeVector(int p, float edge_toward) {
    
    float vector_angle = 2 * _pi * edge_toward / 8;
    
    if(iiammap[p]->isSimilarVector(vector_angle,0.5)==false && isGridAround(p,vector_angle) == -1) {
	    iiammap[p]->pushbackShootingVector(vector_angle);
    }
    
    SetSupportVector(p);

	return;
    
}

//上記以外のセルに撮影ベクトルを付加
void iiam::SetNormalVector(int p, int first_toward) {
    
    int margin_count=0;
    int X = PtoX(p), Y = PtoY(p), x, y, x2, y2;
    int toward;
    float normal_vector;
    int chain[8][2]={ { 1, 0},
                      { 1, 1},
                      { 0, 1},
                      {-1, 1},
                      {-1, 0},
                      {-1,-1},
                      { 0,-1},
                      { 1,-1}};
                      
    //最初の方向決定
     x=X;
    y=Y;
    toward=first_toward;
    while(1){
        x2=x+chain[toward][0];
        y2=y+chain[toward][1];
        //画面の外に出たら
        if(XYCheck(x2,y2)==false){
        //グリッドがある
        }else if(iiammap[XYtoP(x2,y2)]->getCellType() != Free){
            margin_count=0;
        }else{
            margin_count++;
            //空間が2個以上連続した場合に形状探索
            if(margin_count == 2){
                if(toward-1 < 0){
                    normal_vector=ChainCode(p,toward-1+8);

                    if(normal_vector == -1.0){
                        SetPointVector(p);
                    }else{
                        //ベクトル定義
                        //似たベクトルがないか，周りにグリッドがないか
                        if(iiammap[p]->isSimilarVector(normal_vector,0.5) == false && isGridAround(p,normal_vector) == -1){
                            iiammap[p]->pushbackShootingVector(normal_vector);
                        }
                    }
                }else{
                    normal_vector=ChainCode(p,toward-1);

                    if(normal_vector==-1.0){
                        SetPointVector(p);
                    }else{
                        //ベクトル定義
                        //似たベクトルがないか，周りにグリッドがないか
                        if(iiammap[p]->isSimilarVector(normal_vector,0.5) == false && isGridAround(p,normal_vector) == -1){
                            iiammap[p]->pushbackShootingVector(normal_vector);
                        }
                    }
                }
            }
        }
        toward++;
        toward %= 8;
        if(toward == first_toward){	//1周したら
            return;
        }
    }
        
}

void iiam::SetSupportVector(int p) {
   
    int i,j;
    int support_vector_num;
    float support_vector_inteval;
    float vector_margin;

    if(iiammap[p]->getShootingVectorNumber() == 0){
        SetPointVector(p);
            
        return;
            
    }else if(iiammap[p]->getShootingVectorNumber() == 1){
        //ベクトル定義
        float support_vector_angle = iiammap[p]->getShootingVector(0).getAngle();
        for(int i=0; i < VectorNum; i++){
            support_vector_angle += SupportVectorInterval * (i+1);
            //似たベクトルがないか，周りにグリッドがないか
            if(iiammap[p]->isSimilarVector(support_vector_angle,0.5)==false && isGridAround(p, support_vector_angle) == -1){
                iiammap[p]->pushbackShootingVector(support_vector_angle);
            }
        }
	
        return;
		    
    }
		
    return;
    
}

//指定したoccupied cellの撮影ベクトルを定義
void iiam::SetShootingVector(int p) {
    
    int x = PtoX(p), y = PtoY(p);
        
    int toward = GetStartToward(x, y); //指定したセルの右隣から半時計回りに周囲のセルを見て,はじめにoccupied cellかunknown cellが見つかった方向
    //点セルの場合
    if(toward == -1) {
        SetPointVector(p);
        return;
    }
        
    int edge_toward = GetEdgeToward(x, y, toward);
    //端点セルの場合
    if(edge_toward != -1) {
        SetEdgeVector(p, edge_toward);
        return;
    }
        
    //普通のセル
    SetNormalVector(p,toward);
        
    return;
    
}

float iiam::GetShootingArea(float theta) { 
    return (cosf(theta) + sqrt(powf(cosf(theta), 2.0) + pow(tanf(beta), 2.0))) * 0.5 / tanf(beta);
}

//撮影解像度の計算
float iiam::GetResolution(float x, float y, float theta) { 
    float a = fabs(atan((y + cos(theta) / 2.0) / (x - sin(theta) / 2.0)) - atan((y - cos(theta) / 2.0) / (x + sin(theta) / 2.0)));
    return a * camera_width / (alpha * map_resolution);
}

//指定したセルの右隣から半時計回りに周囲のセルを見て、最初にoccupied cellかunknown cellが見つかった方向
//0:右 1:右上 2:上 3:左上 4:左 5:左下 6:下 7:右下 -1:周囲に上記のセルが存在しない(点セル)
int iiam::GetStartToward(int x, int y) {

    int toward = 0;    
    int chain[][2] = {{ 1, 0},
                      { 1, 1},
                      { 0, 1},
                      {-1, 1},
                      {-1, 0},
                      {-1,-1},
                      { 0,-1},
                      { 1,-1}};
                          
    while(1){
	    int X = x+chain[toward][0];
	    int Y = y+chain[toward][1];
	    //画面の外に出たら
	    if(XYCheck(X,Y) == false){
	    //グリッドがあるor未知空間グリッド
	    }else if(iiammap[XYtoP(X,Y)]->getCellType() != Free){
	        return toward;
	    }		    
	    toward++;
	    toward%=8;
	    if(toward==0) return -1;
	}
		
}

int iiam::GetEdgeToward(int x, int y, int first_toward) {
    
    int X,Y;
    int toward,edge_toward,start_toward,best_start_toward;
    int no_grid_count,max_no_grid_count;
    int chain[8][2]={{ 1, 0},
                     { 1, 1},
                     { 0, 1},
                     {-1, 1},
                     {-1, 0},
                     {-1,-1},
                     { 0,-1},
                     { 1,-1}};
                         
    toward=first_toward;
    start_toward=-1;
    no_grid_count=0; //周囲8近傍のうち連続してグリッドがない数をカウント
    max_no_grid_count=0;
        
    while(1){
        X=x+chain[toward][0];
        Y=y+chain[toward][1];
        //画面の外に出たら
        if(XYCheck(X,Y)==false){
            no_grid_count=0;
        //セルが存在する場合
        }else if(iiammap[XYtoP(X,Y)]->getCellType() != Free){
            start_toward=-1;
            no_grid_count=0;
        }else{
            if(start_toward==-1) start_toward=toward;
                no_grid_count++;
            }
        if(max_no_grid_count<no_grid_count){
            max_no_grid_count=no_grid_count;
            best_start_toward=start_toward;	//グリッドが無い空間の左端の方向
        }
        toward++;
        toward%=8;
        if(toward==first_toward) break; //1周したら
    }
        
    if(best_start_toward%2 == 0 && max_no_grid_count==6/*5*/) {	//凸形状
        return -1.0;
    } else if(max_no_grid_count<6/*5*/)
        return -1.0;
    else{
        edge_toward=best_start_toward+(max_no_grid_count-1)*0.5;	//端点
        while(edge_toward>8.0) edge_toward-=8.0;
        return edge_toward;
    }
        
}

void iiam::UpdateShootedGridNumber(GridClass *grid) {

    for(int i = 0; i < grid->getsGridNumber(); i++) {
        for(int j = shooted_grid.size() - 1; j >= 0; j--) {                
            if(grid->getsGrid(i)->getX() == shooted_grid[j]->getX() 
            && grid->getsGrid(i)->getY() == shooted_grid[j]->getY()
            && grid->getsGrid(i)->getAngle() == shooted_grid[j]->getAngle()) {
                grid->updateShootedVectorNumber();
                grid->erasesGrid(i);
                i--;
                break;
            }
        }
    }

    return;

}

//表面形状を推定し,撮影ベクトルの角度を算出(探索範囲小さい)
float iiam::ShortChainCode(int p,int first_toward) {

    int new_p;
    int X = PtoX(p),Y = PtoY(p),x,y,x2,y2,x_st,y_st;
    int end_x1,end_y1,end_x2,end_y2;
    int toward,toward_pre,cycle=0;
    int breakflag=0,near_edge_flag=0,checkflag=0,grid_count=0,edge_toward=-1,ignore_flag=1;
    int neighbor=1,count=0;
    int chain[8][2]={{ 1, 0},
                     { 1, 1},
                     { 0, 1},
                     {-1, 1},
                     {-1, 0},
                     {-1,-1},
                     { 0,-1},
                     { 1,-1}};
    int rechain[8][2]={{ 1,-1},
                       { 0,-1},
                       {-1,-1},
                       {-1, 0},
                       {-1, 1},
                       { 0, 1},
                       { 1, 1},
                       { 1, 0}};

    //正方向にチェイン符号化
    x=X;
    y=Y;
    x_st=X;
    y_st=Y;
    toward=first_toward;
    toward_pre=-10;
    count=0;
    //たどる
    while(breakflag==0){
        while(1){
            x2=x+chain[toward][0];
            y2=y+chain[toward][1];
            new_p=XYtoP(x2,y2);
            //画面の外に出たら
            if(XYCheck(x2,y2)==false){
                toward++;
                toward%=8;
            //スタートに戻ったら終わり
             }else if(y2==y_st && x2==x_st){
                return -1;
                break;
            //次の場所を見付けたら
            }else if(iiammap[new_p]->getCellType() != Free){
                //方向が極端に変わる場合→端点付近
                if(abs(abs(toward-toward_pre)-4)<=1){
                    neighbor=count;
                    near_edge_flag=1;
                    break;
                //次の点へ移動
                }else{
                    y=y2;
                    x=x2;
                    break;
                }
            //別の方向へ
            }else{
                toward++;
                toward%=8;
            }
        }
        toward_pre=toward;
        if(breakflag!=0) break;

        count++;
        if(count>=neighbor) break;

        //一つ前の二個次
        toward+=6;
        toward%=8;
    }
    end_x1=x;
    end_y1=y;

    //折り返し
    x=end_x1;
    y=end_y1;
    x_st=end_x1;
    y_st=end_y1;
    toward_pre=-10;
    toward=7-((toward+4)%8);
    count=0;
    //たどる
    while(breakflag==0){
        while(1){
            x2=x+rechain[toward][0];
            y2=y+rechain[toward][1];
            new_p=XYtoP(x2,y2);
            //画面の外に出たら
            if(XYCheck(x2,y2)==false){
                toward++;
                toward%=8;
            //スタートに戻ったら終わり
            }else if(y2==y_st && x2==x_st){
                return -1;
                break;
            //次の場所を見付けたら
            }else if(iiammap[new_p]->getCellType() != Free){
                //方向が極端に変わる場合→端点付近
                if(abs(abs(toward-toward_pre)-4)<=1){
                    neighbor=(int)(count/2);
                    near_edge_flag=1;
                    break;
                //次の点へ移動
                }else{
                    y=y2;
                    x=x2;
                    break;
                }
            //別の方向へ
            }else{
                toward++;
                toward%=8;
            }
        }
        toward_pre=toward;
        if(breakflag!=0) break;

        count++;
        if((int)(count/2)>=neighbor) break;

        //一つ前の二個次
        toward+=6;
        toward%=8;
    }
    end_x2=x;
    end_y2=y;

    //負方向で正方向より早く端点にいった場合
    if(near_edge_flag==1){

        //スタート地点
        x=X;
        y=Y;
        x_st=X;
        y_st=Y;
        toward=0;
        toward_pre=-1;
        count=0;
        //たどる
        while(breakflag==0){
            while(1){
                x2=x+chain[toward][0];
                y2=y+chain[toward][1];
                new_p=XYtoP(x2,y2);
                //画面の外に出たら
                if(XYCheck(x2,y2)==false){
                    toward++;
                    toward%=8;
                //スタートに戻ったら終わり
                }else if(y2==y_st && x2==x_st){
                    return -1;
                    break;
                    //次の場所を見付けたら
                }else if(iiammap[new_p]->getCellType() != Free){
                    //次の点へ移動
                    y=y2;
                    x=x2;
                    break;
                //別の方向へ
                }else{
                    toward++;
                    toward%=8;
                }
            }
            toward_pre=toward;
            if(breakflag!=0) break;

            count++;
            if(count>=neighbor) break;

            //一つ前の二個次
            toward+=6;
            toward%=8;
        }
        end_x1=x;
        end_y1=y;
    }

    float vector;
    int sideflag1,sideflag2;
          
    vector=atan2((float)(end_y2-end_y1),(float)(end_x2-end_x1))+_pi/2;
    if(vector==0){
        if(first_toward==7 || first_toward==0 || first_toward==1){
            vector=0.0;
        }else if(first_toward==3 || first_toward==4 || first_toward==5){
            vector=_pi;
        }else if(iiammap[XYtoP(X-1,Y)]->getCellType() != Free){
            vector=0.0;
        }else{
            vector=_pi;
        }
    }

    return vector;

}

//表面形状を推定し, 撮影ベクトルの方向を算出
float iiam::ChainCode(int p, int first_toward) {

    int new_p;
    int X = PtoX(p),Y = PtoY(p),x,y,x2,y2,x_st,y_st;
    int end_x1,end_y1,end_x2,end_y2;
    int toward,toward_pre,cycle=0;
    int breakflag=0,near_edge_flag=0,checkflag=0,grid_count=0,edge_toward=-1,ignore_flag=1;
    int neighbor=chain_search,count=0;
    int chain[8][2]={{ 1, 0},
                     { 1, 1},
                     { 0, 1},
                     {-1, 1},
                     {-1, 0},
                     {-1,-1},
                     { 0,-1},
                     { 1,-1}};
                         
    int rechain[8][2]={{ 1,-1},
                       { 0,-1},
                       {-1,-1},
                       {-1, 0},
                       {-1, 1},
                       { 0, 1},
                       { 1, 1},
                       { 1, 0}};

    //正方向にチェイン符号化
    x=X;
    y=Y;
    x_st=X;
    y_st=Y;
    toward=first_toward;
    toward_pre=-10;
    count=0;
    //たどる
    while(breakflag==0){
        while(1){
            x2=x+chain[toward][0];
            y2=y+chain[toward][1];
            new_p=XYtoP(x2,y2);
            //画面の外に出たら
            if(XYCheck(x2,y2)==false){
                toward++;
                toward%=8;
            //スタートに戻ったら終わり
            }else if(y2==y_st && x2==x_st){
                return ShortChainCode(p,first_toward);
                break;
            //次の場所を見付けたら
            }else if(iiammap[new_p]->getCellType() != Free){
                //方向が極端に変わる場合→端点付近
                if(abs(abs(toward-toward_pre)-4)<=1){ //元ソースabs(abs(toward-toward_pre)-4)<=2
                    neighbor=count;
                    near_edge_flag=1;
                    break;
                //次の点へ移動
                }else{
                    y=y2;
                    x=x2;
                    break;
                }
            //別の方向へ
            }else{
                toward++;
                toward%=8;
            }
        }
        toward_pre=toward;
        if(breakflag!=0) break;

        count++;
        if(count>=neighbor) break;

        //一つ前の二個次
        toward+=6;
        toward%=8;
    }
    end_x1=x;
    end_y1=y;

    //折り返し
    x=end_x1;
    y=end_y1;
    x_st=end_x1;
    y_st=end_y1;
    toward_pre=-10;
    toward=7-((toward+4)%8);
    count=0;
    //たどる
    while(breakflag==0){
        while(1){
            x2=x+rechain[toward][0];
            y2=y+rechain[toward][1];
            new_p=XYtoP(x2,y2);
            //画面の外に出たら
            if(XYCheck(x2,y2)==false){
                toward++;
                toward%=8;
            //スタートに戻ったら終わり
            }else if(y2==y_st && x2==x_st){
                return ShortChainCode(p,first_toward);
                break;
            //次の場所を見付けたら
            }else if(iiammap[new_p]->getCellType() != Free){
                //方向が極端に変わる場合→端点付近
                if(abs(abs(toward-toward_pre)-4)<=1){
                    neighbor=(int)(count/2);
                    near_edge_flag=1;
                    break;
                //次の点へ移動
                }else{
                    y=y2;
                    x=x2;
                    break;
                }
            //別の方向へ
            }else{
                toward++;
                toward%=8;
            }
        }
        toward_pre=toward;
        if(breakflag!=0) break;

        count++;
        if((int)(count/2)>=neighbor) break;

        //一つ前の二個次
        toward+=6;
        toward%=8;
    }
    end_x2=x;
    end_y2=y;

    //負方向で正方向より早く端点にいった場合
    if(near_edge_flag==1){
        //スタート地点
        x=X;
        y=Y;
        x_st=X;
        y_st=Y;
        toward=first_toward;
        toward_pre=-1;
        count=0;
        //たどる
        while(breakflag==0){
            while(1){
                x2=x+chain[toward][0];
                y2=y+chain[toward][1];
                new_p=XYtoP(x2,y2);
                //画面の外に出たら
                if(XYCheck(x2,y2)==false){
                    toward++;
                    toward%=8;
                //スタートに戻ったら終わり
                }else if(y2==y_st && x2==x_st){
                    return ShortChainCode(p,first_toward);
                    break;
                //次の場所を見付けたら
                }else if(iiammap[new_p]->getCellType() != Free){
                    //次の点へ移動
                    y=y2;
                    x=x2;
                    break;
                //別の方向へ
                }else{
                    toward++;
                    toward%=8;
                }
            }
            toward_pre=toward;
            if(breakflag!=0) break;

            count++;
            if(count>=neighbor) break;

            //一つ前の二個次
            toward+=6;
            toward%=8;
        }
        end_x1=x;
        end_y1=y;
    }

    float vector;
    int sideflag1,sideflag2;
            
    vector=atan2((float)(end_y2-end_y1),(float)(end_x2-end_x1))+_pi/2;
            
    if(vector==0) {
       if(first_toward==7 || first_toward==0 || first_toward==1){
           vector=0.0;
       }else if(first_toward==3 || first_toward==4 || first_toward==5){
           vector=_pi;
       }else if(iiammap[XYtoP(X-1,Y)]->getCellType() != Free){
           vector=0.0;
       }else{
           vector=_pi;
       }
   }

   return vector;

}

bool iiam::Triangle(int x1, int y1, int x2, int y2, int x3, int y3, float angle, bool *capture){

    int x,p;
    int *x_right,*x_left;
    int min_y=int_min(y1,int_min(y2,y3)), max_y=int_max(y1,int_max(y2,y3));

    x_right = (int *)malloc((max_y-min_y+1)*sizeof(int));
    x_left = (int *)malloc((max_y-min_y+1)*sizeof(int));

    for(int y=0;y<max_y-min_y;y++){
        x_right[y]=0;
        x_left[y]=width;
    }

    int p2 = XYtoP(x2, y2);

    for(int y=0;y<max_y-min_y;y++){
        x=CalcXonLine(x1,y1,x2,y2,y+min_y);
        if(x!=-1 && ((y1<=y+min_y && y+min_y<=y2)||(y2<=y+min_y && y+min_y<=y1))){
            if (x < x_left[y]) {
                x_left[y] = x;
            }
            if (x > x_right[y]) {
                x_right[y] = x;
            }
        }
    }
        
    for(int y=0;y<max_y-min_y;y++){
        x=CalcXonLine(x1,y1,x3,y3,y+min_y);
        if(x!=-1 && ((y1<=y+min_y && y+min_y<=y3)||(y3<=y+min_y && y+min_y<=y1))){
            if (x < x_left[y]) {
                x_left[y] = x;
            }
            if (x > x_right[y]) {
                x_right[y] = x;
            }
        }
    }

    for(int y=0;y<max_y-min_y;y++){
        x=CalcXonLine(x2,y2,x3,y3,y+min_y);
        if(x!=-1 && ((y2<=y+min_y && y+min_y<=y3)||(y3<=y+min_y && y+min_y<=y2))){
            if (x < x_left[y]) {
                x_left[y] = x;
            }
            if (x > x_right[y]) {
                x_right[y] = x;
            }
        }
    }

    /* ---------------観測点から撮影可能な撮影ベクトルを算出-------------- */
    if(y2 >= 0 && y1 >= 0 && y3 >= 0) {
        for(int y=0;y<max_y-min_y;y++){
            for(x=x_left[y];x<=x_right[y];x++){
                p=XYtoP(x,y+min_y);
                if(iiammap[p]->getCellType() == Free && iiammap[p]->isMovableCell()) { //自由空間グリッドなら
                    if(iiammap[p]->getsGridNumber() == 0) { //要素０の場合観測点から計測可能なグリッド追加
                        float angleObs2sGrid = CalcRadianPointToPoint(x, x2, y+min_y, y2);
                        //移動可能範囲グリッドから観測可能な撮影ベクトルにID付加（未付加のものに対してのみ）
                        for(int i=0; i < iiammap[p2]->getShootingVectorNumber(); i++) {	
                            if(iiammap[p2]->getShootingVector(i).getAngle() == angle) {
                                //if(iiam->m_map->P2[p2]->shooting_vector[i].id == -1) {	//ID未割当なら 
                                //iiam->m_map->P2[p2]->shooting_vector[i].id = id_num;
                                //id_num++;	//id割り当て数値インデント
                                iiammap[p]->pushbacksGrid(x2, y2, angle, angleObs2sGrid); //観測可能グリッドの追加
                                iiammap[p]->updateObsCount();
                                iiammap[p]->setProcessCell(true);
                                if(!(*capture)) *capture = true; 
							}
                        }
                    } else {
                    //追加済みの撮影ベクトルかどうか
                        bool samevector = false;
                        for(int i=0; i < iiammap[p]->getsGridNumber(); i++){
                            if(iiammap[p]->getsGrid(i)->getX() == x2 && iiammap[p]->getsGrid(i)->getY() == y2 
                            && iiammap[p]->getsGrid(i)->getAngle() == angle) {
                                samevector=true;
                            }
                        }
                        //未追加のベクトルなら 
                        if(!samevector) {
                            float angleObs2sGrid = CalcRadianPointToPoint(x, x2, y+min_y, y2);
                            //移動可能範囲グリッドから観測可能な撮影ベクトルにID付加（未付加のものに対してのみ）
                            for(int i=0; i < iiammap[p2]->getShootingVectorNumber(); i++) {	
                                if(iiammap[p2]->getShootingVector(i).getAngle() == angle) {
                                     //if(iiam->m_map->P2[p2]->shooting_vector[i].id == -1) {	//ID未割当ならば
                                     //iiam->m_map->P2[p2]->shooting_vector[i].id = id_num;
                                     //id_num++;
                                    iiammap[p]->pushbacksGrid(x2, y2, angle, angleObs2sGrid);	//観測可能グリッド(ベクトル）の追加
                                    iiammap[p]->updateObsCount();
                                    iiammap[p]->setProcessCell(true);
                                    if(!(*capture)) *capture = true; 
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    free(x_right);
    free(x_left);
        
    return true;
        
}

//撮影ベクトルの撮影範囲内にあるfree cellの情報を更新
void iiam::ShootingArea(int p) {

    int X = PtoX(p),Y = PtoY(p),x,y;
    int degree=areadensity;
    int temp_x_point,temp_y_point;

    for(int i = 0; i < iiammap[p]->getShootingVectorNumber(); i++){
        bool capture_angle = false;
        for(int l=-degree/2+1;l<degree/2;l++){
            float th=l*_pi/degree;
            x=round_f(GetShootingArea(th) * cos(th+iiammap[p]->getShootingVector(i).getAngle())+X);
            y=round_f(GetShootingArea(th) * sin(th+iiammap[p]->getShootingVector(i).getAngle())+Y);
            if(XYCheck(X, Y) && XYCheck(x, y)){
                if(l!=-degree/2+1) {
                    if(!Triangle(temp_x_point,temp_y_point,X,Y,x,y, iiammap[p]->getShootingVector(i).getAngle(), &capture_angle)) break;
                }
            }
            temp_x_point=x;
            temp_y_point=y;
        }
        if(capture_angle) { 
            shooting_vector_number++;
            iiammap[p]->getShootingVector(i).notPossibleShooting();
        }
    }
        
    iiammap[p]->setProcessCell(false);
        
    return;
       
}

bool iiam::ShootingExcessJudgment(GridClass *grid, std::vector<ShootingGrid*> temp_shooting_girds) {

    if(grid->getObsCount() - grid->getShootedVectorNumber() <= 0) return false;
        
    for(int i = 0; i < grid->getsGridNumber(); i++) {
        for(int j = 0; j < temp_shooting_girds.size(); j++) {
            if(grid->getsGrid(i)->getX() == temp_shooting_girds[j]->getX() 
            && grid->getsGrid(i)->getY() == temp_shooting_girds[j]->getY() 
            && grid->getsGrid(i)->getAngle() == temp_shooting_girds[j]->getAngle()) return false;
        }
    }
   
    return true;
}

bool obs_count_des(GridClass* left, GridClass* right) {
    return (left->getObsCount() - left->getShootedVectorNumber()) > (right->getObsCount() - right->getShootedVectorNumber());
}

