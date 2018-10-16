#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
  
class VectorClass {
  private :
    float angle;
    bool possible_shooting;
          
  public :
    VectorClass();
    VectorClass(float angle);
    ~VectorClass();
    void notPossibleShooting();
    float getAngle();
    bool isPossibleShooting();
};


class ShootingGrid {
  private :
    int x; //撮影範囲内グリッドのx座標[pixel]
    int y; //撮影範囲内グリッドのy座標[pixel]
    int id; //部分集合問題に利用する要素としてのID
    float angle; //撮影ベクトルの角度[rad]
    float angleObs2sGrid; //観測点と撮影可能範囲グリッドの成す角度	
    bool shootedgrid; //すでに登録されているかどうか
  
  public :
    ShootingGrid(int x, int y);
    ShootingGrid(int x, int y, float angle);
    ShootingGrid(int x, int y, float angle, float angleObs2sGrid);
    ~ShootingGrid();
    int getX();
    int getY();
    int getID();
    float getAngle();
    float getAngleObs2sGrid();
    bool getShootedGrid();
    void setShootedGrid();
    void setShootedGrid(bool type);
};

class GridClass {
  private :
    int x, y; //グリッドにおけるx,y
    short int cell_type; //セルの種類 0:free 1:occupied 2:unknown
    bool movable_cell; //ロボットが進入できるセルかどうか
    bool process_cell; //処理すべきセルかどうか
    bool observation_point; //観測点として登録されているかどうか
    int obs_count; //撮影できる撮影ベクトルの数(撮影範囲の重なり回数)
    int shooted_vector_number; //撮影できる撮影ベクトルのうち,すでに登録済みの撮影ベクトルの数
    std::vector<VectorClass> shooting_vector; //付加された撮影ベクトル(occupied cellのみ使用)
    std::vector<ShootingGrid*> sGrid; //撮影することができる撮影ベクトルが付加されたセル
      
  public :
    GridClass();
    GridClass(int x,int y);
    ~GridClass(void);
    void setCellType(short int type);
    void setMovableCell(bool type);
    void setCautionCell();
    void setProcessCell(bool process);
    void setObsCount(int count);
    void setShootedVectorNumber(int count);
    void setObservationPoint();
    void updateObsCount();
    void updateShootedVectorNumber();
    void pushbackShootingVector(float vector);
    void pushbacksGrid(int x, int y, float angle, float angleObs2sgrid);
    void erasesGrid(int number);
    void clearsGrid();
    int getX();
    int getY();
    VectorClass getShootingVector(int number);
    int getShootingVectorNumber();
    ShootingGrid *getsGrid(int number);
    int getsGridNumber();
    int getObsCount();
    int getShootedVectorNumber();
    short int getCellType();
    bool isProcessCell();
    bool isMovableCell();
    bool isObservationPoint();
    bool isSimilarVector(float src_vector, float cos_interval);
    bool hasShootingVector();
};

class iiam {
  public :
    enum Space {Free = 0, Occupied = 100, Object = 50, Unknown = -1};

  private :
    std::vector<GridClass *>iiammap;
    std::vector<ShootingGrid*> shooted_grid; //観測点に登録済みの撮影ベクトル
    int width, height;
    float resolution;
    geometry_msgs::Pose origin;
    
    float request_resolution; //要求解像度
    int camera_width, camera_height; //カメラの横解像度, 縦解像度(pixel)
    float camera_angle; //カメラの画角(度)
    int map_resolution; //地図のresolution(mm)
    float alpha, beta; //カメラの画角(rad), 要求解像度を満たすのに最低限必要な角度(rad)
    int robot_size, robot_area; //ロボットの半径(mm) ,ロボットの半径(セル)
    int VectorNum, VectorForPoint; //点セルの撮影ベクトルの数, 最大数
    float SupportVectorInterval; //補助ベクトルを追加する間隔(rad)
    int chain_search; //表面形状の探索距離(chain codeの探索範囲)
    int areadensity; //撮影可能範囲の精度
    int shooting_vector_number; //撮影ベクトルの数
    
  public :
    iiam(float resolution);
    ~iiam();
    void MakeIIAM(const nav_msgs::OccupancyGrid &map, geometry_msgs::Polygon *observation_points);
    void GetObservationPoint(geometry_msgs::Polygon *observation_points);
    
  private :
    inline int PtoX(int p);
    inline int PtoY(int p); 
    inline int XYtoP(int x,int y);
    inline bool XYCheck(int x,int y);
    void Init(float resolution);
    int isGridAround(int p,float vector);
    void SetParameter();
    void SetCautionArea(int x, int y);
    void SetPointVector(int p);
    void SetEdgeVector(int p, float edge_toward);
    void SetNormalVector(int p, int first_toward);
    void SetSupportVector(int p);
    void SetShootingVector(int p);
    float GetShootingArea(float theta);
    float GetResolution(float x, float y, float theta);
    int GetStartToward(int x, int y);
    int GetEdgeToward(int x, int y, int first_toward);
    void UpdateShootedGridNumber(GridClass *grid);
    float ShortChainCode(int p,int first_toward);
    float ChainCode(int p, int first_toward);
    bool Triangle(int x1, int y1, int x2, int y2, int x3, int y3, float angle, bool *capture);
    void ShootingArea(int p);
    bool ShootingExcessJudgment(GridClass *grid, std::vector<ShootingGrid*> temp_shooting_girds);
    
};

