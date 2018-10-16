#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string>

#define _pi					((float)(3.141592653589793))

/*直線のクラス*/
class Line{
public:
	Line(int s_x1,int s_y1,int s_x2,int s_y2){
		x1=s_x1;
		y1=s_y1;
		x2=s_x2;
		y2=s_y2;
	};
	int x1;
	int y1;
	int x2;
	int y2;
};
/*二次元点のクラス*/
class Point{
public:
	Point(){};
	Point(Point* s_p){
		x=s_p->x;
		y=s_p->y;
	};
	Point(float s_x,float s_y){
		x=s_x;
		y=s_y;
	};
	float x;
	float y;
};


/*現在の時刻をstring型で取得 */
std::string* GetDateString();



/*変数処理関数*/;

/*float型を四捨五入にしてint型で返す*/
int round_f(float x);
/*int型の2つの値のうち大きいほうを返す*/
int int_max(int a, int b);
/*int型の2つの値のうち小さいほうを返す*/
int int_min(int a, int b);
/*float型の2つの値のうち大きいほうを返す*/
float float_max(float a, float b);
/*float型の2つの値のうち小さいほうを返す*/
float float_min(float a, float b);



/*角度補正関数*/;

/*ラジアンを0からπに*/
float FixRadian_0toPi(float radian);
/*ラジアンを0から2πに*/
float FixRadian_0to2Pi(float radian);
/*ラジアンを-πからπに*/
float FixRadian_minusPitoPi(float radian);


/*座標処理関数*/;

/*2直線の交点*/
Point* CalcIntersectionPoint(float X,float Y,float vector,Line line);
/*ある直線とy軸に平行な線の交点のx座標を求める*/
int CalcXonLine(int x1, int y1, int x2, int y2, int y);
/*2点間の距離を求める*/
float CalcDistancePointToPoint(int x1,int x2,int y1,int y2);
/*2点間の距離を求める*/
float CalcDistancePointToPoint(float x1,float x2,float y1,float y2);
/*点(x,y)と直線(n,X,Y)の距離を求める*/
float CalcDistancePointToLine(float n,int X,int Y,int x,int y);
/*点と線分の距離を求める*/
float CalcDistancePointToSegment(int X,int Y,int x1,int y1,int x2,int y2);
/*2つの角度(ラジアン)の差の小さいほうを絶対値で求める*/
float CalcRadianAbsoluteGap(float vector1,float vector2);
/*2つの角度(ラジアン)の差の小さいほうを求める*/
float CalcRadianGap(float vector1,float vector2);
/*傾き(radian)の直線が各グリッドを通過するか判断するために必要な距離を求める*/
float CalcDistanceToPassGrid(float radian);
/*半径無限遠の扇形の中*/
bool InFan(float x,float y,float range1,float range2);
/*2点間のなす角度を求める（１を基準）*/
float CalcRadianPointToPoint(const int x1, const int x2, const int y1, const int y2);
