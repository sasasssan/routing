#include "iiam/calculation.h"
#include <assert.h>

std::string* GetDateString(){
	struct tm *date;
	char buf[100];

	time_t now;

	int year, month, day;
	int hour, minute, second;


	time(&now);
	date = localtime(&now);


	year = date->tm_year + 1900;
	month = date->tm_mon + 1;
	day = date->tm_mday;
	hour = date->tm_hour;
	minute = date->tm_min;
	second = date->tm_sec;


	sprintf(buf,"%04d_%02d_%02d_%02d_%02d_%02d", year, month, day, hour, minute, second);
	return new std::string(buf); 

}


/*float型を四捨五入にしてint型で返す*/
int round_f(float x) {
    if ( x > 0.0 ) {
        return (int)( floor(x + 0.5) );
    } else {
        return (int)( -1.0 * floor(fabs(x) + 0.5) );
    }
}
/*int型の2つの値のうち大きいほうを返す*/
int int_max(int a, int b){
	if(a>b)
		return a;
	else
		return b;
}
/*int型の2つの値のうち小さいほうを返す*/
int int_min(int a, int b){
	if(a>b)
		return b;
	else
		return a;
}
/*float型の2つの値のうち大きいほうを返す*/
float float_max(float a, float b){
	if(a>b)
		return a;
	else
		return b;
}
/*float型の2つの値のうち小さいほうを返す*/
float float_min(float a, float b){
	if(a>b)
		return b;
	else
		return a;
}




/*ラジアンを0からπに*/
float FixRadian_0toPi(float radian){
	int count=0;
	while(radian>_pi || radian<0){

		//if(count++>5) break;
		assert(count++<20);
		if(radian>_pi*2)
			radian-=2*_pi;
		if(radian>_pi)
			radian-=_pi;
		if(radian<0.0)
			radian+=_pi;
	}

	return radian;
}
/*ラジアンを0から2πに*/
float FixRadian_0to2Pi(float radian){
	int count=0;
	while(radian>2*_pi || radian<0){

		//if(count++>5) break;
		assert(count++<20);
		if(radian>2*_pi)
			radian-=2*_pi;
		if(radian<0.0)
			radian+=2*_pi;
	}

	return radian;
}
/*ラジアンを-πからπに*/
float FixRadian_minusPitoPi(float radian){
	int count=0;
	while(radian>_pi || radian<-_pi){

		//assert(count++<20);
		if(count++>5) break;
		if(radian>_pi)
			radian-=2*_pi;
		if(radian<-_pi)
			radian+=2*_pi;
	}
	
	return radian;
}

/*2直線の交点*/
Point* CalcIntersectionPoint(float X,float Y,float vector,Line line){
	float a_x;
	float a_y;
	float temp;
		
	float x1=line.x1;
	float y1=line.y1;
	float x2=line.x2;
	float y2=line.y2;
	if(CalcRadianAbsoluteGap(FixRadian_0toPi(vector),_pi/2)<0.01){
		temp=(y2-y1)/(x2-x1);
		a_x=X;
		a_y=temp*(a_x-x1)+y1;
		return new Point(a_x,a_y);
	}else if(x1==x2){
		return new Point(x1,tan(vector)*x1+Y-tan(vector)*X);
	}else{
		temp=(y2-y1)/(x2-x1);
		a_x=(Y-tan(vector)*X-y1+temp*x1)/(temp-tan(vector));
		a_y=tan(vector)*a_x+Y-tan(vector)*X;
		
		return new Point(a_x,a_y);
	}
}
/*ある直線とy軸に平行な線の交点のx座標を求める*/
int CalcXonLine(int x1, int y1, int x2, int y2, int y){
	if(y1==y2)
		return -1;
	//printf("x=%d\n",(int)( x1+(x2-x1)*(y-y1)/(y2-y1) ));
	return (int)( x1+(x2-x1)*(y-y1)/(y2-y1) );
}
/*2点間の距離を求める*/
float CalcDistancePointToPoint(int x1,int x2,int y1,int y2){
	return sqrt((float)((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}
/*2点間の距離を求める*/
float CalcDistancePointToPoint(float x1,float x2,float y1,float y2){
	return sqrt((float)((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}
/*点(x,y)と直線(n,X,Y)の距離を求める*/
float CalcDistancePointToLine(float n,int X,int Y,int x,int y){
	n=tan(n);
	return abs(y-n*x-Y+n*X)/sqrt(1+n*n);
}
/*点と線分の距離を求める*/
float CalcDistancePointToSegment(int X,int Y,int x1,int y1,int x2,int y2){
    float dx,dy,a,b,t,tx,ty;
    float distance;
    dx =(float) (x2 - x1); dy = (float)(y2 - y1);
    a = dx*dx + dy*dy;
    b = dx * (x1 - X) + dy * (y1 - Y);
    t = -b / a;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    tx = x1 + dx * t;
    ty = y1 + dy * t;
    distance = sqrt((X - tx)*(X - tx) + (Y - ty)*(Y - ty));
    return distance;
}
/*2つの角度(ラジアン)の差の小さいほうを絶対値で求める*/
float CalcRadianAbsoluteGap(float vector1,float vector2){
	vector1=FixRadian_0to2Pi(vector1);
	vector2=FixRadian_0to2Pi(vector2);
	if(abs(vector1-vector2)<_pi)
		return abs(vector1-vector2);
	else
		return 2*_pi-abs(vector1-vector2);

}
/*2つの角度(ラジアン)の差の小さいほうを求める*/
float CalcRadianGap(float vector1,float vector2){
	vector1=FixRadian_0to2Pi(vector1);
	vector2=FixRadian_0to2Pi(vector2);
	float answer;
	if(abs(vector1-vector2)<_pi)
		answer=vector1-vector2;
	else{
		if(vector1>vector2)
			answer=-(2*_pi-(vector1-vector2));
		else
			answer=2*_pi-(vector2-vector1);
	}
	//if(answer>0 && answer>ALPHA)
	//	answer-=_pi;

	//if(answer<0 && answer<-ALPHA)
	//	answer+=_pi;

	return answer;
}
/*傾き(radian)の直線が各グリッドを通過するか判断するために必要な距離を求める*/
float CalcDistanceToPassGrid(float radian){
	float r;
	r=FixRadian_0toPi(radian);
	if(r>_pi/2)
		r-=_pi/2;
	if(r>_pi/4)
		r=_pi/2-r;
	return (float)(1.0/sqrt(2.0)*cos(_pi/4-r));
}
/*半径無限遠の扇形の中*/
bool InFan(float x,float y,float range1,float range2){
	/*float w1;
	float angle=FixRadian_0to2Pi(atan2( y , x ));
	range1=FixRadian_0to2Pi(range1);
	range2=FixRadian_0to2Pi(range2);
	if(range1<range2)	w1=CAMERA_WIDTH*CalcRadianGap( (range1+range2)*0.5, angle ) / ALPHA + CAMERA_WIDTH/2;
	else				w1=CAMERA_WIDTH*CalcRadianGap( (range1+range2)*0.5-_pi, angle ) / ALPHA + CAMERA_WIDTH/2;
	if(range1<range2){
		if(range1<=angle){
			if(angle<=range2){
				if(w1<0 || w1>CAMERA_WIDTH)
					return false;
				return true;
			}
		}
		if(range1>=angle){
			if(angle>=range2){
				if(w1<0 || w1>CAMERA_WIDTH)
					return false;
				return true;
			}
		}
	}
	if(range2<range1){
		if(range1<=angle){
			if(range2<=angle){
				if(w1<0 || w1>CAMERA_WIDTH)
					return false;
				return true;
			}
		}
		if(range1>=angle){
			if(range2>=angle){
				if(w1<0 || w1>CAMERA_WIDTH)
					return false;
				return true;
			}
		}
	}
	return false;*/
	float angle=FixRadian_minusPitoPi(atan2(y,x));
	if (range2 >= _pi || range1 <= -_pi) {
		//std::cout << "*range2 = " << (range2/_pi)*180 << " range1 = " << (range1/_pi)*180 << " angle = " << (angle/_pi)*180 << std::endl;
		if (FixRadian_minusPitoPi(range2) >= angle || FixRadian_minusPitoPi(range1) < angle) return true;
		else return false;
	}
	else {
		//std::cout << "range2 = " << (range2/_pi)*180 << " range1 = " << (range1/_pi)*180 << " angle = " << (angle/_pi)*180 << std::endl;
		if (FixRadian_minusPitoPi(range2) >= angle && FixRadian_minusPitoPi(range1) < angle) return true;
		else return false;
	}

}
/*aグリッド（を基準に）とbグリッドのなす角度を求める*/
float CalcRadianPointToPoint(const int x1, const int x2, const int y1, const int y2) {
	return atan2((float)(y2 - y1), (float)(x2 - x1));
}
