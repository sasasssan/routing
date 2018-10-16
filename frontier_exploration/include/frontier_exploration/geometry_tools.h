#ifndef GEOMETRY_TOOLS_H_
#define GEOMETRY_TOOLS_H_

#include <boost/foreach.hpp>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration{

  /**
  * @2点間の距離を計算する
  * @param one Point one
  * @param two Point two
  * @return 2点間の距離
  */
  template<typename T, typename S>
  double pointsDistance(const T &one, const S &two){
      return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
  }

  /**
  * @ポリゴンの外周を計算する
  * @param polygon 処理するポリゴン
  * @return ポリゴンの周囲
  */
  double polygonPerimeter(const geometry_msgs::Polygon &polygon){

      double perimeter = 0;
      if(polygon.points.size()   > 1)
      {
        for (int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
        {
          perimeter += pointsDistance(polygon.points[i], polygon.points[j]);
        }
      }
      return perimeter;
  }

/**
* @指定された近接距離内に2つの点がおおよそ隣接しているかどうかを評価する。
* @param one Point one
* @param two Point two
* @param proximity 近接距離
* @return ほぼ隣接する場合はtrue、そうでない場合はfalse
*/
  template<typename T, typename S>
  bool pointsNearby(const T &one, const S &two, const double &proximity){
      return pointsDistance(one, two) <= proximity;
  }
  
  /**
  * @ポイントが指定された近接距離内でリストの任意のポイントにほぼ接近しているかどうかを評価します。
  * @param one Point one
  * @param list ポイントのリスト
  * @param proximity 近接距離
  * @return ほぼ隣接する場合はtrue、そうでない場合はfalse
  */
  template<typename T, typename S>
  bool anyPointsNearby(const T &one, const std::list<S> &list, const double &proximity){
      BOOST_FOREACH(S two, list){
          if (pointsNearby(one, two, proximity)) {
              return true;
          }
      }
      return false;
  }

/**
* @点がポリゴンで定義された領域の内側にあるかどうかを評価する。 ライン上のポイントの未定義の動作。
* @param point テストのポイント
* @param polygon テストするポリゴン
* @return 点がポリゴンの内側にある場合はtrue、そうでない場合はfalse
*/
  template<typename T>
  bool pointInPolygon(const T &point, const geometry_msgs::Polygon &polygon){
      int cross = 0;
      for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
          if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
              (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
              cross++;
          }
      }
      return bool(cross % 2);
  }

/**
* @原点と終点で定義されるベクトルのヨーを計算する
* @param origin 原点
* @param end 終点
* @return ベクトルのヨー角
*/
  template<typename T, typename S>
  double yawOfVector(const T &origin, const S &end){

      double delta_x, delta_y;
      delta_x = end.x - origin.x;
      delta_y = end.y - origin.y;

      double yaw = atan(delta_x/delta_y);

      if(delta_x < 0){
          yaw = M_PI-yaw;
      }

      return yaw;
  }

}

#endif
