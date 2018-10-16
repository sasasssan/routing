/* 指定された範囲に格子状にwaypointを配置し保存する */
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>

//yamlライブラリを使用するためのテンプレート 気にしなくても良い
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

//waypointの構造体を定義
typedef struct {
	int flag;
	geometry_msgs::PoseStamped pose;
} Waypoint;

class Grid_routing {
    private:
        std::vector<Waypoint> make_waypoint, make_gridpoint;//格子点を格納する配列
        geometry_msgs::PoseStamped finish_pose_;//終了姿勢
        std::string save_filename;//waypointを保存するyamlファイル名
        visualization_msgs::MarkerArray waypoints_marker, grid_marker;//rviz上でwaypointを可視化する
        visualization_msgs::Marker clickedpoint_marker;//rviz上で選択領域を可視化する      
        float length;//waypointを配置する間隔

        ros::Publisher checkpoint_pub, waypoints_pub, gridpoints_pub, clickedpoints_pub;
        ros::Subscriber map_sub, griddata_sub, clickedpoint_sub;
        int grid_data;//map.pgmのピクセル情報
        int clickedpoint_flag;
        geometry_msgs::Point square[2];//選択した点を格納する配列、「選択領域を含む長方形」の最小と最大の対角点を格納する配列
        std::vector<geometry_msgs::Point> region;
        bool map_connect, griddata_flag;
            


        bool isInRegion(const geometry_msgs::Pose &point){//対象の格子点が選択領域内に存在するか判定
            int cross = 0;
            for (int i = 0, j = region.size() - 1; i < region.size(); j = i++) {
                if ( ((region[i].y > point.position.y) != (region[j].y>point.position.y)) &&
                    (point.position.x < (region[j].x-region[i].x) * (point.position.y-region[i].y) / (region[j].y-region[i].y) + region[i].x) ){
                    cross++;
                }
            }
            return bool(cross % 2);
        }
        
        void makeGridRoute(){//選択領域内の格子点の座標をmake_waypointに格納していく
            float  last_row;
            float  last_line;
            int row, line;
            Waypoint tmp;
            Waypoint tmp_[4];

            /*「選択領域を含む長方形」の、最大と最小の対角点をsquareに格納する*/
            square[0] = square[1] = region[0];
            //square[0].y = region[0].y;
            //square[1].x = region[0].x;
            //square[1].y = region[0].y;
            for(int i = 1 ; i < region.size() - 1; i++){
                if(square[0].x > region[i].x)square[0].x = region[i].x;
                if(square[0].y > region[i].y)square[0].y = region[i].y;
                if(square[1].x < region[i].x)square[1].x = region[i].x;
                if(square[1].y < region[i].y)square[1].y = region[i].y;
            }


            int width = fabs(square[1].x - square[0].x) /length, height = fabs(square[1].y - square[0].y) /length;

            tmp.pose.pose.position.x = square[0].x;
            tmp.pose.pose.position.y = square[0].y;
            tmp.pose.pose.position.z = 0.0;
            tmp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);            
            tmp.flag = 0;

            
            for(row = 0; row <= width; row++){
                tmp.pose.pose.position.x = square[0].x + row * length + length/2;
                
                for(line = 0; line <= height; line++){
                    tmp.pose.pose.position.y = square[0].y + line * length + length/2;

                    tmp_[0].pose.pose.position.x = tmp.pose.pose.position.x - length/2;
                    tmp_[0].pose.pose.position.y = tmp.pose.pose.position.y - length/2;
                    tmp_[1].pose.pose.position.x = tmp.pose.pose.position.x + length/2;
                    tmp_[1].pose.pose.position.y = tmp.pose.pose.position.y - length/2;
                    tmp_[2].pose.pose.position.x = tmp.pose.pose.position.x - length/2;
                    tmp_[2].pose.pose.position.y = tmp.pose.pose.position.y + length/2;
                    tmp_[3].pose.pose.position.x = tmp.pose.pose.position.x + length/2;
                    tmp_[3].pose.pose.position.y = tmp.pose.pose.position.y + length/2;



                    if(isInRegion(tmp_[0].pose.pose) || isInRegion(tmp_[1].pose.pose) ||
                        isInRegion(tmp_[2].pose.pose) || isInRegion(tmp_[3].pose.pose)){
                        make_waypoint.push_back(tmp);

                        for(int i = 0; i < 4; i++){
                            make_gridpoint.push_back(tmp_[i]);
                        }
                            
                        last_row = tmp.pose.pose.position.x;
                        last_line = tmp.pose.pose.position.y;
                    }
                }

            }


            finish_pose_.pose.position.x = last_row;
            finish_pose_.pose.position.y = last_line;
            finish_pose_.pose.position.z = tmp.pose.pose.position.z;
            finish_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        }

        visualization_msgs::Marker makeMarker(float x, float y, int id) {
            
                visualization_msgs::Marker marker, label;
                marker.header.frame_id = std::string("/map");
                marker.header.stamp = ros::Time::now();
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.pose.position.z = marker.scale.z / 2.0;
                marker.color.r = 0.8f;
                marker.color.g = 0.2f;
                marker.color.b = 0.2f;
              
                std::stringstream name;
                name << "waypoints" << id;
                marker.ns = name.str();
                marker.id = id;
                marker.pose.position.x = x;
                marker.pose.position.y = y; 
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.a = 1.0f;
               
                return marker;
                    
        }

        visualization_msgs::Marker makeGridMarker(float x, float y, int id) {
            
                visualization_msgs::Marker marker, label;
                marker.header.frame_id = std::string("/map");
                marker.header.stamp = ros::Time::now();
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.pose.position.z = marker.scale.z / 2.0;
                marker.color.r = 0.8f;
                marker.color.g = 0.8f;
                marker.color.b = 0.8f;
              
                std::stringstream name;
                name << "gridpoints" << id;
                marker.ns = name.str();
                marker.id = id;
                marker.pose.position.x = x;
                marker.pose.position.y = y; 
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.a = 1.0f;
               
                return marker;
                    
        }

        void makeGridpointMarker(){
    		grid_marker.markers.clear();
            for(int i = 0; i < make_gridpoint.size(); i++){
                grid_marker.markers.push_back(makeGridMarker(make_gridpoint[i].pose.pose.position.x, make_gridpoint[i].pose.pose.position.y, i));
            }
            gridpoints_pub.publish(grid_marker);
        }

        void makeWaypointMarker(){
    		waypoints_marker.markers.clear();
            for(int i = 0; i < make_waypoint.size(); i++){
                waypoints_marker.markers.push_back(makeMarker(make_waypoint[i].pose.pose.position.x, make_waypoint[i].pose.pose.position.y, i));
            }
            waypoints_pub.publish(waypoints_marker);
        }

        void GetMapCallback(const nav_msgs::OccupancyGrid &map){
            map_connect = true;
            return;
        }

    	void GriddataCallback(const std_msgs::Int16 &data){

            grid_data = data.data;
            griddata_flag = true;
            return;

        }

        void ClickedpointCallback(const geometry_msgs::PointStamped &point){
            region.push_back(point.point);
            clickedpoint_marker.points.push_back(point.point);
            clickedpoints_pub.publish(clickedpoint_marker);

            clickedpoint_flag++;
            if(clickedpoint_flag == 5){
                region.push_back(region[0]);
                clickedpoint_marker.points.push_back(region[5]);
                clickedpoints_pub.publish(clickedpoint_marker);
            }
        }

    public:
        Grid_routing(){
            ros::NodeHandle nh;
            checkpoint_pub = nh.advertise<geometry_msgs::Pose>("check_point", 1);
            waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
            gridpoints_pub = nh.advertise<visualization_msgs::MarkerArray>("gridpoints", 1);

            clickedpoints_pub = nh.advertise<visualization_msgs::Marker>("clickedpoints", 1);         
			map_sub = nh.subscribe("map", 1, &Grid_routing::GetMapCallback, this);            
            griddata_sub = nh.subscribe("grid_data", 1, &Grid_routing::GriddataCallback, this);
            clickedpoint_sub = nh.subscribe("clicked_point", 4, &Grid_routing::ClickedpointCallback, this);
            
            ros::NodeHandle private_nh("~");
            private_nh.param("length", length, length);
            private_nh.param("save_filename", save_filename, std::string("grind_routing.yaml"));

            /*visualization_msgs::Markerの初期設定*/
            clickedpoint_marker.header.frame_id = "/map";
            clickedpoint_marker.header.stamp = ros::Time();
            clickedpoint_marker.ns = "clickedpoints";
            clickedpoint_marker.id = 0;
            clickedpoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
            clickedpoint_marker.action = visualization_msgs::Marker::ADD;
            clickedpoint_marker.scale.x = 0.1;
            clickedpoint_marker.color.a = 1.0;
            clickedpoint_marker.color.r = 0.0;
            clickedpoint_marker.color.g = 0.0;
            clickedpoint_marker.color.b = 1.0;
         
            map_connect = false;
            
        }

        void run(){
            //mapを受け取る
			while(ros::ok() && !map_connect) {
				ros::spinOnce();
			}

            for(clickedpoint_flag = 0; ros::ok() && clickedpoint_flag < 5; ){
                ros::spinOnce();
            }
            clickedpoint_flag = 0;
            makeGridRoute();
                        
            makeWaypointMarker();
            makeGridpointMarker();

        }

};

int main(int argc, char** argv){
    
        ros::init(argc, argv, "grid_routing");
        
        Grid_routing grid_routing;
        
        grid_routing.run();

    return 0;
}