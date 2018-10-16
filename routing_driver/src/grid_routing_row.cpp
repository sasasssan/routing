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
        std::vector<Waypoint> make_waypoint;//格子点を格納する配列
        geometry_msgs::PoseStamped finish_pose_;//終了姿勢
        std::string save_filename;//waypointを保存するyamlファイル名
        visualization_msgs::MarkerArray waypoints_marker;//rviz上でwaypointを可視化する
        visualization_msgs::Marker clickedpoint_marker;//rviz上で選択領域を可視化する      
        float width, height;//「選択領域を含む長方形」の幅(x軸方向),高さ(y方向)
        float length;//waypointを配置する間隔

        ros::Publisher checkpoint_pub, waypoints_pub, clickedpoints_pub;
        ros::Subscriber map_sub, griddata_sub, clickedpoint_sub;
        int grid_data;//map.pgmのピクセル情報
        int clickedpoint_flag;
        geometry_msgs::Point region[5], square[2];//選択した点を格納する配列、「選択領域を含む長方形」の最小と最大の対角点を格納する配列
        bool map_connect, griddata_flag;
        bool width_, height_;
            

        void save(const std::string filename_){//make_waypoint及びfinish_pose_の点をsave_filenameに保存する
			
			std::ofstream ofs(filename_.c_str(), std::ios::out);
			
			ofs << "waypoints:" << std::endl;
			for(int i=0; i < make_waypoint.size(); i++){
				ofs << "    " << "- position:" << std::endl;
				ofs << "        x: "        << make_waypoint[i].pose.pose.position.x << std::endl;
				ofs << "        y: "        << make_waypoint[i].pose.pose.position.y << std::endl;
				ofs << "        z: "        << make_waypoint[i].pose.pose.position.z << std::endl;
				ofs << "        qx: "       << make_waypoint[i].pose.pose.orientation.x << std::endl;
				ofs << "        qy: "       << make_waypoint[i].pose.pose.orientation.y << std::endl;
				ofs << "        qz: "       << make_waypoint[i].pose.pose.orientation.z << std::endl;
				ofs << "        qw: "       << make_waypoint[i].pose.pose.orientation.w << std::endl;
				ofs << "        flag: "     << make_waypoint[i].flag << std::endl; 
			}
			
			ofs << "finish_pose:"           << std::endl;
			ofs << "    header:"            << std::endl;
			ofs << "        seq: "          << finish_pose_.header.seq << std::endl;
			ofs << "        stamp: "        << finish_pose_.header.stamp << std::endl;
			ofs << "        frame_id: "     << finish_pose_.header.frame_id << std::endl;;
			ofs << "    pose:"              << std::endl;
			ofs << "        position:"      << std::endl;
			ofs << "            x: "        << finish_pose_.pose.position.x << std::endl;
			ofs << "            y: "        << finish_pose_.pose.position.y << std::endl;
			ofs << "            z: "        << finish_pose_.pose.position.z << std::endl;
			ofs << "        orientation:"   << std::endl;
			ofs << "            x: "        << finish_pose_.pose.orientation.x << std::endl;
			ofs << "            y: "        << finish_pose_.pose.orientation.y << std::endl;
			ofs << "            z: "        << finish_pose_.pose.orientation.z << std::endl;
			ofs << "            w: "        << finish_pose_.pose.orientation.w << std::endl;
			
			ofs.close();
			
			printf("Write Success");
			
			double roll, pitch, yaw;
			tf::Quaternion q2(finish_pose_.pose.orientation.x, finish_pose_.pose.orientation.y, 
							  finish_pose_.pose.orientation.z, finish_pose_.pose.orientation.w);
			tf::Matrix3x3 m2(q2);
			m2.getRPY(roll, pitch, yaw);

        }

        bool isInRegion(const geometry_msgs::Pose &point){//対象の格子点が選択領域内に存在するか判定
            int cn = 0;
            float vt;

            for(int i = 0; i < 4; i++){
                if( (region[i].y <= point.position.y) && (region[i+1].y > point.position.y) || ((region[i].y > point.position.y) && (region[i+1].y <= point.position.y)) ){    
                    vt = (point.position.y - region[i].y) / (region[i+1].y - region[i].y);
                    if(point.position.x <= (region[i].x + (vt * (region[i+1].x - region[i].x)))){
                        ++cn;
                    }
                }
            }
            
            return cn % 2 == 1? true : false;

        }
        
        void makeGridRoute(){//選択領域内の格子点の座標をmake_waypointに格納していく
            float row, last_row;
            float line, last_line;
            Waypoint tmp;

            /*「選択領域を含む長方形」の、最大と最小の対角点をsquareに格納する*/
            square[0].x = region[0].x;
            square[0].y = region[0].y;
            square[1].x = region[0].x;
            square[1].y = region[0].y;
            for(int i = 1 ; i < 4; i++){
                if(square[0].x > region[i].x)square[0].x = region[i].x;
                if(square[0].y > region[i].y)square[0].y = region[i].y;
                if(square[1].x < region[i].x)square[1].x = region[i].x;
                if(square[1].y < region[i].y)square[1].y = region[i].y;
            }


            width = square[1].x - square[0].x;
            if(width < 0){
                width = -1 * width;
                width_ = false;
            }
            height = square[1].y - square[0].y;
            if(height < 0){
                height = -1 * height;
                height_ = false;
            }

            tmp.pose.pose.position.x = square[0].x;
            tmp.pose.pose.position.y = square[0].y;
            tmp.pose.pose.position.z = 0.0;
            tmp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);            
            tmp.flag = 0;

            line = 0;
            for(row = 0; row <= width; row += length){
                 tmp.pose.pose.position.x = width_? square[0].x + row: square[0].x - row;
                
                
                if(line < height){
                    for(line = 0; line <= height; line += length){
                        tmp.pose.pose.position.y = height_? square[0].y + line : square[0].y - line;

                        if(!isInRegion(tmp.pose.pose))continue;

                        griddata_flag = false;
                        checkpoint_pub.publish(tmp.pose.pose);
                        while(!griddata_flag){//grid_dataが更新されるまで待機
                            ros::spinOnce();
                        }
                        if(grid_data == 0){
                            make_waypoint.push_back(tmp);
                            last_row = tmp.pose.pose.position.x;
                            last_line = tmp.pose.pose.position.y;
                            printf("put waypoint on 'x = %.1lf, y = %.1lf'\n", last_row, last_line);
                        }
                    }
                } else{
                    for(line -= length; line >= 0; line -= length){
                        tmp.pose.pose.position.y = height_? square[0].y + line : square[0].y - line;

                        if(!isInRegion(tmp.pose.pose))continue;

                        griddata_flag = false;
                        checkpoint_pub.publish(tmp.pose.pose);
                        while(!griddata_flag){//grid_dataが更新されるまで待機
                            ros::spinOnce();
                        }
                        if(grid_data == 0){
                            make_waypoint.push_back(tmp);                         
                            last_row = tmp.pose.pose.position.x;
                            last_line = tmp.pose.pose.position.y;
                            printf("put waypoint on 'x = %.1lf, y = %.1lf'\n", last_row, last_line);
                        }
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
            region[clickedpoint_flag].x = point.point.x;
            region[clickedpoint_flag].y = point.point.y;
            region[clickedpoint_flag].z = 0.0;
            clickedpoint_marker.points.push_back(point.point);
            clickedpoints_pub.publish(clickedpoint_marker);

            clickedpoint_flag++;
            if(clickedpoint_flag == 4){
                region[4] = region[0];
                clickedpoint_marker.points.push_back(region[4]);
                clickedpoints_pub.publish(clickedpoint_marker);
            }
        }

    public:
        Grid_routing(){
            ros::NodeHandle nh;
            checkpoint_pub = nh.advertise<geometry_msgs::Pose>("check_point", 1);
            waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
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
            width_ = true;
            height_ = true;
            
        }

        void run(){
            //mapを受け取る
			while(ros::ok() && !map_connect) {
				ros::spinOnce();
			}

            for(clickedpoint_flag = 0; ros::ok() && clickedpoint_flag < 4; ){
                ros::spinOnce();
            }
            clickedpoint_flag = 0;
            makeGridRoute();
            
            save(save_filename);
            
            makeWaypointMarker();
        }

};

int main(int argc, char** argv){
    
        ros::init(argc, argv, "grid_routing");
        
        Grid_routing grid_routing;
        
        grid_routing.run();

    return 0;
}