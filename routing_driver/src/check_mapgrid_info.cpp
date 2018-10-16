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

#include <string>
#include <math.h>
#include <vector>
#include <fstream>

class Check_mapgrid{
    private:
        ros::Subscriber map_sub, point_sub, pose_sub;
        ros::Publisher data_pub;

        float resolution;//map.pgmの解像度(m/cell)
        int width, height, data_size;//map.pgmの幅(cell), 高さ(cell), dataの大きさ(幅×高さ)
        std_msgs::Int16 grid_info;
        geometry_msgs::Point origin;//grobal座標系におけるmap.pgmの原点(左下)の座標
        std::vector<int> data;

        bool map_connect, flag;
        float global_x, global_y;

        void search_griddata(){
            float x = global_x - origin.x;
            float y = global_y - origin.y;

            int i_x = (int)(x / resolution);
            int i_y = (int)(y / resolution);
            int index = width * i_y + i_x;

            printf("(%.2f,%.2f) = (%d,%d) = map[%d]", global_x, global_y, i_x, i_y, index);
            if(index >= data_size){
                grid_info.data = -1;
            }else {            
                grid_info.data = data[index];
            }
        }

        void GetPointCallback(const geometry_msgs::Pose &pose){//global座標系をgrid座標系に変換して、その点の情報を返す
            global_x = pose.position.x;
            global_y = pose.position.y;

            flag = true;
            return;
        }

        void GetPoseCallback(const geometry_msgs::PointStamped &point){
            global_x = point.point.x;
            global_y = point.point.y;

            flag = true;
            return;
        }

        void GetMapCallback(const nav_msgs::OccupancyGrid &map){

            resolution = map.info.resolution;
            width = map.info.width;
            height = map.info.height;
            data_size = width * height;
            origin.x = map.info.origin.position.x;
            origin.y = map.info.origin.position.y;
            
            int i, tmp;
            for(i = 0; i < data_size; i++){
                tmp = map.data[i];
                data.push_back(tmp);
            }

            map_connect = true;
            return;
        }


    public:
        Check_mapgrid() {
            ros::NodeHandle nh;
            map_sub = nh.subscribe("map", 1, &Check_mapgrid::GetMapCallback, this);
            point_sub = nh.subscribe("check_point", 1, &Check_mapgrid::GetPointCallback, this);
            pose_sub = nh.subscribe("check_pose", 1, &Check_mapgrid::GetPoseCallback, this);
            data_pub = nh.advertise<std_msgs::Int16>("grid_data", 1);
            
            map_connect = false;
            flag = false;
        }

        void run(){
			//mapを受け取る
			while(ros::ok() && !map_connect) {
				ros::spinOnce();
            }
            
            while(ros::ok()){
                ros::spinOnce();
                if(flag){
                    search_griddata();
                    printf(" = %d\n", grid_info.data);
                    data_pub.publish(grid_info);
                    flag = false;
                }
            }


        }

};

int main(int argc, char** argv){
    
        ros::init(argc, argv, "check_mapgrid_info");
        
        Check_mapgrid check_mapgrid;

        check_mapgrid.run();
		
    return 0;
}
