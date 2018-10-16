#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <frontier_exploration/geometry_tools.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <costmap_2d/footprint.h>
#include <tf/transform_listener.h>

#include <ros/wall_timer.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <time.h>
#include <boost/foreach.hpp>


namespace frontier_exploration{

/**
 * @brief rvizからコントロールポイントを受け取り、フロンティア探査用の境界ポリゴンを作成するFrontierExplorationServer用のクライアント
 */
class FrontierExplorationClient{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber point_sub;
    ros::Publisher point_viz_pub_;
    ros::WallTimer point_viz_timer_;
    ros::ServiceClient map_reset;
    geometry_msgs::Point start_point;
    geometry_msgs::PolygonStamped input_;
    std::vector<frontier_exploration::ExploreTaskGoal> map_section;

    bool waiting_for_center_;
    double sensor_range;
    clock_t start, end;

    /**
     * @境界ポリゴンの点を視覚化するためのマーカーを公開する。
     */
    void vizPubCb(){

        visualization_msgs::Marker points, line_strip;

        points.header = line_strip.header = input_.header;
        points.ns = line_strip.ns = "explore_points";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        if(!input_.polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = 0.05;

            BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
                line_strip.points.push_back(costmap_2d::toPoint(point));
                points.points.push_back(costmap_2d::toPoint(point));
            }

            if(waiting_for_center_){
                line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
                points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            }else{
                points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
            }
        }else{
            points.action = line_strip.action = visualization_msgs::Marker::DELETE;
        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);

    }

    /**
     * @rviz guiから受け取ったポイントから境界ポリゴンを構築します。
     * @param point rvizから受け取った座標
     */
    void pointCb(const geometry_msgs::PointStampedConstPtr& point){

        double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();
                
        if(waiting_for_center_){//境界が閉じていたらここに入る

            if(!pointInPolygon(point->point,input_.polygon)){//境界内に最後のpointを置いていないとき
                ROS_ERROR("Center not inside polygon, restarting");
                waiting_for_center_ = false;
                input_.polygon.points.clear();
            }else{
                //正常に境界と目標pointが配置されると、サーバに目標地点として送信する
                start = clock();
                printf("start_time: %.1lf\n", (double)start);
                actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
                exploreClient.waitForServer();

                frontier_exploration::ExploreTaskGoal goal;
                if(putWaypoints(point)){//巡回点(waypoint)を各区画の中心に設置します
                    while(!map_section.empty() && ros::ok()){
                        printf("reamined sections = %d\n", (int)map_section.size());
                        ROS_INFO("Sending goal");
                        goal = get_nextSection(start_point);
                        exploreClient.sendGoalAndWait(goal, ros::Duration(0,0), ros::Duration(0,0));//動作を完了するまで待機
                        start_point = exploreClient.getResult()->base_position.pose.position;
                        std_srvs::Empty empty;
                        map_reset.call(empty);
                    }
                }else {//区画設定に失敗は場合はこっち
                    goal.explore_center = *point;
                    goal.explore_boundary = input_;
                    exploreClient.sendGoal(goal);
                }
            }
            end = clock();
            printf("Completed exploring all sections\n");
            printf("end_time: %.1lf\n", (double)end);
            printf("Time = %lf[ms]\n", (double)(end - start)/10000);
            waiting_for_center_ = false;
            input_.polygon.points.clear();

        }else if(input_.polygon.points.empty()){
            //最初の制御点なので、境界ポリゴンのヘッダを初期化する

            input_.header = point->header;
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));

        }else if(input_.header.frame_id != point->header.frame_id){
            ROS_ERROR("Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();

        }else if(input_.polygon.points.size() > 1 && pointsNearby(input_.polygon.points.front(), point->point, average_distance*0.1)){
            //今配置したpointが始めのpointに近い場合、境界が閉じられたかを確認する

            if(input_.polygon.points.size() < 3){
                ROS_ERROR("Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }else{
                waiting_for_center_ = true;
                ROS_INFO("Please select an initial point for exploration inside the polygon");
            }

        }else{

            //それ以外の場合は、境界ポリゴン内の通常点でなければなりません
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
            input_.header.stamp = ros::Time::now();
        }

    }

    /**
     * 選択された領域を格子状に区切り、各区画の頂点と中心点を記憶する関数
     * 分割を行わない場合はfalseを返すようにすること
     */
    bool putWaypoints(const geometry_msgs::PointStampedConstPtr& point){
        geometry_msgs::Point32 tmp, tmp_[4], square[2];
        frontier_exploration::ExploreTaskGoal buf;
        buf.explore_center.header = point->header;
        buf.explore_boundary.header = input_.header;

        square[0] = square[1] = input_.polygon.points[0];
        BOOST_FOREACH(geometry_msgs::Point32 vertex, input_.polygon.points){
                if(square[0].x > vertex.x) square[0].x = vertex.x; if(square[0].y > vertex.y) square[0].y = vertex.y;
                if(square[1].x < vertex.x) square[1].x = vertex.x; if(square[1].y < vertex.y) square[1].y = vertex.y;
        }

        int width = fabs(square[1].x - square[0].x) /sensor_range , height = fabs(square[1].y - square[0].y) /sensor_range;
        
        for(int row = 0; ; row++){
            tmp.x = square[0].x + row * sensor_range + sensor_range/2;
                
            for(int line = 0; ; line++){
                tmp.y = square[0].y + line * sensor_range + sensor_range/2;

                tmp_[0].x = tmp.x - sensor_range/2; tmp_[0].y = tmp.y - sensor_range/2;
                tmp_[1].x = tmp.x + sensor_range/2; tmp_[1].y = tmp.y - sensor_range/2;
                tmp_[2].x = tmp.x + sensor_range/2; tmp_[2].y = tmp.y + sensor_range/2;
                tmp_[3].x = tmp.x - sensor_range/2; tmp_[3].y = tmp.y + sensor_range/2;

                if(pointInPolygon(tmp_[0],input_.polygon) || pointInPolygon(tmp_[1],input_.polygon) ||
                     pointInPolygon(tmp_[2],input_.polygon) || pointInPolygon(tmp_[3],input_.polygon)){

                    buf.explore_center.point.x = tmp.x; buf.explore_center.point.y = tmp.y;

                    buf.explore_boundary.polygon.points.clear();
                    for(int i = 0; i < 4; i++){
                        buf.explore_boundary.polygon.points.push_back(tmp_[i]);
                    }
                    map_section.push_back(buf);
                }
                if(line > height) break;
            }
            if(row > width) break;
        }

        if(map_section.empty()) return false;
        //return true;
        return false;
    }

    /**
     * 次の巡回点を決定する関数
     * ロボットの現在位置に一番近い区画を選ぶ
     */
    frontier_exploration::ExploreTaskGoal get_nextSection(const geometry_msgs::Point &point){
        frontier_exploration::ExploreTaskGoal next_section = map_section.back(), tmp;
        map_section.pop_back();
        double min_distance = pointsDistance(point, next_section.explore_center.point), distance;
        
        for(int itr = 0; itr < map_section.size(); itr++){
            distance = pointsDistance(point, map_section[itr].explore_center.point);
            if(distance < min_distance){
                tmp = map_section[itr];
                map_section[itr] = next_section;
                next_section = tmp;
                min_distance = distance;
            }
        }

        return next_section;
    }

public:

    /**
     * @クライアントのコンストラクタです。
     */
    FrontierExplorationClient() :
        nh_(),
        private_nh_("~"),
        waiting_for_center_(false)
    {
        input_.header.frame_id = "map";
        point_sub = nh_.subscribe("/clicked_point",10,&FrontierExplorationClient::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
        map_reset = nh_.serviceClient<std_srvs::Empty>("reset_map");
        point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&FrontierExplorationClient::vizPubCb, this));
        private_nh_.param<double>("sensor_range", sensor_range, 10.0);
        private_nh_.param<double>("initial_pose_x", start_point.x, 0.0);
        private_nh_.param<double>("initial_pose_y", start_point.y, 0.0);        
        ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");
    }    

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client");

    frontier_exploration::FrontierExplorationClient client;
    ros::spin();
    return 0;
}
