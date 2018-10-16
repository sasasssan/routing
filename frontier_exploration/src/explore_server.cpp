#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/MapMetaData.h>

#include <geometry_msgs/PolygonStamped.h>

#include <std_srvs/Empty.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/BlacklistPoint.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_exploration/geometry_tools.h>
#include <frontier_exploration/costmap_tools.h>

#include "iiam/iiam.h"

typedef struct{
    std::vector<unsigned int> cell;    
    geometry_msgs::Point centroid;
} Object;

namespace frontier_exploration{
    
    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

/**
 * @フロンティア探査のためのサーバであり、
 * 構造化フロンティア探索タスクに関連する状態機械を実行し、move_baseを介してロボットの動きを管理する。
 */
class FrontierExplorationServer
{

public:

    /**
     * @サーバのコンストラクタです。探索のためにこのノードのActionServerを設定し、ロボットの移動のためにActionClientをmove_baseに設定します。
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false),
        move_client_("move_base",true),
        retry_(5),
        a(0.10)
    {
        private_nh_.param<double>("frequency", frequency_, 0.0);
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);

        explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));
        /*mapデータの読み取り処理*/
        carrent_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("carrent_costmap", tf_listener_));
        carrent_costmap = carrent_costmap_ros_->getCostmap();
        /*pre_mapのデータ読み込み処理*/
        pre_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("pre_costmap", tf_listener_));
        pre_costmap = pre_costmap_ros_->getCostmap();//pre_map...事前地図を示すcostmap_2d
      
        map_sub = nh_.subscribe("map", 1, &FrontierExplorationServer::GetMapCallback, this);

        as_.registerPreemptCallback(boost::bind(&FrontierExplorationServer::preemptCb, this));
        as_.start();
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber map_sub;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;//explore_clientとのactionlibサーバ

    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_,carrent_costmap_ros_, pre_costmap_ros_;//costmap_2d::Costmap2DROSのオブジェクトのポインタを作成する
    costmap_2d::Costmap2D* carrent_costmap;
    costmap_2d::Costmap2D* pre_costmap;
    nav_msgs::OccupancyGrid carrent_octomap;
    double frequency_, goal_aliasing_;
    bool success_, moving_;
    int retry_;

    boost::mutex move_client_lock_;
    frontier_exploration::ExploreTaskGoalConstPtr goal;
    frontier_exploration::ExploreTaskResult result_;
    frontier_exploration::ExploreTaskFeedback feedback_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;//move_baseへのactionlibクライアント
    move_base_msgs::MoveBaseGoal move_client_goal_;

    iiam a;

    /**
     * @actionserverのコールバックを実行し、新しい目標(goal)を受け入れた後に実行します
     * @param goal ActionGoalは、探索する領域の境界とその領域の有効な中心点を含んでいます
     */
    void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &explore_task)
    {
        goal = explore_task;

        success_ = false;
        moving_ = false;

        explore_costmap_ros_->resetLayers();

        //コストマップのサービスクライアントを作る
        ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        ros::ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");

        //move_baseとコストマップの接続待機
        if(!move_client_.waitForServer() || !updateBoundaryPolygon.waitForExistence() || !getNextFrontier.waitForExistence()){
            as_.setAborted();
            return;
        }

        //コストマップに探査境界をセット
        if(ros::ok() && as_.isActive()){
            frontier_exploration::UpdateBoundaryPolygon srv;
            srv.request.explore_boundary = goal->explore_boundary;
            if(updateBoundaryPolygon.call(srv)){
                ROS_INFO("Region boundary set");
            }else{
                ROS_ERROR("Failed to set region boundary");
                as_.setAborted();
                return;
            }
        }

        std::vector<geometry_msgs::PoseStamped> observation_points;
        geometry_msgs::PoseStamped tmp;
        double min_distance, distance;
        bool finish_ = false;

        //領域内を全て探索する
        ros::Rate rate(frequency_);
        while(ros::ok() && as_.isActive()){

            frontier_exploration::GetNextFrontier srv;//bounded_explore_layerへのサービス::req：自己位置,res：次の目的地

            geometry_msgs::PoseStamped goal_pose;//次の目的地

            //global_frameでの現在の自己位置を取得する
            tf::Stamped<tf::Pose> robot_pose;
            explore_costmap_ros_->getRobotPose(robot_pose);

            //サービスに自己位置を提供する
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);

            //ロボットの自己位置から、探査境界内にあるかどうかを評価する
            geometry_msgs::PoseStamped eval_pose = srv.request.start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){//自己位置と探索境界のフレームを一致させる
                tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
            }

            //ロボットが探査境界内になく、探索領域の中心に戻る必要があるかどうかを確認する
            if(goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                
                //ロボットが少なくとも1つのフロンティアを調査したかどうかを確認し、デバッグメッセージを警告に促す
                if(success_){
                    ROS_WARN("Robot left exploration boundary, returning to center");
                }else{
                    ROS_DEBUG("Robot not initially in exploration boundary, traveling to center");
                }
                //探索センターの枠内に現在のロボット位置を取得する
                geometry_msgs::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;
                if(eval_point.header.frame_id != goal->explore_center.header.frame_id){
                    geometry_msgs::PointStamped temp = eval_point;
                    tf_listener_.transformPoint(goal->explore_center.header.frame_id, temp, eval_point);
                }

                //目的地を探査センターに設定する
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(eval_point.point, goal->explore_center.point) );

            }else if(!finish_ && getNextFrontier.call(srv)){ //境界にある場合は、次のフロンティアを検索してください

                ROS_DEBUG("Found frontier to explore");
                success_ = true;
                goal_pose = feedback_.next_frontier = srv.response.next_frontier;
                retry_ = 5;

            }else if(!observation_points.empty()){

                printf("remained observation points = %d\n", (int)observation_points.size());
                goal_pose = observation_points.back();
                observation_points.pop_back();
                min_distance = pointsDistance(eval_pose.pose.position, goal_pose.pose.position);
                for(int itr = 0; itr < observation_points.size(); itr++){
                    distance = pointsDistance(eval_pose.pose.position, observation_points[itr].pose.position);
                    if(distance < min_distance){
                        tmp = observation_points[itr];
                        observation_points[itr] = goal_pose;
                        goal_pose = tmp;
                        min_distance = distance;
                    }
                }
                printf("goto = (%.1f, %.1f)\n", goal_pose.pose.position.x, goal_pose.pose.position.y);

                observation_points.clear();//一番近い観測点を目標地点に登録して、他をすべて消去する

            }else { //フロンティアが見つからなければ、検索が成功したかどうかをチェックする
                ROS_DEBUG("Couldn't find a frontier");

                if(!finish_){//始めてこの条件に入った場合はここに入る
                    finish_ = true;
                    if(getObservationPoints(&observation_points)) continue;//観測点を取得できればループの始めに戻る
                    
                }

                //成功
                if(retry_ == 0 && success_){
                    ROS_INFO("Finished exploring room");
                    result_.base_position = feedback_.base_position;
                    as_.setSucceeded(result_);
                    boost::unique_lock<boost::mutex> lock(move_client_lock_);
                    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                    return;

                }else if(retry_ == 0 || !ros::ok()){ //失敗
                    ROS_ERROR("Failed exploration");
                    result_.base_position = feedback_.base_position;
                    as_.setSucceeded(result_);
                    return;
                }

                ROS_DEBUG("Retrying...");
                retry_--;
                //ロボットを動かすことなく、フロンティアを再び見つけよう
                continue;
            }
            //上記の条件がこのループステップをエスケープしない場合、searchには有効なgoal_poseがあります

            //新しいゴールが古いゴールに近いかどうかを確認してください。再送信する必要はありません
            if(!moving_ || !pointsNearby(move_client_goal_.target_pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5)){
                ROS_DEBUG("New exploration goal");
                move_client_goal_.target_pose = goal_pose;
                boost::unique_lock<boost::mutex> lock(move_client_lock_);
                if(as_.isActive()){
                    move_client_.sendGoal(move_client_goal_, boost::bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
                    moving_ = true;
                }
                lock.unlock();
            }

            //継続的な目標の更新が有効になっているかどうか確認
            if(frequency_ > 0 && !finish_){
                //指定した頻度でスリープ状態にしてから検索を続行する
                rate.sleep();
            }else{
                //移動が完了するのを待ってから続行する
                while(ros::ok() && as_.isActive() && moving_){
                    ros::WallDuration(0.1).sleep();
                }
            }
        }

        //目標はこの時点で決してアクティブではいけません
        ROS_ASSERT(!as_.isActive());

    }


    /**
     * @サーバーのコールバックをプリエンプトし、現在の実行中の目標と関連するすべての移動アクションを取り消します
     */
    void preemptCb(){

        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        ROS_WARN("Current exploration task cancelled");

        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    /**
     * @move_baseクライアントのためのフィードバックコールバック、探査サーバのフィードバックとして再公開
     * @param feedback move_baseクライアントからのフィードバック
     */
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    /**
     * @move_baseクライアントのコールバックを完了し、エラーをチェックし、必要に応じて探索タスクを中止します
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move. Blacklisting point.");
            moving_ = false;
            
            //ブラックリストサービスを見つける
            ros::ServiceClient blacklistPointService = private_nh_.serviceClient<BlacklistPoint>("explore_costmap/explore_boundary/blacklist_point");
            //サービス要求を作成する
            BlacklistPoint srv;
            srv.request.point = feedback_.next_frontier.pose.position;
            
            // Call the service
            if (!blacklistPointService.call(srv)) {
                ROS_ERROR("Failed to blacklist point.");
            }
        }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            moving_ = false;
            
            //明確なブラックリストサービスを見つける
            ros::ServiceClient clearBlacklistService = private_nh_.serviceClient<std_srvs::Empty>("explore_costmap/explore_boundary/clear_blacklist");
            
            //引数なし
            std_srvs::Empty sr;
            
            // Call the service
            if (!clearBlacklistService.call(sr)) {
                ROS_ERROR("Failed to clear blacklist.");
            }

            //ブラックリストサービスを見つける
            ros::ServiceClient blacklistPointService = private_nh_.serviceClient<BlacklistPoint>("explore_costmap/explore_boundary/blacklist_point");
            //サービス要求を作成する
            BlacklistPoint srv;
            srv.request.point = feedback_.next_frontier.pose.position;
            
            // Call the service
            if (!blacklistPointService.call(srv)) {
                ROS_ERROR("Failed to blacklist point.");
            }
        }

    }

    /**
     * @mapの更新毎にデータコピー
     * @param octomap 更新されたoctomap
     */    
    void GetMapCallback(const nav_msgs::OccupancyGrid &octomap){
        if(goal == NULL)return;
        carrent_octomap = octomap;
    }

    /**
     * @観測点を取得する
     * @param observation_points 観測点を格納する配列
     * @観測点が一つ以上存在する場合はTrueを返す
     */
    bool getObservationPoints(std::vector<geometry_msgs::PoseStamped> *observation_points){
        std::vector<unsigned int> new_object_cell;
        if(!mapMatching(&new_object_cell)) return false;

        std::vector<Object> new_object;
        if(!clusteringObjects(&new_object, new_object_cell)) return false; 

        geometry_msgs::PoseStamped buf;
        geometry_msgs::Polygon ob;
        nav_msgs::OccupancyGrid map;
        int i = 0;
        BOOST_FOREACH(Object object, new_object){
            ob.points.clear();
            map = carrent_octomap;
            BOOST_FOREACH(unsigned int cell, object.cell){
                map.data[cell] = 50;
            }
            a.MakeIIAM(map, &ob);
            buf.header = goal->explore_center.header;
            printf("Object%d: cell:%d centroid(%.1f,%.1f)\n", i++, (int)object.cell.size(), object.centroid.x, object.centroid.y);
            BOOST_FOREACH(geometry_msgs::Point32 point, ob.points){
                printf("(%.1f,%.1f) ", point.x, point.y);
                buf.pose.position = costmap_2d::toPoint(point);
                buf.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(buf.pose.position, object.centroid) );
                observation_points->push_back(buf);
            }
            printf("\n");
        }
        printf("observation points = %d\n", (int)observation_points->size());
        if(observation_points->empty()) return false;
        else return true;
    }

    /**
     * @地図のマッチング
     * @param new_object_cell 新規オブジェクトセルを格納する配列
     * @新規オブジェクトセルが一つ以上存在するTrueを返す
     */
    bool mapMatching(std::vector<unsigned int> *new_object_cell){
        unsigned char* carrent_map;
        unsigned int mx, my, index, max_index;
        geometry_msgs::Point buf;

        carrent_map = carrent_costmap->getCharMap();
        max_index = carrent_costmap->getSizeInCellsX() * carrent_costmap->getSizeInCellsY();

        
        for(index = 0; index < max_index; index++){
            if(carrent_map[index] != LETHAL_OBSTACLE) continue;
            carrent_costmap->indexToCells(index, mx, my);
            carrent_costmap->mapToWorld(mx, my, buf.x, buf.y);
            if(!pointInPolygon(buf,goal->explore_boundary.polygon)) continue;//探査境界内のみを参照

            if(!(pre_costmap->worldToMap(buf.x, buf.y, mx, my))) continue;
            if(pre_costmap->getCost(mx, my) != FREE_SPACE) continue;
            //周囲８グリッドは誤差と見なす
            if(pre_costmap->getCost(mx - 1, my) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx + 1, my) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx, my - 1) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx, my + 1) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx - 1, my - 1) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx + 1, my - 1) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx - 1, my + 1) != FREE_SPACE) continue;
            if(pre_costmap->getCost(mx + 1, my - 1) != FREE_SPACE) continue;
            new_object_cell->push_back(index);
        }

        printf("new objekt cells = %d\n", (int)new_object_cell->size());
        if(new_object_cell->empty()) return false;
        return true;
    }

    /**
     * @オブジェクトセルのクラスタリング
     * @param new_object 新規オブジェクトを格納する配列
     * @param new_object_cell 新規オブジェクトセルが格納されている配列
     * @新規オブジェクトが一つ以上存在する場合Trueを返す
     */
    bool clusteringObjects(std::vector<Object> *new_object, std::vector<unsigned int> &new_object_cell){
        unsigned int idx, mx, my;
        bool flag;
        Object buf;
        std::vector<Object> object;
        std::vector<unsigned int> bfs;
        geometry_msgs::Point point;

        while(!new_object_cell.empty()){
            buf.cell.clear();
            bfs.clear();
            bfs.push_back(new_object_cell.back());
            new_object_cell.pop_back();

            while(!bfs.empty()){
                idx = bfs.back();
                bfs.pop_back();
                buf.cell.push_back(idx);
                BOOST_FOREACH(unsigned int nbr, nhood8(idx, *carrent_costmap)){
                    for(int i = 0; i < new_object_cell.size(); i++){
                        if(nbr != new_object_cell[i]) continue;
                        bfs.push_back(new_object_cell[i]);
                        new_object_cell[i] = new_object_cell.back();
                        new_object_cell.pop_back();
                        i--;
                    }
                }
            }

            object.push_back(buf);
        }


        BOOST_FOREACH(buf, object){
            if(buf.cell.size() <= 2) continue;
            buf.centroid.x = 0.0;
            buf.centroid.y = 0.0;
            BOOST_FOREACH(unsigned int cell, buf.cell){
                carrent_costmap->indexToCells(cell, mx, my);
                carrent_costmap->mapToWorld(mx, my, point.x, point.y);
                buf.centroid.x += point.x;
                buf.centroid.y += point.y;
            }
            buf.centroid.x /= buf.cell.size();
            buf.centroid.y /= buf.cell.size();
            new_object->push_back(buf);
        }

        printf("new objects = %d\n", (int)new_object->size());
        if(new_object->empty()) return false;
        else return true;
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
