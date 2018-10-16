#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/BlacklistPoint.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        ros::NodeHandle nh_("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        blacklist_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("blacklist", 5);
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");
        nh_.param<int>("min_frontier_size", min_frontier_size_, 1);

        polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);
        blacklistPointService_ = nh_.advertiseService("blacklist_point", &BoundedExploreLayer::blacklistPointService, this);
        clearBlacklistService_ = nh_.advertiseService("clear_blacklist", &BoundedExploreLayer::clearBlacklistService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        blacklist_radius_ = 0.5;
    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier);
    }

    bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

        //境界線が表示されるまでコストマップを待つ
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }

        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //使用可能な変換がない場合はエラーが発生します。
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                return false;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        //frontier_searchオブジェクトを生成
        /*新規オブジェクト発見時フラグを立ててobject_searchモードに移行する*/
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_size_, frontier_travel_point_);
        //frontierのリストを取得する
        std::list<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        //選択したフロンティアのプレースホルダを作成する
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //視覚化のためのポイントクラウド
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            //フロンティアを視覚化ポテンシャルクラウドにロードする
            frontier_point_viz.x = frontier.travel_point.x;
            frontier_point_viz.y = frontier.travel_point.y;
            frontier_cloud_viz.push_back(frontier_point_viz);

            //このフロンティアがロボットに最も近いかどうかをチェックする
            if (frontier.min_distance < selected.min_distance && !anyPointsNearby(frontier.travel_point, blacklist_, blacklist_radius_) && pointInPolygon(frontier.travel_point, polygon_)){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }

        if (std::isinf(selected.min_distance)) {
            ROS_DEBUG("No valid (non-blacklisted) frontiers found, exploration complete");
            return false;
        }

        //選択された色のフロンティア
        frontier_cloud_viz[max].intensity = 100;

        //視覚化ポイントクラウドを公開する
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //目標を次のフロンティアに設定する
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        //
        next_frontier.pose.position = selected.travel_point;
        next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        return true;

    }

    bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void BoundedExploreLayer::reset(){

        //costmap_のchar配列をデフォルト値にリセットする
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
        
    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //既存の境界があればそれをクリアする
        polygon_.points.clear();

        //ポリゴンとコストマップの間で変換が利用できない場合はエラー
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return false;
        }

        //境界ポリゴンのすべての点をcostmapフレームに変換する
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //空の境界が提供された場合、マップ全体に設定
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //ポリゴンの最小/最大点を見つけることによって地図のサイズと原点を見つける
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //コストマップをポリゴン境界にリサイズし、解像度を変更しない
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //レイヤーが有効になっており、境界線が設定されているかどうかを確認
        if (!enabled_ || !configured_){ return; }

        /*全体のコストマップを更新する
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();*/

    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //レイヤーが有効になっており、境界線が設定されているかどうかを確認
        if (!enabled_ || !configured_){ return; }

        //ポリゴンの各点の間に線を引く
        MarkCell marker(costmap_, LETHAL_OBSTACLE);//costmap_:このオブジェクトがもっているローカルな地図、master_grid = exploration_server/exploration_costmap

        //円形イテレータ
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);//選択領域の境界線にLATHAL_OBSTACLEを設置する
        }
        //内部コストマップからマスターグリッドを更新する
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //ローカルのコストマップセルが致命的/高い値であり、マスタグリッド内の致命的な障害を上書きしていない場合にのみ、マスタグリッドを更新する
                /*新規オブジェクトの検出*/
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }

    bool BoundedExploreLayer::blacklistPointService(frontier_exploration::BlacklistPoint::Request &req, frontier_exploration::BlacklistPoint::Response &res) {
        //ブラックリストにポイントを追加する
        blacklist_.push_back(req.point);
        ROS_WARN("Blacklist point added %f, %f", req.point.x, req.point.y);

        //ブラックリストトピックのポイントを表示する
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        marker.id = blacklist_.size();
        marker.action = visualization_msgs::Marker::ADD;

        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time::now();

        marker.pose.position = req.point;
        marker.pose.orientation.w = 1.0;

        //スケールは形状の直径です
        marker.scale.x = 2 * blacklist_radius_;
        marker.scale.y = 2 * blacklist_radius_;
        //サークル
        marker.scale.z = 0.05;

        marker.color.r = 1.0;
        marker.color.a = 0.6;

        blacklist_marker_pub_.publish(marker);

        // All is good :)
        return true;
    }

    bool BoundedExploreLayer::clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        //リストをクリアする
        blacklist_.clear();
        ROS_WARN("Blacklist cleared");

        //すべてのマーカーを視覚化から削除する
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        //機能は実装されていますが、定数はROSインディゴには存在しません。 定義はこのcpp内で定義されています
        marker.action = DELETEALL;

        // All is good :)
        return true;
    }
}
