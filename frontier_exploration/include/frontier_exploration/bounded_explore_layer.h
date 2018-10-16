#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Polygon.h>
#include <std_srvs/Empty.h>
#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/BlacklistPoint.h>

namespace frontier_exploration
{

/**
 * @フロンティア探査タスクの状態を保持するcostmap_2dレイヤープラグイン。
 * 境界ポリゴンを管理し、全体の探索コストマップにポリゴンを重ね合わせ、探索する次の境界線を見つけるためにコストマップを処理します。
 */
class BoundedExploreLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    BoundedExploreLayer();
    ~BoundedExploreLayer();

    /**
     * @デフォルト値を読み込み、探索コストマップを初期化します。
     */
    virtual void onInitialize();

    /**
     * @更新するコストマップウィンドウの境界を計算する
     */
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x,
                              double* polygon_max_y);

    /**
     * @要求されたコストマップウィンドウを更新する
     */
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @親コストマップの次元と原点を一致させる
     */
    virtual void matchSize();

    /**
     * @探査の進捗状況をリセットする
     */
    virtual void reset();
    
    /**
     * @visualization_msgs::Marker::DELETEALLの定数定義。ROS Indigoでは定義されていませんが、機能は実装されています
     */
    static const int DELETEALL = 3;

protected:

    /**
     * @updateBoundaryPolygonのROSサービスラッパー
     * @param req Service request
     * @param res Service response
     * @return サービス成功の場合はtrue、そうでない場合はfalse
     */
    bool updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res);

    /**
     * @各更新で地図上に描くためにポリゴンの境界線をロードする
     * @param polygon_stamped ポリゴン境界
     * @return ポリゴンが正常に読み込まれた場合はtrue、そうでない場合はfalse
     */
    bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);

    /**
     * @getNextFrontierのROSサービスラッパー
     * @param req Service request
     * @param res Service response
     * @return サービス成功の場合はtrue、そうでない場合はfalse
     */
    bool getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res);

    /**
     * @次の到達可能なフロンティアを探索するためにコストマップを検索する
     * @param start_pose 検索を開始する姿勢
     * @param next_frontier 発見されたフロンティアの姿勢
     * @return 到達可能なフロンティアが見つかった場合はtrue、そうでない場合はfalse
     */
    bool getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);

    /**
     * @フロンティアブラックリストにポイントを追加するためのROS Serviceラッパー
     * @param req Service request
     * @param res Service response
     * @return Always true
     */
    bool blacklistPointService(frontier_exploration::BlacklistPoint::Request &req, frontier_exploration::BlacklistPoint::Response &res);

    /**
     * @フロンティアブラックリストをクリアするためのROS Serviceラッパー 
     * @param req Service request
     * @param res Service response
     * @return Always true
     */
    bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

private:

    /**
     * @探索境界データで地図を更新する
     * @param master_grid マスターのコストマップへの参照
     */
    void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::ServiceServer polygonService_;
    ros::ServiceServer frontierService_;
    ros::ServiceServer blacklistPointService_;
    ros::ServiceServer clearBlacklistService_;
    geometry_msgs::Polygon polygon_;
    tf::TransformListener tf_listener_;

    ros::Publisher frontier_cloud_pub;
    ros::Publisher blacklist_marker_pub_;

    bool configured_, marked_;

    std::list<geometry_msgs::Point> blacklist_;

    std::string global_frame_;
    std::string frontier_travel_point_;
    bool resize_to_boundary_;
    int min_frontier_size_;
    double blacklist_radius_;
};

}
#endif
