#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/costmap_tools.h>
#include <frontier_exploration/Frontier.h>

namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap, int min_frontier_size, std::string &travel_point) :
    costmap_(costmap), min_frontier_size_(min_frontier_size), travel_point_(travel_point) { }

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position){

    std::list<Frontier> frontier_list;

    //検索前にロボットがコストマップの範囲内にあることを確認する
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //検索期間中、地図が一貫してロックされていることを確認する
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //訪問先セルおよびフロンティアセルを追跡するフラグ配列を初期化する
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //幅広い最初の検索を初期化する
    std::queue<unsigned int> bfs;

    //最も近いクリアセルを見つけて検索を開始する
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){//nearestCell()→cosmap_tools.h
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //4つの接続された近所を反復する
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_)){
            //追加された未使用のセルをすべてキューに追加し、非フリーセルで初期化された場合は降順検索を使用する
            if(map_[nbr] <= map_[idx] && !visited_flag[nbr]){
                visited_flag[nbr] = true;
                bfs.push(nbr);
                //セルが新しいフロンティアセルであるかどうかをチェックする（未訪問、NO_INFORMATION、フリー近隣）
            }else if(isNewFrontierCell(nbr, frontier_flag)){
                frontier_flag[nbr] = true;
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if(new_frontier.size > min_frontier_size_){
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    return frontier_list;

}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag){

    //フロンティア構造を初期化する
    Frontier output;
    geometry_msgs::Point centroid, middle;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //フロンティアの初期接点を記録する
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    costmap_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y);

    //最初のグリッドセルをキューにプッシュする
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //世界座標系におけるキャッシュの参照位置
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //フロンティアに隣接する8つのセルにセルを追加してみてください
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //隣人が潜在的なフロンティアセルであるかどうかをチェックする
            if(isNewFrontierCell(nbr,frontier_flag)){

                //セルをフロンティアとしてマークする
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //フロンティアサイズを更新する
                output.size++;

                //フロンティアの重心を更新する
                centroid.x += wx;
                centroid.y += wy;

                //最寄りのグリッドセルからロボットまでのフロンティアの距離を決定する
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    middle.x = wx;
                    middle.y = wy;
                }

                //幅広い最初の検索のためにキューに追加する
                bfs.push(nbr);
            }
        }
    }

    //フロンティア重心を平均化する
    centroid.x /= output.size;
    centroid.y /= output.size;

    if(travel_point_ == "closest"){
        // 既に設定されたポイント
    }else if(travel_point_ == "middle"){
        output.travel_point = middle;
    }else if(travel_point_ == "centroid"){
        output.travel_point = centroid;
    }else{
        ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
        // 既に設定されたポイント
    }

    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //セルが未知で、フロンティアとしてマークされていないことを確認する
    if(map_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //フロンティア細胞は、4連結された近隣に少なくとも1つのセルが存在し、遊離している
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, costmap_)){
        if(map_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

}
