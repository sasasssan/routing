#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>

namespace frontier_exploration{

/**
 * @入力セルの4接続された近傍を判定し、マップのエッジをチェックする
 * @param idx 入力セルインデックス
 * @param costmap 地図データへの参照
 * @隣接セルインデックスを返す
 */
std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap){
    //4つの接続された近傍インデックスを取得し、マップのエッジをチェックする
    std::vector<unsigned int> out;

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1){
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }

    if(idx % size_x_ > 0){
        out.push_back(idx - 1);
    }
    if(idx % size_x_ < size_x_ - 1){
        out.push_back(idx + 1);
    }
    if(idx >= size_x_){
        out.push_back(idx - size_x_);
    }
    if(idx < size_x_*(size_y_-1)){
        out.push_back(idx + size_x_);
    }
    return out;

}

/**
 * @入力セルの8つの接続された近傍を決定し、マップのエッジをチェックする
 * @param idx 入力セルインデックス
 * @param costmap 地図データへの参照
 * @隣接セルインデックスを返す
 */
std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap){
    //8つの隣接するインデックスを取得し、マップのエッジをチェックする
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1){
        return out;
    }

    if(idx % size_x_ > 0 && idx >= size_x_){
        out.push_back(idx - 1 - size_x_);
    }
    if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
        out.push_back(idx - 1 + size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
        out.push_back(idx + 1 - size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
        out.push_back(idx + 1 + size_x_);
    }

    return out;

}

/**
 * @指定された値の最も近いセルを見つける
 * @param result 見つかったセルのインデックス
 * @param start 現在位置
 * @param val 検索する指定値
 * @param costmap 地図データへの参照
 * @要求された値を持つセルが見つかった場合はTrueを返します
 */
bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap){

    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

    if(start >= size_x * size_y){
        return false;
    }

    //最初の検索の幅を初期化する
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    //初期セルをpush
    bfs.push(start);
    visited_flag[start] = true;

    //隣接セル一致値を検索する
    while(!bfs.empty()){
        //bgsから一つ値をpopする
        unsigned int idx = bfs.front();
        bfs.pop();

        //val値のセルが見つかった場合return
        if(map[idx] == val){
            result = idx;
            return true;
        }

        //隣接していないすべてのセルを反復処理する
        BOOST_FOREACH(unsigned nbr, nhood8(idx, costmap)){//idxの周囲8セルをpush
            if(!visited_flag[nbr]){
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}

}
#endif
