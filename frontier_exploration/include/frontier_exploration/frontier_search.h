#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <frontier_exploration/Frontier.h>
#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration{

/**
 * @入力コストマップに対するフロンティア検索タスクのスレッドセーフ実装。
 */
class FrontierSearch{

public:

    /**
     * @検索タスクのコンストラクタ
     * @param costmap 検索するコストマップデータへの参照。
     * @param min_frontier_size フロンティアを受け入れるための最小サイズ
     * @param travel_point 要求された移動ポイント（最も近い|中央|重心）
     */
    FrontierSearch(costmap_2d::Costmap2D& costmap, int min_frontier_size, std::string &travel_point);

    /**
     * @開始位置から外側へ検索実装を実行します。
     * @param position 検索する最初の位置
     * @return フロンティアのリスト（存在する場合）
     */
    std::list<Frontier> searchFrom(geometry_msgs::Point position);

protected:

    /**
     * @最初のセルから始まり、有効な隣接セルからフロンティアを構築する
     * @param initial_cell フロンティア構築を開始するセルのインデックス
     * @param reference 位置を計算する参照インデックス
     * @param frontier_flag どのセルが既にフロンティアとしてマークされているかを示すフラグ配列
     * @return
     */
    Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);

    /**
     * @候補セルが新しいフロンティアの有効な候補であるかどうかを評価する。
     * @param idx 候補セルの指標
     * @param frontier_flag どのセルが既にフロンティアとしてマークされているかを示すフラグ配列
     * @return
     */
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);

private:

    costmap_2d::Costmap2D& costmap_;
    unsigned char* map_;
    unsigned int size_x_ , size_y_;
    int min_frontier_size_;
    std::string travel_point_;

};

}
#endif
