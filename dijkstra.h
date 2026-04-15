#pragma once

#include <QMap>
#include <QVector>
#include <QString>
#include <limits>
#include "graph.h"

/// 两点直接最短路径计算结果
struct NodeShortestPath {

    quint32 source; // source
    quint32 target; // destination
    double distance; // 最短距离
    QVector<quint32> path; // 经过的路径

    NodeShortestPath() : source(0), target(0), distance(0.0) {}
};

/// 全图的最短路径计算结果
struct GraphShortestPath {
    Graph graph; // 图的数据
    QMap<QPair<quint32, quint32>, NodeShortestPath> pathMap; // 最短路径

    GraphShortestPath() {}
};

class ShortestPathResult {
public:
    ShortestPathResult() : m_valid(false), m_source(0) {}

    ShortestPathResult(quint32 source,
                       const QMap<quint32, double> &distances,
                       const QMap<quint32, quint32> &predecessors,
                       const QMap<quint32, bool> &hasPredecessor,
                       const QMap<quint32, QVector<quint32>> &paths)
        : m_valid(true)
        , m_source(source)
        , m_distances(distances)
        , m_predecessors(predecessors)
        , m_hasPredecessor(hasPredecessor)
        , m_paths(paths)
    {}

    bool isValid() const { return m_valid; }
    quint32 source() const { return m_source; }
    const QMap<quint32, double> &allDistances() const { return m_distances; }

    // 返回到达目标节点的距离，不可达时返回 infinity
    double distance(quint32 destination) const;

    // 获取从 source 到 destination 的路径，不可达时返回空 QVector
    QVector<quint32> path(quint32 destination) const;

    // 返回所有路径
    const QMap<quint32, QVector<quint32>> &allPaths() const { return m_paths; }

private:
    bool m_valid;
    quint32 m_source;
    QMap<quint32, double> m_distances;
    // predecessors[node] = 前驱节点（仅当 hasPredecessor[node] 为 true 时有效）
    QMap<quint32, quint32> m_predecessors;
    QMap<quint32, bool> m_hasPredecessor;
    QMap<quint32, QVector<quint32>> m_paths;
};

// 对给定图从 source 出发执行 Dijkstra 最短路径算法
// 成功返回 true，result 填充结果；失败返回 false，errorMsg 填充原因
bool dijkstra(const Graph &graph, quint32 source,
              ShortestPathResult &result, QString *errorMsg = nullptr);
