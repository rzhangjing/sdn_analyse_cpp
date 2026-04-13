#pragma once

#include <QMap>
#include <QVector>
#include <QString>
#include <limits>
#include "graph.h"

class ShortestPathResult {
public:
    ShortestPathResult() : m_valid(false), m_source(0) {}

    ShortestPathResult(quint32 source,
                       const QMap<quint32, double> &distances,
                       const QMap<quint32, quint32> &predecessors,
                       const QMap<quint32, bool> &hasPredecessor)
        : m_valid(true)
        , m_source(source)
        , m_distances(distances)
        , m_predecessors(predecessors)
        , m_hasPredecessor(hasPredecessor)
    {}

    bool isValid() const { return m_valid; }
    quint32 source() const { return m_source; }
    const QMap<quint32, double> &allDistances() const { return m_distances; }

    // 返回到达目标节点的距离，不可达时返回 infinity
    double distance(quint32 destination) const;

    // 重建从 source 到 destination 的路径，不可达时返回空 QVector
    QVector<quint32> path(quint32 destination) const;

private:
    bool m_valid;
    quint32 m_source;
    QMap<quint32, double> m_distances;
    // predecessors[node] = 前驱节点（仅当 hasPredecessor[node] 为 true 时有效）
    QMap<quint32, quint32> m_predecessors;
    QMap<quint32, bool> m_hasPredecessor;
};

// 对给定图从 source 出发执行 Dijkstra 最短路径算法
// 成功返回 true，result 填充结果；失败返回 false，errorMsg 填充原因
bool dijkstra(const Graph &graph, quint32 source,
              ShortestPathResult &result, QString *errorMsg = nullptr);
