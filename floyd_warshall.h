#pragma once

#include <QMap>
#include <QVector>
#include <QPair>
#include <QString>
#include <tuple>
#include <limits>
#include "graph.h"

class FloydWarshallResult {
public:
    FloydWarshallResult() : m_valid(false) {}

    FloydWarshallResult(const QMap<QPair<quint32, quint32>, double> &distances,
                        const QMap<QPair<quint32, quint32>, quint32> &nextHop,
                        const QVector<quint32> &nodes)
        : m_valid(true)
        , m_distances(distances)
        , m_nextHop(nextHop)
        , m_nodes(nodes)
    {}

    bool isValid() const { return m_valid; }

    // 返回 source -> target 的最短距离，无路径时返回 infinity
    double distance(quint32 source, quint32 target) const;

    // 重建 source -> target 的路径，不可达时返回空 QVector
    QVector<quint32> path(quint32 source, quint32 target) const;

    // 返回所有节点对的 (source, target, distance) 列表
    QVector<std::tuple<quint32, quint32, double>> allDistances() const;

    // 返回图中所有节点列表
    QVector<quint32> nodeList() const;

private:
    bool m_valid;
    QMap<QPair<quint32, quint32>, double> m_distances;
    QMap<QPair<quint32, quint32>, quint32> m_nextHop;
    QVector<quint32> m_nodes;
};

// 对给定图执行 Floyd-Warshall 全源最短路径算法
// 成功返回 true，result 填充结果；失败返回 false，errorMsg 填充原因
bool floydWarshall(const Graph &graph, FloydWarshallResult &result,
                   QString *errorMsg = nullptr);
