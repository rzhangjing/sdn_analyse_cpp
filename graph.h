#pragma once

#include <QMap>
#include <QVector>
#include <QList>
#include <QHash>
#include <QPair>
#include <QString>
#include <tuple>
#include "edge.h"

class Graph 
{
public:
    Graph() = default;

    // 添加节点（如果不存在则插入空邻接表）
    void addNode(quint32 node);

    // 添加有向边，权重不能为负；返回 false 并设置 errorMsg
    bool addEdge(quint32 source, quint32 target, double weight, double bandWidth,
                 QString *errorMsg = nullptr);

    // 从边列表构造图；出错返回 false 并设置 errorMsg
    static bool fromEdges(const QVector<Edge> &edges, Graph &outGraph,
                          QString *errorMsg = nullptr);

    // 获取邻居列表指针，节点不存在返回 nullptr
    // 内层 QMap 键为邻居节点，值为 (weight, bandwidth) tuple
    const QMap<quint32, std::tuple<double, double>> *neighbors(quint32 node) const;

    bool containsNode(quint32 node) const;
    int nodeCount() const;
    QList<quint32> nodes() const;
    bool isEmpty() const;

    // 清空图的所有数据
    void clear();

    // 计算 Floyd-Warshall 全源最短路径并缓存结果
    bool computeFloydWarshall(QString *errorMsg = nullptr);

    // 是否有有效的 Floyd-Warshall 缓存结果
    bool hasFloydWarshallResult() const;

    // 返回 source -> target 的最短距离，无路径时返回 infinity
    double distance(quint32 source, quint32 target) const;

    // 从缓存中获取 source -> target 的最短路径，不可达时返回空 QVector
    QVector<quint32> path(quint32 source, quint32 target) const;

    // 返回所有节点对的 (source, target, distance) 列表
    QVector<std::tuple<quint32, quint32, double>> allDistances() const;

    // 返回图中所有节点列表（Floyd-Warshall 计算时的节点顺序）
    QVector<quint32> nodeList() const;

    // 返回所有预计算的最短路径
    const QHash<QPair<quint32, quint32>, QVector<quint32>> &allPaths() const;

private:
    // 邻接表：节点 -> {邻居 -> (权重, 带宽)}
    QMap<quint32, QMap<quint32, std::tuple<double, double>>> m_adjacencyList;

    // Floyd-Warshall 缓存结果
    bool m_floydValid = false;
    QHash<QPair<quint32, quint32>, double> m_floydDistances;
    QHash<QPair<quint32, quint32>, QVector<quint32>> m_floydPaths;
    QVector<quint32> m_floydNodes;
};
