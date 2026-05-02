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

    // 判断图中是否包含指定节点
    bool containsNode(quint32 node) const;

    // 返回图中节点总数
    int nodeCount() const;

    // 返回图中所有节点列表
    QList<quint32> nodes() const;

    // 判断图是否为空（无节点）
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
    // 返回值：QHash<键, 值>，键为 packNodePair(source, target)，值为 source -> target 的最短路径节点序列
    const QHash<quint64, QVector<quint32>> &allPaths() const;

    // 将 (source, target) 打包为 quint64：低32位 source，高32位 target
    static inline quint64 packNodePair(quint32 source, quint32 target) 
    {
        return (static_cast<quint64>(target) << 32) | static_cast<quint64>(source);
    }

    // 将 quint64 解包为 (source, target)：低32位 source，高32位 target
    static inline QPair<quint32, quint32> unpackNodePair(quint64 packed) 
    {
        return qMakePair(static_cast<quint32>(packed), static_cast<quint32>(packed >> 32));
    }

private:
    // 邻接表：节点 -> {邻居 -> (权重, 带宽)}
    QMap<quint32, QMap<quint32, std::tuple<double, double>>> m_adjacencyList;

    // Floyd-Warshall 缓存结果
    // 标记 Floyd-Warshall 计算结果是否有效
    bool m_floydValid = false;
    // 所有节点对的最短距离，键为 packNodePair(source, target)
    QHash<quint64, double> m_floydDistances;
    // 所有节点对的最短路径，键为 packNodePair(source, target)
    QHash<quint64, QVector<quint32>> m_floydPaths;
    // Floyd-Warshall 计算时的节点列表
    QVector<quint32> m_floydNodes;
};
