#pragma once

#include <QMap>
#include <QVector>
#include <QList>
#include <QHash>
#include <QPair>
#include <QString>
#include <tuple>
#include <QSharedPointer>
#include "edge.h"

/// 一条链路的完整信息
///
/// 字段对应算法标识：
/// - `source`       : st_e  — 链路起始节点
/// - `target`       : en_e  — 链路终止节点
/// - `bandwidth`    : Be    — 链路带宽（Gbps）
/// - `bec`          : bec   — Ec 中链路的专用网络带宽（≤ Be）
/// - `delayMs`      :       — 原始延迟值（ms）
/// - `weight`       : We    — 链路成本权重（秒）= delayMs / 1000
/// - `hsEG`         : hs_e_G — 总回避度，对所有 (Ci, Cj) 对的 YG_e_ci_cj 累加和
/// - `weightedCost` : Wep   — 加权成本 = We × hs_e_G
struct NetworkEdge {
    /// st_e：链路起始节点
    quint32 source;
    /// en_e：链路终止节点
    quint32 target;
    /// Be：链路带宽（单位 Gbps）
    double bandwidth;
    /// bec：Ec 中该链路的专用网络带宽（单位 Gbps，必须 ≤ Be）
    double bec;
    /// 原始延迟值（单位 ms）
    double delayMs;
    /// We：链路成本权重（秒）= delayMs / 1000
    double weight;
    /// hs_e_G：链路总回避度
    double hsEG;
    /// Wep：链路加权成本 = We × hs_e_G
    double weightedCost;
    // 最短路径（原始图中的最短路径）
    QVector<quint32> path;

    NetworkEdge()
        : source(0), target(0), bandwidth(0.0), bec(0.0),
        delayMs(0.0), weight(0.0), hsEG(0.0), weightedCost(0.0) {}

    /// 从原始边属性创建链路实例
    NetworkEdge(quint32 src, quint32 tgt, double bw, double delay)
        : source(src), target(tgt), bandwidth(bw), bec(bw),
        delayMs(delay), weight(delay / 1000.0), hsEG(0.0), weightedCost(0.0) {}
};

// 一条路径上最大的点数
#define MaxPathNode (50)

/// 一个图的完整信息
class Graph 
{
public:
    Graph() = default;

    // 添加节点（如果不存在则插入空邻接表）
    void addNode(quint32 node);

    // 添加有向边，权重不能为负；返回 false 并设置 errorMsg
    bool addEdge(quint32 source, quint32 target, double weight, double bandWidth,
                 QString *errorMsg = nullptr);
    // 更新图的权重
    void updateEdge(quint32 source, quint32 target, double weight, double bandWidth);
    // 移除有向边
    void removeEdge(quint32 source, quint32 target);

    // 设置节点对直接边的有效性
    void setEdgeValid(quint32 source, quint32 target, bool status);

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

    // 添加一条NetworkEdge
    void addNetworkEdge(const QSharedPointer<NetworkEdge>& edge);
    // 移除一条NetworkEdge
    void removeNetworkEdge(quint32 source, quint32 target);
    // 存在一条NetworkEdge
    bool hasNetworkEdge(quint32 source, quint32 target);
    // 获取一条NetworkEdge
    QSharedPointer<NetworkEdge> getNetworkEdge(quint32 source, quint32 target);
    // 清理NetworkEdge
    void clearNetworkEdge();

    // 初始化Floyd-Warshall 算法使用的距离和后继节点矩阵
    void initDistAndNextNode();
    // 计算 Floyd-Warshall 全源最短路径并缓存结果
    bool computeFloydWarshall(QString& errorMsg);
    // 是否有有效的 Floyd-Warshall 缓存结果
    bool hasFloydWarshallResult() const;

    // 计算跳数矩阵
    void buildHopMatrix();
    // 获取跳数矩阵
    const QMap<quint64, quint32>& getHopMatrix() const;

    // 返回 source -> target 的最短距离，无路径时返回 infinity
    double distance(quint32 source, quint32 target) const;

    // 从缓存中获取 source -> target 的最短路径，不可达时返回空 QVector
    QVector<quint32> path(quint32 source, quint32 target) const;

    // 返回所有节点对的 (source, target, distance) 列表
    QVector<std::tuple<quint32, quint32, double>> allDistances() const;

    // 返回图中所有节点列表（Floyd-Warshall 计算时的节点顺序）
    QVector<quint32> nodeList() const;

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

public:
    // 节点列表
    QVector<quint32> m_nodes;
    // 所有节点对的有效状态，键为 packNodePair(source, target)
    QMap<quint64, bool> m_edgeValidMap;
    // 邻接表：节点 -> {邻居 -> (权重, 带宽)}
    QMap<quint32, QMap<quint32, std::tuple<double, double>>> m_adjacencyList;
    // 节点对快速查找表，键为 Graph::packNodePair(source, target)
    QMap<quint64, QSharedPointer<NetworkEdge>> m_edgeMap;

    // Floyd-Warshall 缓存结果
    // 标记 Floyd-Warshall 计算结果是否有效
    bool m_floydValid = false;
    // 所有节点对的最短距离，键为 packNodePair(source, target)
    QHash<quint64, double> m_floydDistances;
    // 所有节点对的最短路径，键为 packNodePair(source, target)
    QHash<quint64, QVector<quint32>> m_floydPaths;
    /// 有节点对的的最短跳数矩阵，键为 packNodePair(source, target)
    QMap<quint64, quint32> m_hopMap;

    // Floyd-Warshall 算法中的节点位置索引
    QHash<quint32, int> m_nodeIndex;
    // Floyd-Warshall 算法中的最短距离矩阵
    QVector<double> m_dist;
    // Floyd-Warshall 算法中的后继节点矩阵
    QVector<int> m_nextNode;
};
