#pragma once

#include <QMap>
#include <QVector>
#include <QList>
#include <QString>
#include <tuple>
#include "edge.h"

class Graph {
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
    const QVector<std::tuple<quint32, double, double>> *neighbors(quint32 node) const;

    bool containsNode(quint32 node) const;
    int nodeCount() const;
    QList<quint32> nodes() const;
    bool isEmpty() const;

private:
    // 邻接表：节点 -> [(邻居, 权重, 带宽)]
    QMap<quint32, QVector<std::tuple<quint32, double, double>>> m_adjacencyList;
};
