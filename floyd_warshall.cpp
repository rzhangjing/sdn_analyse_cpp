#include "floyd_warshall.h"

#include <limits>

// -------------------- FloydWarshallResult 方法实现 --------------------

double FloydWarshallResult::distance(quint32 source, quint32 target) const
{
    if (!m_valid) {
        return std::numeric_limits<double>::infinity();
    }
    auto it = m_distances.find(qMakePair(source, target));
    if (it == m_distances.end()) {
        return std::numeric_limits<double>::infinity();
    }
    return it.value();
}

QVector<quint32> FloydWarshallResult::path(quint32 source, quint32 target) const
{
    if (!m_valid) {
        return {};
    }

    // 检查路径是否存在
    auto distIt = m_distances.find(qMakePair(source, target));
    if (distIt == m_distances.end() ||
        distIt.value() == std::numeric_limits<double>::infinity()) {
        return {};
    }

    // 相同节点
    if (source == target) {
        return {source};
    }

    // 通过 next_hop 重建路径
    QVector<quint32> result;
    quint32 current = source;
    result.append(current);

    while (current != target) {
        auto nextIt = m_nextHop.find(qMakePair(current, target));
        if (nextIt == m_nextHop.end()) {
            // 路径断裂
            return {};
        }
        current = nextIt.value();
        result.append(current);

        // 防止循环（理论上 Floyd-Warshall 结果中不应存在）
        if (result.size() > static_cast<int>(m_nodes.size()) + 1) {
            return {};
        }
    }

    return result;
}

QVector<std::tuple<quint32, quint32, double>> FloydWarshallResult::allDistances() const
{
    QVector<std::tuple<quint32, quint32, double>> result;
    if (!m_valid) {
        return result;
    }
    for (auto it = m_distances.cbegin(); it != m_distances.cend(); ++it) {
        result.append(std::make_tuple(it.key().first, it.key().second, it.value()));
    }
    return result;
}

QVector<quint32> FloydWarshallResult::nodeList() const
{
    return m_nodes;
}

// -------------------- floydWarshall 算法实现 --------------------

bool floydWarshall(const Graph &graph, FloydWarshallResult &result, QString *errorMsg)
{
    if (graph.isEmpty()) {
        if (errorMsg) *errorMsg = QStringLiteral("Graph is empty");
        return false;
    }

    const double INF = std::numeric_limits<double>::infinity();

    // 收集节点并建立索引映射
    QList<quint32> nodeList = graph.nodes();
    int n = nodeList.size();

    QMap<quint32, int> nodeIndex;
    for (int i = 0; i < n; ++i) {
        nodeIndex[nodeList[i]] = i;
    }

    // 初始化距离矩阵和 next 矩阵
    QVector<QVector<double>> dist(n, QVector<double>(n, INF));
    // nextNode[i][j] = -1 表示无路径，否则表示 i -> j 路径上 i 的下一跳节点索引
    QVector<QVector<int>> nextNode(n, QVector<int>(n, -1));

    // 对角线初始化为 0
    for (int i = 0; i < n; ++i) {
        dist[i][i] = 0.0;
    }

    // 根据图的边初始化
    for (int i = 0; i < n; ++i) {
        quint32 u = nodeList[i];
        const QVector<std::tuple<quint32, double, double>> *nbrs = graph.neighbors(u);
        if (!nbrs) continue;
        for (const auto &[v, w, bw] : *nbrs) {
            int j = nodeIndex.value(v, -1);
            if (j < 0) continue;
            // 若存在多条边，取最小权重
            if (w < dist[i][j]) {
                dist[i][j] = w;
                nextNode[i][j] = j;
            }
        }
    }

    // Floyd-Warshall 三重循环
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            if (dist[i][k] == INF) continue;
            for (int j = 0; j < n; ++j) {
                if (dist[k][j] == INF) continue;
                double newDist = dist[i][k] + dist[k][j];
                if (newDist < dist[i][j]) {
                    dist[i][j] = newDist;
                    nextNode[i][j] = nextNode[i][k];
                }
            }
        }
    }

    // 将矩阵结果转换回节点 ID 映射
    QMap<QPair<quint32, quint32>, double> distances;
    QMap<QPair<quint32, quint32>, quint32> nextHop;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            quint32 u = nodeList[i];
            quint32 v = nodeList[j];
            distances[qMakePair(u, v)] = dist[i][j];
            if (nextNode[i][j] >= 0) {
                nextHop[qMakePair(u, v)] = nodeList[nextNode[i][j]];
            }
        }
    }

    QVector<quint32> nodes(nodeList.begin(), nodeList.end());
    result = FloydWarshallResult(distances, nextHop, nodes);
    return true;
}
