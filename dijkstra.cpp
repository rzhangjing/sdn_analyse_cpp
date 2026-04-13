#include "dijkstra.h"

#include <QSet>
#include <limits>
#include <queue>
#include <utility>
#include <functional>
#include <vector>

// -------------------- ShortestPathResult 方法实现 --------------------

double ShortestPathResult::distance(quint32 destination) const
{
    if (!m_valid) {
        return std::numeric_limits<double>::infinity();
    }
    auto it = m_distances.find(destination);
    if (it == m_distances.end()) {
        return std::numeric_limits<double>::infinity();
    }
    return it.value();
}

QVector<quint32> ShortestPathResult::path(quint32 destination) const
{
    if (!m_valid) {
        return {};
    }

    // 目标节点不可达
    auto distIt = m_distances.find(destination);
    if (distIt == m_distances.end() ||
        distIt.value() == std::numeric_limits<double>::infinity()) {
        return {};
    }

    QVector<quint32> result;
    quint32 current = destination;

    while (true) {
        result.prepend(current);
        if (current == m_source) {
            break;
        }
        auto hasIt = m_hasPredecessor.find(current);
        if (hasIt == m_hasPredecessor.end() || !hasIt.value()) {
            // 没有前驱但也不是起点，说明路径断裂
            return {};
        }
        current = m_predecessors.value(current);
    }

    return result;
}

// -------------------- dijkstra 算法实现 --------------------

bool dijkstra(const Graph &graph, quint32 source,
              ShortestPathResult &result, QString *errorMsg)
{
    if (graph.isEmpty()) {
        if (errorMsg) *errorMsg = QStringLiteral("Graph is empty");
        return false;
    }
    if (!graph.containsNode(source)) {
        if (errorMsg) *errorMsg = QStringLiteral("Source node does not exist in the graph");
        return false;
    }

    const double INF = std::numeric_limits<double>::infinity();

    QMap<quint32, double> distances;
    QMap<quint32, quint32> predecessors;
    QMap<quint32, bool> hasPredecessor;
    QSet<quint32> visited;

    // 初始化所有节点距离为无穷
    for (quint32 node : graph.nodes()) {
        distances[node] = INF;
        hasPredecessor[node] = false;
    }
    distances[source] = 0.0;

    // 小根堆：(distance, node)
    using PQItem = std::pair<double, quint32>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [dist, u] = pq.top();
        pq.pop();

        if (visited.contains(u)) {
            continue;
        }
        visited.insert(u);

        const QVector<std::tuple<quint32, double, double>> *nbrs = graph.neighbors(u);
        if (!nbrs) {
            continue;
        }

        for (const auto &[v, w, bw] : *nbrs) {
            if (visited.contains(v)) {
                continue;
            }
            double newDist = dist + w;
            if (newDist < distances.value(v, INF)) {
                distances[v] = newDist;
                predecessors[v] = u;
                hasPredecessor[v] = true;
                pq.push({newDist, v});
            }
        }
    }

    result = ShortestPathResult(source, distances, predecessors, hasPredecessor);
    return true;
}
