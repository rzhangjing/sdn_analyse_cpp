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
    QMap<quint32, double>::const_iterator it = m_distances.find(destination);
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

    // 直接从 m_paths 中查找路径
    QMap<quint32, QVector<quint32>>::const_iterator it = m_paths.find(destination);
    if (it == m_paths.end()) {
        return {};
    }
    return it.value();
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

    // 构建无向邻接表：将所有边的反向也加入
    QMap<quint32, QVector<std::tuple<quint32, double, double>>> undirectedAdj;
    for (quint32 node : graph.nodes()) {
        const QVector<std::tuple<quint32, double, double>> *nbrs = graph.neighbors(node);
        if (nbrs) {
            for (const auto &[v, w, bw] : *nbrs) {
                // 添加正向边 node -> v
                undirectedAdj[node].append(std::make_tuple(v, w, bw));
                // 添加反向边 v -> node
                undirectedAdj[v].append(std::make_tuple(node, w, bw));
            }
        }
    }

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

        const QVector<std::tuple<quint32, double, double>> *nbrs = nullptr;
        auto it = undirectedAdj.find(u);
        if (it != undirectedAdj.end()) {
            nbrs = &it.value();
        }
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

    // 预先计算所有可达节点的路径
    QMap<quint32, QVector<quint32>> paths;
    for (auto it = distances.begin(); it != distances.end(); ++it) {
        quint32 node = it.key();
        double dist = it.value();
        if (dist == INF) {
            continue; // 不可达节点，跳过
        }

        // 重建从 source 到 node 的路径
        QVector<quint32> path;
        quint32 current = node;
        while (true) {
            path.prepend(current);
            if (current == source) {
                break;
            }
            QMap<quint32, bool>::const_iterator hasIt = hasPredecessor.find(current);
            if (hasIt == hasPredecessor.end() || !hasIt.value()) {
                // 没有前驱但也不是起点，说明路径断裂
                path.clear();
                break;
            }
            current = predecessors.value(current);
        }
        if (!path.isEmpty()) {
            paths[node] = path;
        }
    }

    result = ShortestPathResult(source, distances, predecessors, hasPredecessor, paths);
    return true;
}
