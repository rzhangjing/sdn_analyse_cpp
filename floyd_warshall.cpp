#include "floyd_warshall.h"

#include <limits>
#include <vector>

// -------------------- FloydWarshallResult 方法实现 --------------------

double FloydWarshallResult::distance(quint32 source, quint32 target) const
{
    if (!m_valid) {
        return std::numeric_limits<double>::infinity();
    }
    QHash<QPair<quint32, quint32>, double>::const_iterator it = m_distances.find(qMakePair(source, target));
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

    // 相同节点
    if (source == target) {
        return {source};
    }

    // 从缓存中获取路径
    QHash<QPair<quint32, quint32>, QVector<quint32>>::const_iterator it = m_paths.find(qMakePair(source, target));
    if (it != m_paths.end()) {
        return it.value();
    }

    // 缓存中不存在，表示不可达
    return {};
}

QVector<std::tuple<quint32, quint32, double>> FloydWarshallResult::allDistances() const
{
    QVector<std::tuple<quint32, quint32, double>> result;
    if (!m_valid) {
        return result;
    }
    result.reserve(m_distances.size());
    for (QHash<QPair<quint32, quint32>, double>::const_iterator it = m_distances.cbegin(); it != m_distances.cend(); ++it) {
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

    // 收集节点并建立索引映射（用 QHash 加速查找）
    QList<quint32> nodeList = graph.nodes();
    const int n = nodeList.size();

    QHash<quint32, int> nodeIndex;
    nodeIndex.reserve(n);
    for (int i = 0; i < n; ++i) {
        nodeIndex[nodeList[i]] = i;
    }

    // 使用一维连续内存数组提升缓存命中率
    std::vector<double> dist(static_cast<size_t>(n) * n, INF);
    std::vector<int> nextNode(static_cast<size_t>(n) * n, -1);

    // 宏定义二维索引访问
    #define IDX(i, j) (static_cast<size_t>(i) * n + (j))

    // 对角线初始化为 0
    for (int i = 0; i < n; ++i) {
        dist[IDX(i, i)] = 0.0;
    }

    // 根据图的边初始化（无向图：同时初始化正向和反向边）
    for (int i = 0; i < n; ++i) {
        quint32 u = nodeList[i];
        const QVector<std::tuple<quint32, double, double>> *nbrs = graph.neighbors(u);
        if (!nbrs) continue;
        for (const auto &[v, w, bw] : *nbrs) {
            QHash<quint32, int>::const_iterator it = nodeIndex.constFind(v);
            if (it == nodeIndex.cend()) continue;
            int j = it.value();
            // 若存在多条边，取最小权重
            // 正向边 u -> v
            if (w < dist[IDX(i, j)]) {
                dist[IDX(i, j)] = w;
                nextNode[IDX(i, j)] = j;
            }
            // 反向边 v -> u（无向图）
            if (w < dist[IDX(j, i)]) {
                dist[IDX(j, i)] = w;
                nextNode[IDX(j, i)] = i;
            }
        }
    }

    // Floyd-Warshall 三重循环（连续内存访问，缓存友好）
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            double dik = dist[IDX(i, k)];
            if (dik == INF) continue;
            const size_t rowI = static_cast<size_t>(i) * n;
            const size_t rowK = static_cast<size_t>(k) * n;
            for (int j = 0; j < n; ++j) {
                double dkj = dist[rowK + j];
                if (dkj == INF) continue;
                double newDist = dik + dkj;
                if (newDist < dist[rowI + j]) {
                    dist[rowI + j] = newDist;
                    nextNode[rowI + j] = nextNode[IDX(i, k)];
                }
            }
        }
    }

    // 将矩阵结果转换回节点 ID 映射（用 QHash，预分配容量）
    QHash<QPair<quint32, quint32>, double> distances;
    distances.reserve(n * n);

    for (int i = 0; i < n; ++i) {
        quint32 u = nodeList[i];
        const size_t rowI = static_cast<size_t>(i) * n;
        for (int j = 0; j < n; ++j) {
            distances.insert(qMakePair(u, nodeList[j]), dist[rowI + j]);
        }
    }

    QVector<quint32> nodes(nodeList.begin(), nodeList.end());

    // 预计算所有节点对的最短路径并缓存
    QHash<QPair<quint32, quint32>, QVector<quint32>> paths;
    paths.reserve(n * n);
    for (int i = 0; i < n; ++i) {
        const size_t rowI = static_cast<size_t>(i) * n;
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            if (dist[rowI + j] == INF) continue;
            if (nextNode[rowI + j] < 0) continue;

            quint32 src = nodeList[i];
            quint32 tgt = nodeList[j];
            QVector<quint32> p;
            p.reserve(n); // 预分配避免多次扩容
            int cur = i;
            p.append(nodeList[cur]);
            while (cur != j) {
                cur = nextNode[static_cast<size_t>(cur) * n + j];
                if (cur < 0) { p.clear(); break; }
                p.append(nodeList[cur]);
                if (p.size() > n + 1) { p.clear(); break; }
            }
            if (!p.isEmpty()) {
                paths.insert(qMakePair(src, tgt), std::move(p));
            }
        }
    }

    #undef IDX

    result = FloydWarshallResult(std::move(distances), std::move(nodes), std::move(paths));
    return true;
}
