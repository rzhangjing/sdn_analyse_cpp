#include "graph.h"

#include <limits>
#include <vector>

void Graph::addNode(quint32 node)
{
    if (!m_adjacencyList.contains(node))
    {
        m_adjacencyList.insert(node, QMap<quint32, std::tuple<double, double>>());
        m_nodes.append(node);
    }
}

bool Graph::addEdge(quint32 source, quint32 target, double weight, double bandWidth,
                    QString *errorMsg)
{
    if (weight < 0.0)
    {
        if (errorMsg)
        {
            *errorMsg = QStringLiteral("Edge weight cannot be negative");
        }
        return false;
    }

    addNode(source);
    addNode(target);
    m_adjacencyList[source][target] = std::make_tuple(weight, bandWidth);
    m_edgeValidMap.insert(packNodePair(source, target), true);
    return true;
}

// 设置节点对直接边的有效性
void Graph::setEdgeValid(quint32 source, quint32 target, bool status)
{
    quint64 firstPairVal = packNodePair(source, target);
    if (m_edgeValidMap.contains(firstPairVal))
    {
        m_edgeValidMap[firstPairVal] = status;
    }
    quint64 secondPairVal = packNodePair(source, target);
    if (m_edgeValidMap.contains(secondPairVal))
    {
        m_edgeValidMap[secondPairVal] = status;
    }
}

bool Graph::fromEdges(const QVector<Edge> &edges, Graph &outGraph, QString *errorMsg)
{
    Graph g;
    for (const Edge &e : edges)
    {
        if (!g.addEdge(e.source, e.target, e.weight, e.bandWidth, errorMsg))
        {
            return false;
        }
    }
    outGraph = std::move(g);
    return true;
}

const QMap<quint32, std::tuple<double, double>> *Graph::neighbors(quint32 node) const
{
    QMap<quint32, QMap<quint32, std::tuple<double, double>>>::const_iterator it = m_adjacencyList.find(node);
    if (it == m_adjacencyList.end())
    {
        return nullptr;
    }
    return &it.value();
}

bool Graph::containsNode(quint32 node) const
{
    return m_adjacencyList.contains(node);
}

int Graph::nodeCount() const
{
    return m_adjacencyList.size();
}

QList<quint32> Graph::nodes() const
{
    return m_adjacencyList.keys();
}

bool Graph::isEmpty() const
{
    return m_adjacencyList.isEmpty();
}

void Graph::clear()
{
    m_nodes.clear();
    m_edgeValidMap.clear();
    m_adjacencyList.clear();
    m_floydValid = false;
    m_floydDistances.clear();
    m_floydPaths.clear();
    m_hopMap.clear();
    m_nodeIndex.clear();
    m_dist.clear();
    m_nextNode.clear();
}

// 初始化 Floyd-Warshall 算法使用的节点索引、距离矩阵和后继节点矩阵
void Graph::initDistAndNextNode()
{
    QList<quint32> nl = nodes();
    const int n = nl.size();

    m_nodeIndex.clear();
    m_nodeIndex.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        m_nodeIndex[nl[i]] = i;
    }
    const double INF = std::numeric_limits<double>::infinity();
    m_dist = QVector<double>(static_cast<size_t>(n) * n, INF);
    m_nextNode = QVector<int>(static_cast<size_t>(n) * n, -1);
}

bool Graph::computeFloydWarshall(QString *errorMsg)
{
     m_hopMap.clear();
    if (m_nodeIndex.size() <= 0) { initDistAndNextNode(); }

    if (isEmpty())
    {
        if (errorMsg) *errorMsg = QStringLiteral("Graph is empty");
        return false;
    }

    QList<quint32> nl = nodes();
    const int n = nl.size();
    const double INF = std::numeric_limits<double>::infinity();

    // 初始化 m_dist 为无穷大，m_nextNode 为 -1
    m_dist = QVector<double>(static_cast<size_t>(n) * n, INF);
    m_nextNode = QVector<int>(static_cast<size_t>(n) * n, -1);

#define IDX(i, j) (static_cast<size_t>(i) * n + (j))

    for (int i = 0; i < n; ++i)
    {
        m_dist[IDX(i, i)] = 0.0;
    }

    for (int i = 0; i < n; ++i)
    {
        quint32 u = nl[i];
        const QMap<quint32, std::tuple<double, double>> *nbrs = neighbors(u);
        if (!nbrs) continue;
        for (auto it = nbrs->cbegin(); it != nbrs->cend(); ++it)
        {
            quint32 v = it.key();
            double w = std::get<0>(it.value());
            double bw = std::get<1>(it.value());
            QHash<quint32, int>::const_iterator nit = m_nodeIndex.constFind(v);
            if (nit == m_nodeIndex.cend()) continue;
            int j = nit.value();
            if (w < m_dist[IDX(i, j)])
            {
                m_dist[IDX(i, j)] = w;
                m_nextNode[IDX(i, j)] = j;
            }
            if (w < m_dist[IDX(j, i)])
            {
                m_dist[IDX(j, i)] = w;
                m_nextNode[IDX(j, i)] = i;
            }
        }
    }

    for (int k = 0; k < n; ++k)
    {
        for (int i = 0; i < n; ++i)
        {
            double dik = m_dist[IDX(i, k)];
            if (dik == INF) continue;
            const size_t rowI = static_cast<size_t>(i) * n;
            const size_t rowK = static_cast<size_t>(k) * n;
            for (int j = 0; j < n; ++j)
            {
                double dkj = m_dist[rowK + j];
                if (dkj == INF) continue;
                double newDist = dik + dkj;
                if (newDist < m_dist[rowI + j])
                {
                    m_dist[rowI + j] = newDist;
                    m_nextNode[rowI + j] = m_nextNode[IDX(i, k)];
                }
            }
        }
    }

    m_floydDistances.clear();
    m_floydDistances.reserve(n * n);
    for (int i = 0; i < n; ++i)
    {
        quint32 u = nl[i];
        const size_t rowI = static_cast<size_t>(i) * n;
        for (int j = 0; j < n; ++j)
        {
            m_floydDistances.insert(Graph::packNodePair(u, nl[j]), m_dist[rowI + j]);
        }
    }

    m_floydPaths.clear();
    m_floydPaths.reserve(n * n);
    for (int i = 0; i < n; ++i)
    {
        const size_t rowI = static_cast<size_t>(i) * n;
        for (int j = 0; j < n; ++j)
        {
            if (i == j) continue;
            if (m_dist[rowI + j] == INF) continue;
            if (m_nextNode[rowI + j] < 0) continue;

            quint32 src = nl[i];
            quint32 tgt = nl[j];
            QVector<quint32> p;
            p.reserve(n);
            int cur = i;
            p.append(nl[cur]);
            while (cur != j)
            {
                cur = m_nextNode[static_cast<size_t>(cur) * n + j];
                if (cur < 0) { p.clear(); break; }
                p.append(nl[cur]);
                if (p.size() > n + 1) { p.clear(); break; }
            }
            if (!p.isEmpty())
            {
                m_floydPaths.insert(Graph::packNodePair(src, tgt), std::move(p));
            }
        }
    }

#undef IDX

    m_floydValid = true;
    // 计算跳数矩阵
    buildHopMatrix();

    return true;
}

bool Graph::hasFloydWarshallResult() const
{
    return m_floydValid;
}

// 计算跳数矩阵
void Graph::buildHopMatrix()
{
    m_hopMap.clear();
    for (quint32 source : m_nodes)
    {
        for (quint32 target : m_nodes)
        {
            if (source == target) continue;
            auto it = m_floydPaths.find(Graph::packNodePair(source, target));
            if (it != m_floydPaths.end())
            {
                m_hopMap.insert(Graph::packNodePair(source, target), static_cast<quint32>((*it).size() - 1));
            }
        }
    }
}

double Graph::distance(quint32 source, quint32 target) const
{
    if (!m_floydValid)
    {
        return std::numeric_limits<double>::infinity();
    }
    auto it = m_floydDistances.find(Graph::packNodePair(source, target));
    if (it == m_floydDistances.end())
    {
        return std::numeric_limits<double>::infinity();
    }
    return it.value();
}

QVector<quint32> Graph::path(quint32 source, quint32 target) const
{
    if (!m_floydValid)
    {
        return {};
    }
    if (source == target)
    {
        return {source};
    }
    auto it = m_floydPaths.find(Graph::packNodePair(source, target));
    if (it != m_floydPaths.end())
    {
        return it.value();
    }
    return {};
}

QVector<std::tuple<quint32, quint32, double>> Graph::allDistances() const
{
    QVector<std::tuple<quint32, quint32, double>> result;
    if (!m_floydValid)
    {
        return result;
    }
    result.reserve(m_floydDistances.size());
    for (auto it = m_floydDistances.cbegin(); it != m_floydDistances.cend(); ++it)
    {
        auto pair = Graph::unpackNodePair(it.key());
        result.append(std::make_tuple(pair.first, pair.second, it.value()));
    }
    return result;
}

QVector<quint32> Graph::nodeList() const
{
    return m_nodes;
}

const QHash<quint64, QVector<quint32>> &Graph::allPaths() const
{
    return m_floydPaths;
}
