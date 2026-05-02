#include "graph.h"

#include <limits>
#include <vector>

void Graph::addNode(quint32 node)
{
    if (!m_adjacencyList.contains(node))
    {
        m_adjacencyList.insert(node, QMap<quint32, std::tuple<double, double>>());
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
    return true;
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
    m_adjacencyList.clear();
    m_floydValid = false;
    m_floydDistances.clear();
    m_floydPaths.clear();
    m_floydNodes.clear();
}

bool Graph::computeFloydWarshall(QString *errorMsg)
{
    if (isEmpty())
    {
        if (errorMsg) *errorMsg = QStringLiteral("Graph is empty");
        return false;
    }

    const double INF = std::numeric_limits<double>::infinity();

    QList<quint32> nl = nodes();
    const int n = nl.size();

    QHash<quint32, int> nodeIndex;
    nodeIndex.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        nodeIndex[nl[i]] = i;
    }

    std::vector<double> dist(static_cast<size_t>(n) * n, INF);
    std::vector<int> nextNode(static_cast<size_t>(n) * n, -1);

#define IDX(i, j) (static_cast<size_t>(i) * n + (j))

    for (int i = 0; i < n; ++i)
    {
        dist[IDX(i, i)] = 0.0;
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
            QHash<quint32, int>::const_iterator nit = nodeIndex.constFind(v);
            if (nit == nodeIndex.cend()) continue;
            int j = nit.value();
            if (w < dist[IDX(i, j)])
            {
                dist[IDX(i, j)] = w;
                nextNode[IDX(i, j)] = j;
            }
            if (w < dist[IDX(j, i)])
            {
                dist[IDX(j, i)] = w;
                nextNode[IDX(j, i)] = i;
            }
        }
    }

    for (int k = 0; k < n; ++k)
    {
        for (int i = 0; i < n; ++i)
        {
            double dik = dist[IDX(i, k)];
            if (dik == INF) continue;
            const size_t rowI = static_cast<size_t>(i) * n;
            const size_t rowK = static_cast<size_t>(k) * n;
            for (int j = 0; j < n; ++j)
            {
                double dkj = dist[rowK + j];
                if (dkj == INF) continue;
                double newDist = dik + dkj;
                if (newDist < dist[rowI + j])
                {
                    dist[rowI + j] = newDist;
                    nextNode[rowI + j] = nextNode[IDX(i, k)];
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
            m_floydDistances.insert(Graph::packNodePair(u, nl[j]), dist[rowI + j]);
        }
    }

    m_floydNodes = QVector<quint32>(nl.begin(), nl.end());
    m_floydPaths.clear();
    m_floydPaths.reserve(n * n);
    for (int i = 0; i < n; ++i)
    {
        const size_t rowI = static_cast<size_t>(i) * n;
        for (int j = 0; j < n; ++j)
        {
            if (i == j) continue;
            if (dist[rowI + j] == INF) continue;
            if (nextNode[rowI + j] < 0) continue;

            quint32 src = nl[i];
            quint32 tgt = nl[j];
            QVector<quint32> p;
            p.reserve(n);
            int cur = i;
            p.append(nl[cur]);
            while (cur != j)
            {
                cur = nextNode[static_cast<size_t>(cur) * n + j];
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
    return true;
}

bool Graph::hasFloydWarshallResult() const
{
    return m_floydValid;
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
    return m_floydNodes;
}

const QHash<quint64, QVector<quint32>> &Graph::allPaths() const
{
    return m_floydPaths;
}
