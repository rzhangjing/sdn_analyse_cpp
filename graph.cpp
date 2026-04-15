#include "graph.h"

void Graph::addNode(quint32 node)
{
    if (!m_adjacencyList.contains(node)) {
        m_adjacencyList.insert(node, QVector<std::tuple<quint32, double, double>>());
    }
}

bool Graph::addEdge(quint32 source, quint32 target, double weight, double bandWidth,
                    QString *errorMsg)
{
    if (weight < 0.0) {
        if (errorMsg) {
            *errorMsg = QStringLiteral("Edge weight cannot be negative");
        }
        return false;
    }

    addNode(source);
    addNode(target);
    m_adjacencyList[source].append(std::make_tuple(target, weight, bandWidth));
    return true;
}

bool Graph::fromEdges(const QVector<Edge> &edges, Graph &outGraph, QString *errorMsg)
{
    Graph g;
    for (const Edge &e : edges) {
        if (!g.addEdge(e.source, e.target, e.weight, e.bandWidth, errorMsg)) {
            return false;
        }
    }
    outGraph = std::move(g);
    return true;
}

const QVector<std::tuple<quint32, double, double>> *Graph::neighbors(quint32 node) const
{
    QMap<quint32, QVector<std::tuple<quint32, double, double>>>::const_iterator it = m_adjacencyList.find(node);
    if (it == m_adjacencyList.end()) {
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
