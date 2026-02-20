#include "graph.h"

#include <QSet>

namespace {
constexpr int kNoEdge = -1;
}

AdjMatrixGraph::AdjMatrixGraph(int n) {
    reset(n);
}

void AdjMatrixGraph::reset(int n) {
    m_n = n;
    m_weight = QVector<QVector<int>>(n, QVector<int>(n, kNoEdge));
    m_edgeId = QVector<QVector<int>>(n, QVector<int>(n, kNoEdge));
    m_edges.clear();
}

void AdjMatrixGraph::addEdge(int u, int v, int w) {
    if (u == v || u < 0 || v < 0 || u >= m_n || v >= m_n || m_weight[u][v] != kNoEdge) {
        return;
    }
    const int id = m_edges.size();
    m_edges.push_back({id, u, v, w});
    m_weight[u][v] = w;
    m_weight[v][u] = w;
    m_edgeId[u][v] = id;
    m_edgeId[v][u] = id;
}

int AdjMatrixGraph::vertexCount() const { return m_n; }
int AdjMatrixGraph::edgeCount() const { return m_edges.size(); }
QVector<Edge> AdjMatrixGraph::edges() const { return m_edges; }

QVector<int> AdjMatrixGraph::neighbors(int vertex) const {
    QVector<int> result;
    for (int i = 0; i < m_n; ++i) {
        if (m_weight[vertex][i] != kNoEdge) {
            result.push_back(i);
        }
    }
    return result;
}

int AdjMatrixGraph::edgeId(int u, int v) const { return m_edgeId[u][v]; }
int AdjMatrixGraph::edgeWeight(int u, int v) const { return m_weight[u][v]; }

AdjListGraph::AdjListGraph(int n) {
    reset(n);
}

void AdjListGraph::reset(int n) {
    m_n = n;
    m_adj = QVector<QVector<To>>(n);
    m_edges.clear();
}

void AdjListGraph::addEdge(int u, int v, int w) {
    if (u == v || u < 0 || v < 0 || u >= m_n || v >= m_n || edgeId(u, v) != kNoEdge) {
        return;
    }
    const int id = m_edges.size();
    m_edges.push_back({id, u, v, w});
    m_adj[u].push_back({v, w, id});
    m_adj[v].push_back({u, w, id});
}

int AdjListGraph::vertexCount() const { return m_n; }
int AdjListGraph::edgeCount() const { return m_edges.size(); }
QVector<Edge> AdjListGraph::edges() const { return m_edges; }

QVector<int> AdjListGraph::neighbors(int vertex) const {
    QVector<int> result;
    for (const auto& to : m_adj[vertex]) {
        result.push_back(to.v);
    }
    return result;
}

int AdjListGraph::edgeId(int u, int v) const {
    for (const auto& to : m_adj[u]) {
        if (to.v == v) {
            return to.id;
        }
    }
    return kNoEdge;
}

int AdjListGraph::edgeWeight(int u, int v) const {
    for (const auto& to : m_adj[u]) {
        if (to.v == v) {
            return to.w;
        }
    }
    return kNoEdge;
}

namespace GraphFactory {

template <typename GraphType>
GraphType randomConnectedGraph(int n, double density, int minW, int maxW) {
    GraphType g(n);
    if (n <= 1) {
        return g;
    }

    // 先生成随机树，保证连通。
    for (int v = 1; v < n; ++v) {
        const int parent = QRandomGenerator::global()->bounded(v);
        const int w = QRandomGenerator::global()->bounded(minW, maxW + 1);
        g.addEdge(v, parent, w);
    }

    // 再补边。
    for (int u = 0; u < n; ++u) {
        for (int v = u + 1; v < n; ++v) {
            if (g.edgeId(u, v) != kNoEdge) {
                continue;
            }
            const double roll = QRandomGenerator::global()->generateDouble();
            if (roll <= density) {
                const int w = QRandomGenerator::global()->bounded(minW, maxW + 1);
                g.addEdge(u, v, w);
            }
        }
    }
    return g;
}

AdjMatrixGraph randomConnectedMatrixGraph(int n, double density, int minW, int maxW) {
    return randomConnectedGraph<AdjMatrixGraph>(n, density, minW, maxW);
}

AdjListGraph randomConnectedListGraph(int n, double density, int minW, int maxW) {
    return randomConnectedGraph<AdjListGraph>(n, density, minW, maxW);
}

} // namespace GraphFactory
