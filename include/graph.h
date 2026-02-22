#pragma once

#include <QRandomGenerator>
#include <QVector>

struct Edge {
    int id;
    int u;
    int v;
    int weight;
};

class IGraphStorage {
public:
    virtual ~IGraphStorage() = default;
    virtual int vertexCount() const = 0;
    virtual int edgeCount() const = 0;
    virtual const QVector<Edge>& edges() const = 0;
    virtual QVector<int> neighbors(int vertex) const = 0;
    virtual int edgeId(int u, int v) const = 0;
    virtual int edgeWeight(int u, int v) const = 0;
};

class AdjMatrixGraph final : public IGraphStorage {
public:
    explicit AdjMatrixGraph(int n = 0);
    void reset(int n);
    void addEdge(int u, int v, int w);

    int vertexCount() const override;
    int edgeCount() const override;
    const QVector<Edge>& edges() const override;
    QVector<int> neighbors(int vertex) const override;
    int edgeId(int u, int v) const override;
    int edgeWeight(int u, int v) const override;

private:
    int m_n;
    QVector<QVector<int>> m_weight;
    QVector<QVector<int>> m_edgeId;
    QVector<Edge> m_edges;
};

class AdjListGraph final : public IGraphStorage {
public:
    explicit AdjListGraph(int n = 0);
    void reset(int n);
    void addEdge(int u, int v, int w);

    int vertexCount() const override;
    int edgeCount() const override;
    const QVector<Edge>& edges() const override;
    QVector<int> neighbors(int vertex) const override;
    int edgeId(int u, int v) const override;
    int edgeWeight(int u, int v) const override;

private:
    struct To {
        int v;
        int w;
        int id;
    };

    int m_n;
    QVector<QVector<To>> m_adj;
    QVector<Edge> m_edges;
};

namespace GraphFactory {
AdjMatrixGraph randomConnectedMatrixGraph(int n, double density, int minW, int maxW);
AdjListGraph randomConnectedListGraph(int n, double density, int minW, int maxW);
}
