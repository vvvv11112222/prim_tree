#pragma once

#include "graph.h"

#include <QSet>
#include <QString>
#include <QVector>

struct PrimStep {
    enum class Type {
        ConsiderCut,
        CandidateMinEdges,
        ChooseEdge,
        CompleteMST
    };

    Type type;
    QVector<int> verticesInTree;
    QVector<int> relatedEdgeIds;
    int chosenEdgeId = -1;
    int currentCost = 0;
    QString note;
};

struct MSTSolution {
    QVector<int> edgeIds;
    int totalCost = 0;
    QVector<PrimStep> trace;
};

struct SolverLimits {
    int maxSolutions = 200;
    int maxExpandedStates = 100000;
};

struct SolverStats {
    int expandedStates = 0;
    bool truncatedBySolutionLimit = false;
    bool truncatedByStateLimit = false;

    bool isTruncated() const { return truncatedBySolutionLimit || truncatedByStateLimit; }
};

struct SolveResult {
    QVector<MSTSolution> solutions;
    SolverStats stats;
};

class PrimAllMSTSolver {
public:
    SolveResult solveAll(const IGraphStorage& graph, int startVertex, const SolverLimits& limits = {}) const;

private:
    struct SearchState {
        QSet<int> inTree;
        QVector<int> edgeIds;
        int cost = 0;
        QVector<PrimStep> trace;
    };

    QVector<int> minCutEdges(const IGraphStorage& graph, const QSet<int>& inTree) const;
    QString edgeSetKey(const QVector<int>& edgeIds) const;
    void dfs(const IGraphStorage& graph,
             int bestCost,
             SearchState state,
             QVector<MSTSolution>& out,
             QSet<QString>& dedup,
             const SolverLimits& limits,
             SolverStats& stats) const;
};
