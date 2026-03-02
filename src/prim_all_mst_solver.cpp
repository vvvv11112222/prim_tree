#include "prim_all_mst_solver.h"

#include <QHash>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QVector<int> sortedValues(const QSet<int>& values) {
    QVector<int> out = values.values().toVector();
    std::sort(out.begin(), out.end());
    return out;
}

QVector<int> sortedCopy(const QVector<int>& values) {
    QVector<int> out = values;
    std::sort(out.begin(), out.end());
    return out;
}

constexpr double kEps = 1e-9;

bool almostEqual(double a, double b) {
    return std::abs(a - b) <= kEps;
}

bool strictlyLess(double a, double b) {
    return (b - a) > kEps;
}

QString formatWeight(double value) {
    return QString::number(value, 'g', 12);
}

int appendBranchNode(QVector<BranchTreeNode>& nodes,
                     int parentId,
                     int depth,
                     int viaEdgeId,
                     const QSet<int>& inTree,
                     const QVector<int>& vertexSelectionOrder,
                     const QVector<int>& selectedEdgeIds,
                     const QVector<int>& candidateEdgeIds,
                     double currentCost,
                     double incrementalCost,
                     const QString& label,
                     bool isBacktrack = false,
                     bool isPruned = false,
                     bool isComplete = false) {
    BranchTreeNode node;
    node.id = nodes.size();
    node.parentId = parentId;
    node.depth = depth;
    node.viaEdgeId = viaEdgeId;
    node.verticesInTree = sortedValues(inTree);
    node.vertexSelectionOrder = vertexSelectionOrder;
    node.selectedEdgeIds = sortedCopy(selectedEdgeIds);
    node.candidateEdgeIds = sortedCopy(candidateEdgeIds);
    node.currentCost = currentCost;
    node.incrementalCost = incrementalCost;
    node.isBacktrack = isBacktrack;
    node.isPruned = isPruned;
    node.isComplete = isComplete;
    node.label = label;
    nodes.push_back(node);
    return node.id;
}

void markOptimalPath(QVector<BranchTreeNode>& branchNodes, const QVector<int>& branchPath) {
    for (int id : branchPath) {
        if (id >= 0 && id < branchNodes.size()) {
            branchNodes[id].reachesOptimalSolution = true;
        }
    }
}

} // namespace

QVector<int> PrimAllMSTSolver::minCutEdges(const IGraphStorage& graph, const QSet<int>& inTree) const {
    QVector<int> cut;
    double minWeight = std::numeric_limits<double>::infinity();

    for (const auto& e : graph.edges()) {
        const bool a = inTree.contains(e.u);
        const bool b = inTree.contains(e.v);
        if (a == b) {
            continue;
        }
        if (strictlyLess(e.weight, minWeight)) {
            minWeight = e.weight;
            cut = {e.id};
        } else if (almostEqual(e.weight, minWeight)) {
            cut.push_back(e.id);
        }
    }
    return cut;
}

QString PrimAllMSTSolver::edgeSetKey(const QVector<int>& edgeIds) const {
    QVector<int> sorted = edgeIds;
    std::sort(sorted.begin(), sorted.end());
    QStringList parts;
    for (int id : sorted) {
        parts << QString::number(id);
    }
    return parts.join("-");
}

void PrimAllMSTSolver::dfs(const IGraphStorage& graph,
                           double bestCost,
                           SearchState state,
                           QVector<MSTSolution>& out,
                           QSet<QString>& dedup,
                           const SolverLimits& limits,
                           SolverStats& stats,
                           QVector<BranchTreeNode>& branchNodes,
                           QVector<int>& branchPath) const {
    if (stats.expandedStates >= limits.maxExpandedStates) {
        stats.truncatedByStateLimit = true;
        return;
    }
    if (out.size() >= limits.maxSolutions) {
        stats.truncatedBySolutionLimit = true;
        return;
    }

    ++stats.expandedStates;
    if (state.edgeIds.size() == graph.vertexCount() - 1) {
        if (!almostEqual(state.cost, bestCost)) {
            return;
        }
        const QString key = edgeSetKey(state.edgeIds);
        if (dedup.contains(key)) {
            return;
        }
        dedup.insert(key);

        const int completeId = appendBranchNode(branchNodes,
                                                state.branchNodeId,
                                                state.edgeIds.size() + 1,
                                                -1,
                                                state.inTree,
                                                state.vertexSelectionOrder,
                                                state.edgeIds,
                                                {},
                                                state.cost,
                                                0,
                                                QString("✅ 最优解 cost=%1").arg(formatWeight(state.cost)),
                                                false,
                                                false,
                                                true);
        branchPath.push_back(completeId);
        markOptimalPath(branchNodes, branchPath);
        branchPath.pop_back();

        state.trace.push_back({PrimStep::Type::CompleteMST,
                               sortedValues(state.inTree),
                               sortedCopy(state.edgeIds),
                               -1,
                               state.cost,
                               "找到一棵最小生成树",
                               completeId,
                               state.branchNodeId,
                               -1});
        out.push_back({state.edgeIds, state.cost, state.trace});
        if (out.size() >= limits.maxSolutions) {
            stats.truncatedBySolutionLimit = true;
        }
        return;
    }

    const QVector<int> candidateIds = minCutEdges(graph, state.inTree);
    const int expandId = appendBranchNode(branchNodes,
                                          state.branchNodeId,
                                          state.edgeIds.size() + 1,
                                          -1,
                                          state.inTree,
                                          state.vertexSelectionOrder,
                                          state.edgeIds,
                                          candidateIds,
                                          state.cost,
                                          0,
                                          QString("展开分支，候选边=%1").arg(candidateIds.size()));

    state.trace.push_back({PrimStep::Type::CandidateMinEdges,
                           sortedValues(state.inTree),
                           sortedCopy(candidateIds),
                           -1,
                           state.cost,
                           "在割边中找到当前最小权候选边",
                           expandId,
                           state.branchNodeId,
                           -1});

    const int oldParent = state.branchNodeId;
    state.branchNodeId = expandId;
    branchPath.push_back(expandId);

    const auto& edgeList = graph.edges();
    for (int edgeId : candidateIds) {
        const Edge e = edgeList[edgeId];
        const int nextVertex = state.inTree.contains(e.u) ? e.v : e.u;
        if (state.inTree.contains(nextVertex)) {
            continue;
        }

        SearchState next = state;
        next.inTree.insert(nextVertex);
        next.vertexSelectionOrder.push_back(nextVertex);
        next.edgeIds.push_back(edgeId);
        next.cost += e.weight;

        const int chooseId = appendBranchNode(branchNodes,
                                              state.branchNodeId,
                                              next.edgeIds.size() + 1,
                                              edgeId,
                                              next.inTree,
                                              next.vertexSelectionOrder,
                                              next.edgeIds,
                                              candidateIds,
                                              next.cost,
                                              e.weight,
                                              QString("选择节点 %1").arg(nextVertex));

        next.trace.push_back({PrimStep::Type::ChooseEdge,
                              sortedValues(next.inTree),
                              sortedCopy(candidateIds),
                              edgeId,
                              next.cost,
                              QString("选择边(%1,%2,w=%3)").arg(e.u).arg(e.v).arg(formatWeight(e.weight)),
                              chooseId,
                              state.branchNodeId,
                              edgeId});

        next.branchNodeId = chooseId;
        branchPath.push_back(chooseId);

        if (strictlyLess(bestCost, next.cost)) {
            appendBranchNode(branchNodes,
                             next.branchNodeId,
                             next.edgeIds.size() + 2,
                             -1,
                             next.inTree,
                             next.vertexSelectionOrder,
                             next.edgeIds,
                             {},
                             next.cost,
                             0,
                             QString("✂ 剪枝：cost=%1 > best=%2").arg(formatWeight(next.cost)).arg(formatWeight(bestCost)),
                             false,
                             true,
                             false);
            branchPath.pop_back();
            continue;
        }

        dfs(graph, bestCost, next, out, dedup, limits, stats, branchNodes, branchPath);

        appendBranchNode(branchNodes,
                         next.branchNodeId,
                         next.edgeIds.size() + 2,
                         -1,
                         next.inTree,
                         next.vertexSelectionOrder,
                         next.edgeIds,
                         {},
                         next.cost,
                         0,
                         "↩ 回溯",
                         true,
                         false,
                         false);
        branchPath.pop_back();
    }

    state.branchNodeId = oldParent;
    branchPath.pop_back();
}

SolveResult PrimAllMSTSolver::solveAll(const IGraphStorage& graph, int startVertex, const SolverLimits& limits) const {
    SolveResult result;
    if (graph.vertexCount() == 0 || startVertex < 0 || startVertex >= graph.vertexCount()) {
        return result;
    }

    QSet<int> inTree = {startVertex};
    double bestCost = 0.0;

    while (inTree.size() < graph.vertexCount()) {
        double pickWeight = std::numeric_limits<double>::infinity();
        int pickU = -1;
        int pickV = -1;
        for (const auto& e : graph.edges()) {
            const bool a = inTree.contains(e.u);
            const bool b = inTree.contains(e.v);
            if (a == b) {
                continue;
            }
            if (strictlyLess(e.weight, pickWeight)) {
                pickWeight = e.weight;
                pickU = e.u;
                pickV = e.v;
            }
        }
        if (!std::isfinite(pickWeight)) {
            return result;
        }
        bestCost += pickWeight;
        inTree.insert(pickU);
        inTree.insert(pickV);
    }

    SearchState init;
    init.inTree.insert(startVertex);
    init.vertexSelectionOrder.push_back(startVertex);

    result.branchRootId = appendBranchNode(result.branchNodes,
                                           -1,
                                           0,
                                           -1,
                                           init.inTree,
                                           init.vertexSelectionOrder,
                                           {},
                                           {},
                                           0,
                                           0,
                                           QString("起点=%1").arg(startVertex));
    init.branchNodeId = result.branchRootId;

    QSet<QString> dedup;
    QVector<int> branchPath = {result.branchRootId};
    dfs(graph,
        bestCost,
        init,
        result.solutions,
        dedup,
        limits,
        result.stats,
        result.branchNodes,
        branchPath);
    return result;
}
