#include "prim_all_mst_solver.h"

#include <QHash>
#include <QStringList>

#include <algorithm>
#include <climits>

QVector<int> PrimAllMSTSolver::minCutEdges(const IGraphStorage& graph, const QSet<int>& inTree) const {
    QVector<int> cut;
    int minWeight = INT_MAX;

    for (const auto& e : graph.edges()) {
        const bool a = inTree.contains(e.u);
        const bool b = inTree.contains(e.v);
        if (a == b) {
            continue;
        }
        if (e.weight < minWeight) {
            minWeight = e.weight;
            cut = {e.id};
        } else if (e.weight == minWeight) {
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
                           int bestCost,
                           SearchState state,
                           QVector<MSTSolution>& out,
                           QSet<QString>& dedup) const {
    if (state.edgeIds.size() == graph.vertexCount() - 1) {
        if (state.cost != bestCost) {
            return;
        }
        const QString key = edgeSetKey(state.edgeIds);
        if (dedup.contains(key)) {
            return;
        }
        dedup.insert(key);
        state.trace.push_back({PrimStep::Type::CompleteMST, state.inTree.values().toVector(), state.edgeIds, -1, state.cost, "找到一棵最小生成树"});
        out.push_back({state.edgeIds, state.cost, state.trace});
        return;
    }

    const QVector<int> candidateIds = minCutEdges(graph, state.inTree);
    state.trace.push_back({PrimStep::Type::CandidateMinEdges,
                           state.inTree.values().toVector(),
                           candidateIds,
                           -1,
                           state.cost,
                           "在割边中找到当前最小权候选边"});

    const auto& edgeList = graph.edges();
    for (int edgeId : candidateIds) {
        const Edge e = edgeList[edgeId];
        const int nextVertex = state.inTree.contains(e.u) ? e.v : e.u;
        if (state.inTree.contains(nextVertex)) {
            continue;
        }

        SearchState next = state;
        next.inTree.insert(nextVertex);
        next.edgeIds.push_back(edgeId);
        next.cost += e.weight;
        next.trace.push_back({PrimStep::Type::ChooseEdge,
                              next.inTree.values().toVector(),
                              candidateIds,
                              edgeId,
                              next.cost,
                              QString("选择边(%1,%2,w=%3)").arg(e.u).arg(e.v).arg(e.weight)});
        if (next.cost > bestCost) {
            continue;
        }
        dfs(graph, bestCost, next, out, dedup);
    }
}

QVector<MSTSolution> PrimAllMSTSolver::solveAll(const IGraphStorage& graph, int startVertex) const {
    QVector<MSTSolution> result;
    if (graph.vertexCount() == 0 || startVertex < 0 || startVertex >= graph.vertexCount()) {
        return result;
    }

    // 先做一次标准 Prim 得到最优值上界。
    QSet<int> inTree = {startVertex};
    int bestCost = 0;

    while (inTree.size() < graph.vertexCount()) {
        int pickWeight = INT_MAX;
        int pickU = -1;
        int pickV = -1;
        for (const auto& e : graph.edges()) {
            const bool a = inTree.contains(e.u);
            const bool b = inTree.contains(e.v);
            if (a == b) {
                continue;
            }
            if (e.weight < pickWeight) {
                pickWeight = e.weight;
                pickU = e.u;
                pickV = e.v;
            }
        }
        if (pickU == -1) {
            return {};
        }
        bestCost += pickWeight;
        inTree.insert(inTree.contains(pickU) ? pickV : pickU);
    }

    SearchState init;
    init.inTree.insert(startVertex);
    init.trace.push_back({PrimStep::Type::ConsiderCut, {startVertex}, {}, -1, 0, "从随机起点开始"});

    QSet<QString> dedup;
    dfs(graph, bestCost, init, result, dedup);
    return result;
}
