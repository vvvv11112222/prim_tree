#include "mainwindow.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHash>
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSpinBox>
#include <QStringList>
#include <QSplitter>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace {
QString defaultMatrixInput() {
    return "0 2 3 -1\n"
           "2 0 1 4\n"
           "3 1 0 5\n"
           "-1 4 5 0";
}

QString defaultListInput() {
    return "0 1 2 2 3\n"
           "1 2 1 3 4\n"
           "2 3 5\n"
           "3";
}

QString formatWeight(double value) {
    return QString::number(value, 'g', 12);
}

bool parseWeightToken(const QString& token, double& value, bool& hasEdge) {
    const QString t = token.trimmed().toUpper();
    if (t == "INF" || t == "X" || t == "-1") {
        hasEdge = false;
        value = -1;
        return true;
    }
    bool ok = false;
    const double w = t.toDouble(&ok);
    if (!ok) {
        return false;
    }
    if (w <= 0) {
        hasEdge = false;
        value = -1;
        return true;
    }
    hasEdge = true;
    value = w;
    return true;
}

QVector<QPointF> buildWeightAwareLayout(const QVector<Edge>& edges, int vertexCount, const QPointF& center) {
    QVector<QPointF> positions(vertexCount, center);
    if (vertexCount <= 0) {
        return positions;
    }
    if (vertexCount == 1) {
        return positions;
    }

    constexpr double kMinTargetDistance = 75.0;
    constexpr double kMaxTargetDistance = 235.0;
    constexpr double kCanvasRadius = 190.0;
    constexpr double kMinVertexGap = 34.0;
    constexpr double kHardOverlapGap = 24.0;
    constexpr double kMinAngleRad = 0.52; // ~30°
    constexpr double kCrowdRadius = 42.0;

    constexpr double kLengthWeight = 1.0;
    constexpr double kSeparationWeight = 140.0;
    constexpr double kOverlapWeight = 360.0;
    constexpr double kAngleWeight = 75.0;
    constexpr double kCrowdWeight = 90.0;

    double minWeight = std::numeric_limits<double>::infinity();
    double maxWeight = -std::numeric_limits<double>::infinity();
    for (const auto& e : edges) {
        minWeight = std::min(minWeight, e.weight);
        maxWeight = std::max(maxWeight, e.weight);
    }

    const auto mapWeightToTargetDistance = [&](double weight) {
        if (minWeight >= maxWeight) {
            return (kMinTargetDistance + kMaxTargetDistance) * 0.5;
        }
        const double norm = static_cast<double>(weight - minWeight) / static_cast<double>(maxWeight - minWeight);
        return kMinTargetDistance + norm * (kMaxTargetDistance - kMinTargetDistance);
    };

    struct NeighborRef {
        int to;
        int edgeIndex;
    };

    QVector<QVector<NeighborRef>> adjacency(vertexCount);
    for (int i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        adjacency[e.u].push_back({e.v, i});
        adjacency[e.v].push_back({e.u, i});
    }

    auto distance = [](const QPointF& a, const QPointF& b) {
        return std::hypot(a.x() - b.x(), a.y() - b.y());
    };

    auto pointToSegmentDistance = [&](const QPointF& p, const QPointF& a, const QPointF& b) {
        const QPointF ab = b - a;
        const double ab2 = ab.x() * ab.x() + ab.y() * ab.y();
        if (ab2 <= 1e-9) {
            return distance(p, a);
        }
        const QPointF ap = p - a;
        double t = (ap.x() * ab.x() + ap.y() * ab.y()) / ab2;
        t = qBound(0.0, t, 1.0);
        const QPointF proj = a + ab * t;
        return distance(p, proj);
    };

    auto clampToCanvas = [&](const QPointF& p) {
        QPointF d = p - center;
        const double len = std::hypot(d.x(), d.y());
        if (len <= kCanvasRadius || len <= 1e-9) {
            return p;
        }
        return center + d * (kCanvasRadius / len);
    };

    auto energy = [&](const QVector<QPointF>& pts, const QVector<bool>& active) {
        double e = 0.0;

        for (const auto& edge : edges) {
            if (!active[edge.u] || !active[edge.v]) {
                continue;
            }
            const double d = distance(pts[edge.u], pts[edge.v]);
            const double target = mapWeightToTargetDistance(edge.weight);
            const double diff = d - target;
            e += kLengthWeight * diff * diff;
        }

        for (int i = 0; i < vertexCount; ++i) {
            if (!active[i]) {
                continue;
            }
            for (int j = i + 1; j < vertexCount; ++j) {
                if (!active[j]) {
                    continue;
                }
                const double d = distance(pts[i], pts[j]);
                if (d < kMinVertexGap) {
                    const double p = kMinVertexGap - d;
                    e += kSeparationWeight * p * p;
                }
                if (d < kHardOverlapGap) {
                    const double p = kHardOverlapGap - d;
                    e += kOverlapWeight * p * p;
                }
            }
        }

        for (int v = 0; v < vertexCount; ++v) {
            if (!active[v]) {
                continue;
            }
            const auto& nbs = adjacency[v];
            for (int i = 0; i < nbs.size(); ++i) {
                if (!active[nbs[i].to]) {
                    continue;
                }
                for (int j = i + 1; j < nbs.size(); ++j) {
                    if (!active[nbs[j].to]) {
                        continue;
                    }
                    const QPointF a = pts[nbs[i].to] - pts[v];
                    const QPointF b = pts[nbs[j].to] - pts[v];
                    const double la = std::hypot(a.x(), a.y());
                    const double lb = std::hypot(b.x(), b.y());
                    if (la <= 1e-6 || lb <= 1e-6) {
                        continue;
                    }
                    double c = (a.x() * b.x() + a.y() * b.y()) / (la * lb);
                    c = qBound(-1.0, c, 1.0);
                    const double angle = std::acos(c);
                    if (angle < kMinAngleRad) {
                        const double p = kMinAngleRad - angle;
                        e += kAngleWeight * p * p;
                    }
                }
            }
        }

        for (const auto& edge : edges) {
            if (!active[edge.u] || !active[edge.v]) {
                continue;
            }
            for (int v = 0; v < vertexCount; ++v) {
                if (!active[v] || v == edge.u || v == edge.v) {
                    continue;
                }
                const double d = pointToSegmentDistance(pts[v], pts[edge.u], pts[edge.v]);
                if (d < kCrowdRadius) {
                    const double p = kCrowdRadius - d;
                    e += kCrowdWeight * p * p;
                }
            }
        }

        return e;
    };

    QVector<int> order(vertexCount);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return adjacency[a].size() > adjacency[b].size(); });

    QVector<bool> placed(vertexCount, false);
    const int first = order[0];
    positions[first] = center;
    placed[first] = true;

    auto localSearch = [&](int vertex) {
        double step = 38.0;
        double bestE = energy(positions, placed);
        for (int rounds = 0; rounds < 10; ++rounds) {
            bool improved = false;
            QPointF bestPos = positions[vertex];
            for (int k = 0; k < 16; ++k) {
                const double angle = 2.0 * 3.14159265358979323846 * k / 16.0;
                QPointF cand = positions[vertex] + QPointF(step * std::cos(angle), step * std::sin(angle));
                cand = clampToCanvas(cand);
                const QPointF old = positions[vertex];
                positions[vertex] = cand;
                const double e = energy(positions, placed);
                if (e < bestE) {
                    bestE = e;
                    bestPos = cand;
                    improved = true;
                }
                positions[vertex] = old;
            }
            positions[vertex] = bestPos;
            if (!improved) {
                step *= 0.55;
            }
        }
    };

    for (int idx = 1; idx < order.size(); ++idx) {
        const int next = order[idx];

        QVector<int> anchors;
        for (const auto& ref : adjacency[next]) {
            if (placed[ref.to]) {
                anchors.push_back(ref.to);
            }
        }

        QVector<QPointF> candidates;
        if (!anchors.isEmpty()) {
            const int a = anchors[0];
            int anchorWeight = minWeight;
            for (const auto& ref : adjacency[next]) {
                if (ref.to == a) {
                    anchorWeight = edges[ref.edgeIndex].weight;
                    break;
                }
            }
            const double r = mapWeightToTargetDistance(anchorWeight);
            for (int i = 0; i < 20; ++i) {
                const double ang = 2.0 * 3.14159265358979323846 * i / 20.0;
                candidates.push_back(clampToCanvas(positions[a] + QPointF(r * std::cos(ang), r * std::sin(ang))));
            }
        }

        for (int i = 0; i < 12; ++i) {
            const double ang = 2.0 * 3.14159265358979323846 * i / 12.0;
            candidates.push_back(clampToCanvas(center + QPointF((70.0 + i * 6.0) * std::cos(ang), (70.0 + i * 6.0) * std::sin(ang))));
        }

        QPointF bestPos = center;
        double bestE = std::numeric_limits<double>::max();
        for (const QPointF& c : candidates) {
            positions[next] = c;
            placed[next] = true;
            const double e = energy(positions, placed);
            if (e < bestE) {
                bestE = e;
                bestPos = c;
            }
            placed[next] = false;
        }

        positions[next] = bestPos;
        placed[next] = true;

        localSearch(next);

        for (int iter = 0; iter < 3; ++iter) {
            for (int v = 0; v < vertexCount; ++v) {
                if (placed[v]) {
                    localSearch(v);
                }
            }
        }
    }

    QPointF gravityCenter(0, 0);
    for (int i = 0; i < vertexCount; ++i) {
        gravityCenter += positions[i];
    }
    gravityCenter /= static_cast<double>(vertexCount);
    const QPointF shift = center - gravityCenter;
    for (auto& p : positions) {
        p = clampToCanvas(p + shift);
    }

    return positions;
}
} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUi();
}

void MainWindow::setupUi() {
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    auto* splitter = new QSplitter(Qt::Horizontal, central);

    auto* left = new QWidget(splitter);
    auto* leftLayout = new QVBoxLayout(left);
    auto* form = new QFormLayout();

    m_storageType = new QComboBox(left);
    m_storageType->addItems({"邻接矩阵", "邻接表"});

    m_randomStart = new QCheckBox("随机起点", left);
    m_randomStart->setChecked(true);

    m_startVertex = new QSpinBox(left);
    m_startVertex->setRange(0, 0);
    m_startVertex->setEnabled(false);

    connect(m_randomStart, &QCheckBox::toggled, this, [this](bool checked) {
        m_startVertex->setEnabled(!checked);
    });

    m_input = new QTextEdit(left);
    m_input->setPlaceholderText("输入邻接矩阵或邻接表");
    m_input->setPlainText(defaultMatrixInput());

    connect(m_storageType,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int idx) { m_input->setPlainText(idx == 0 ? defaultMatrixInput() : defaultListInput()); });

    m_solveButton = new QPushButton("解析并求所有MST", left);
    connect(m_solveButton, &QPushButton::clicked, this, &MainWindow::onParseAndSolve);

    m_nextStepButton = new QPushButton("下一步", left);
    m_nextStepButton->setEnabled(false);
    connect(m_nextStepButton, &QPushButton::clicked, this, &MainWindow::onNextStep);

    m_playButton = new QPushButton("自动播放", left);
    m_playButton->setEnabled(false);
    connect(m_playButton, &QPushButton::clicked, this, &MainWindow::onToggleAutoPlay);

    auto* btnRow = new QHBoxLayout();
    btnRow->addWidget(m_solveButton);
    btnRow->addWidget(m_nextStepButton);
    btnRow->addWidget(m_playButton);

    form->addRow("输入格式", m_storageType);
    form->addRow("起点模式", m_randomStart);
    form->addRow("固定起点", m_startVertex);

    leftLayout->addLayout(form);
    leftLayout->addWidget(new QLabel("图输入：", left));
    leftLayout->addWidget(m_input);
    leftLayout->addLayout(btnRow);

    m_status = new QLabel("就绪", left);
    m_stepInfo = new QLabel("步骤：-", left);
    leftLayout->addWidget(m_status);
    leftLayout->addWidget(m_stepInfo);

    m_scene = new QGraphicsScene(left);
    auto* graphView = new QGraphicsView(m_scene, left);
    graphView->setMinimumSize(480, 360);
    leftLayout->addWidget(graphView);

    auto* right = new QWidget(splitter);
    right->setMinimumWidth(380);
    auto* rightLayout = new QVBoxLayout(right);

    m_prevVisibleNodeButton = new QPushButton("上一个节点", right);
    m_nextVisibleNodeButton = new QPushButton("下一个节点", right);
    m_nextVisibleLeafButton = new QPushButton("下一个叶子", right);
    connect(m_prevVisibleNodeButton, &QPushButton::clicked, this, &MainWindow::onPrevVisibleBranchNode);
    connect(m_nextVisibleNodeButton, &QPushButton::clicked, this, &MainWindow::onNextVisibleBranchNode);
    connect(m_nextVisibleLeafButton, &QPushButton::clicked, this, &MainWindow::onNextVisibleLeafNode);

    m_branchTree = new QTreeWidget(right);
    m_branchTree->setHeaderLabels({"分支树节点", "代价", "标签"});
    auto* branchHeader = m_branchTree->header();
    branchHeader->setSectionResizeMode(0, QHeaderView::Stretch);
    branchHeader->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    branchHeader->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    branchHeader->setStretchLastSection(false);
    connect(m_branchTree, &QTreeWidget::itemClicked, this, &MainWindow::onBranchTreeItemClicked);

    rightLayout->addWidget(new QLabel("分支树", right));
    rightLayout->addWidget(m_prevVisibleNodeButton);
    rightLayout->addWidget(m_nextVisibleNodeButton);
    rightLayout->addWidget(m_nextVisibleLeafButton);
    rightLayout->addWidget(m_branchTree, 1);
    updateBranchNavigationButtons();

    splitter->addWidget(left);
    splitter->addWidget(right);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({700, 480});

    root->addWidget(splitter);
    setCentralWidget(central);

    m_timer = new QTimer(this);
    m_timer->setInterval(700);
    connect(m_timer, &QTimer::timeout, this, [this]() {
        if (m_currentSolution >= m_solutions.size()) {
            m_timer->stop();
            m_playButton->setText("自动播放");
            return;
        }
        const auto& trace = m_solutions[m_currentSolution].trace;
        if (m_currentStep >= trace.size()) {
            m_timer->stop();
            m_playButton->setText("自动播放");
            return;
        }
        onNextStep();
    });

    setWindowTitle("Prim 所有最小生成树可视化");
    resize(1180, 720);
}

void MainWindow::renderBranchState(const BranchTreeNode& node) {
    resetStepStyle();

    for (int v : node.verticesInTree) {
        if (m_vertexItems.contains(v)) {
            m_vertexItems[v]->setBrush(QBrush(QColor(202, 255, 191)));
        }
    }

    for (int edgeId : node.selectedEdgeIds) {
        if (m_edgeItems.contains(edgeId)) {
            m_edgeItems[edgeId]->setPen(QPen(QColor(24, 140, 60), 2.4));
        }
    }

    for (int edgeId : node.candidateEdgeIds) {
        if (m_edgeItems.contains(edgeId)) {
            m_edgeItems[edgeId]->setPen(QPen(QColor(227, 151, 48), 2.4));
        }
    }

    if (node.viaEdgeId >= 0 && m_edgeItems.contains(node.viaEdgeId)) {
        m_edgeItems[node.viaEdgeId]->setPen(QPen(QColor(210, 35, 35), 3.2));
    }

    QString visibleInfo;
    if (m_currentVisibleBranchIndex >= 0 && m_currentVisibleBranchIndex < m_visibleBranchNodeIds.size()) {
        visibleInfo = QString("（第 %1 / %2 可见节点）").arg(m_currentVisibleBranchIndex + 1).arg(m_visibleBranchNodeIds.size());
    }
    m_stepInfo->setText(QString("分支节点 #%1：%2（当前代价=%3）%4")
                            .arg(node.id)
                            .arg(node.label)
                            .arg(formatWeight(node.currentCost))
                            .arg(visibleInfo));
}

void MainWindow::rebuildBranchTree() {
    const int previousNodeId =
        (m_currentVisibleBranchIndex >= 0 && m_currentVisibleBranchIndex < m_visibleBranchNodeIds.size())
            ? m_visibleBranchNodeIds[m_currentVisibleBranchIndex]
            : -1;

    m_branchTree->clear();
    m_branchTreeItemsById.clear();
    if (m_branchNodes.isEmpty() || m_branchRootId < 0 || m_branchRootId >= m_branchNodes.size()) {
        m_visibleBranchNodeIds.clear();
        m_visibleBranchIndexById.clear();
        m_currentVisibleBranchIndex = -1;
        updateBranchNavigationButtons();
        return;
    }

    QMap<int, QVector<int>> children;
    for (const auto& node : m_branchNodes) {
        children[node.parentId].push_back(node.id);
    }

    QMap<int, QTreeWidgetItem*> itemById;
    QHash<int, int> visibleAncestorById;
    QVector<int> stack = {m_branchRootId};
    while (!stack.isEmpty()) {
        const int id = stack.takeFirst();
        for (int child : children[id]) {
            stack.push_back(child);
        }

        const auto& node = m_branchNodes[id];
        const bool shouldShow = (node.parentId < 0) || (node.viaEdgeId >= 0) || node.isComplete;
        const int nearestVisibleParent = (node.parentId >= 0) ? visibleAncestorById.value(node.parentId, -1) : -1;
        visibleAncestorById.insert(node.id, shouldShow ? node.id : nearestVisibleParent);
        if (!shouldShow) {
            continue;
        }

        QString title;
        if (!node.vertexSelectionOrder.isEmpty()) {
            QStringList ids;
            for (int v : node.vertexSelectionOrder) {
                ids << QString::number(v);
            }
            title = ids.join("→");
        } else {
            title = "-";
        }

        auto* item = new QTreeWidgetItem({title, formatWeight(node.incrementalCost), node.label});
        item->setData(0, Qt::UserRole, node.id);
        if (nearestVisibleParent >= 0 && itemById.contains(nearestVisibleParent)) {
            itemById[nearestVisibleParent]->addChild(item);
        } else {
            m_branchTree->addTopLevelItem(item);
        }
        itemById.insert(node.id, item);
        m_branchTreeItemsById.insert(node.id, item);
    }

    m_branchTree->expandToDepth(2);
    rebuildVisibleBranchSequence(previousNodeId, m_currentVisibleBranchIndex);
}

void MainWindow::rebuildVisibleBranchSequence(int preferredNodeId, int preferredIndex) {
    m_visibleBranchNodeIds.clear();
    m_visibleBranchIndexById.clear();
    m_currentVisibleBranchIndex = -1;

    QVector<QTreeWidgetItem*> stack;
    for (int i = m_branchTree->topLevelItemCount() - 1; i >= 0; --i) {
        stack.push_back(m_branchTree->topLevelItem(i));
    }

    while (!stack.isEmpty()) {
        QTreeWidgetItem* item = stack.takeLast();
        if (!item) {
            continue;
        }

        bool ok = false;
        const int nodeId = item->data(0, Qt::UserRole).toInt(&ok);
        if (ok && nodeId >= 0 && nodeId < m_branchNodes.size()) {
            m_visibleBranchIndexById.insert(nodeId, m_visibleBranchNodeIds.size());
            m_visibleBranchNodeIds.push_back(nodeId);
        }

        for (int i = item->childCount() - 1; i >= 0; --i) {
            stack.push_back(item->child(i));
        }
    }

    if (m_visibleBranchNodeIds.isEmpty()) {
        updateBranchNavigationButtons();
        return;
    }

    int targetIndex = 0;
    if (preferredNodeId >= 0 && m_visibleBranchIndexById.contains(preferredNodeId)) {
        targetIndex = m_visibleBranchIndexById.value(preferredNodeId);
    } else if (preferredIndex >= 0 && preferredIndex < m_visibleBranchNodeIds.size()) {
        targetIndex = preferredIndex;
    } else if (m_currentVisibleBranchIndex >= 0 && m_currentVisibleBranchIndex < m_visibleBranchNodeIds.size()) {
        targetIndex = m_currentVisibleBranchIndex;
    }

    navigateToVisibleBranchIndex(targetIndex);
}

void MainWindow::navigateToVisibleBranchIndex(int index) {
    if (m_visibleBranchNodeIds.isEmpty()) {
        m_currentVisibleBranchIndex = -1;
        updateBranchNavigationButtons();
        return;
    }

    index = qBound(0, index, m_visibleBranchNodeIds.size() - 1);
    m_currentVisibleBranchIndex = index;
    const int nodeId = m_visibleBranchNodeIds[index];

    if (m_branchTreeItemsById.contains(nodeId)) {
        QTreeWidgetItem* item = m_branchTreeItemsById.value(nodeId);
        m_branchTree->setCurrentItem(item);
        m_branchTree->scrollToItem(item);
    }

    if (nodeId >= 0 && nodeId < m_branchNodes.size()) {
        renderBranchState(m_branchNodes[nodeId]);
    }
    updateBranchNavigationButtons();
}

void MainWindow::updateBranchNavigationButtons() {
    const bool hasVisible = !m_visibleBranchNodeIds.isEmpty();
    if (m_prevVisibleNodeButton) {
        m_prevVisibleNodeButton->setEnabled(hasVisible && m_currentVisibleBranchIndex > 0);
    }
    if (m_nextVisibleNodeButton) {
        m_nextVisibleNodeButton->setEnabled(hasVisible && m_currentVisibleBranchIndex >= 0
                                            && m_currentVisibleBranchIndex < m_visibleBranchNodeIds.size() - 1);
    }

    bool hasNextLeaf = false;
    if (hasVisible && m_currentVisibleBranchIndex >= 0) {
        for (int i = m_currentVisibleBranchIndex + 1; i < m_visibleBranchNodeIds.size(); ++i) {
            const int nodeId = m_visibleBranchNodeIds[i];
            if (nodeId >= 0 && nodeId < m_branchNodes.size() && m_branchTreeItemsById.contains(nodeId) && m_branchTreeItemsById.value(nodeId)->childCount() == 0) {
                hasNextLeaf = true;
                break;
            }
        }
    }
    if (m_nextVisibleLeafButton) {
        m_nextVisibleLeafButton->setEnabled(hasNextLeaf);
    }
}

void MainWindow::onBranchTreeItemClicked(QTreeWidgetItem* item, int /*column*/) {
    if (!item) {
        return;
    }
    bool ok = false;
    const int nodeId = item->data(0, Qt::UserRole).toInt(&ok);
    if (!ok || !m_visibleBranchIndexById.contains(nodeId)) {
        return;
    }
    navigateToVisibleBranchIndex(m_visibleBranchIndexById.value(nodeId));
}


void MainWindow::onPrevVisibleBranchNode() {
    if (m_currentVisibleBranchIndex <= 0) {
        return;
    }
    navigateToVisibleBranchIndex(m_currentVisibleBranchIndex - 1);
}

void MainWindow::onNextVisibleBranchNode() {
    if (m_currentVisibleBranchIndex < 0 || m_currentVisibleBranchIndex >= m_visibleBranchNodeIds.size() - 1) {
        return;
    }
    navigateToVisibleBranchIndex(m_currentVisibleBranchIndex + 1);
}

void MainWindow::onNextVisibleLeafNode() {
    if (m_currentVisibleBranchIndex < 0) {
        return;
    }
    for (int i = m_currentVisibleBranchIndex + 1; i < m_visibleBranchNodeIds.size(); ++i) {
        const int nodeId = m_visibleBranchNodeIds[i];
        if (nodeId >= 0 && nodeId < m_branchNodes.size() && m_branchTreeItemsById.contains(nodeId) && m_branchTreeItemsById.value(nodeId)->childCount() == 0) {
            navigateToVisibleBranchIndex(i);
            return;
        }
    }
}

bool MainWindow::parseAdjMatrix(const QString& text, AdjMatrixGraph& graph, QString& error) const {
    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty()) {
        error = "邻接矩阵输入为空";
        return false;
    }

    QVector<QVector<double>> matrix;
    for (const auto& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList tokens = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        QVector<double> row;
        for (const QString& token : tokens) {
            double value = -1.0;
            bool hasEdge = false;
            if (!parseWeightToken(token, value, hasEdge)) {
                error = QString("无法解析矩阵元素: %1").arg(token);
                return false;
            }
            row.push_back(hasEdge ? value : -1.0);
        }
        matrix.push_back(row);
    }

    const int n = matrix.size();
    for (const auto& row : matrix) {
        if (row.size() != n) {
            error = "邻接矩阵必须是 n x n";
            return false;
        }
    }

    graph.reset(n);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const double a = matrix[i][j];
            const double b = matrix[j][i];
            if (!qFuzzyCompare(1.0 + a, 1.0 + b)) {
                error = QString("矩阵不是对称矩阵: (%1,%2) 与 (%2,%1) 不一致").arg(i).arg(j);
                return false;
            }
            if (a > 0) {
                graph.addEdge(i, j, a);
            }
        }
    }
    return true;
}

bool MainWindow::parseAdjList(const QString& text, AdjListGraph& graph, QString& error) const {
    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty()) {
        error = "邻接表输入为空";
        return false;
    }

    struct SeenEdgeInfo {
        double weight;
        QString rawLine;
    };

    QHash<quint64, SeenEdgeInfo> seenEdges;
    int maxV = -1;

    for (const auto& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList tokens = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (tokens.size() < 1 || (tokens.size() - 1) % 2 != 0) {
            error = QString("邻接表每行应为: u (v w)...，错误行: %1").arg(line);
            return false;
        }

        bool okU = false;
        const int u = tokens[0].toInt(&okU);
        if (!okU || u < 0) {
            error = QString("邻接表含非法值，错误行: %1").arg(line);
            return false;
        }

        maxV = qMax(maxV, u);
        for (int i = 1; i < tokens.size(); i += 2) {
            bool okV = false;
            bool okW = false;
            const int v = tokens[i].toInt(&okV);
            const double w = tokens[i + 1].toDouble(&okW);
            if (!okV || !okW || v < 0 || w <= 0 || u == v) {
                error = QString("邻接表含非法值，错误行: %1").arg(line);
                return false;
            }

            const int a = qMin(u, v);
            const int b = qMax(u, v);
            const quint64 edgeKey = (static_cast<quint64>(a) << 32) | static_cast<quint32>(b);
            const auto existing = seenEdges.constFind(edgeKey);
            if (existing != seenEdges.constEnd()) {
                if (!qFuzzyCompare(1.0 + existing->weight, 1.0 + w)) {
                    error = QString("检测到重复边且权重冲突，已有行: %1；冲突行: %2")
                                .arg(existing->rawLine, line);
                    return false;
                }
                continue;
            }

            seenEdges.insert(edgeKey, {w, line});
            maxV = qMax(maxV, v);
        }
    }

    if (maxV < 0 || seenEdges.isEmpty()) {
        error = "邻接表未解析出有效边";
        return false;
    }

    graph.reset(maxV + 1);
    for (auto it = seenEdges.cbegin(); it != seenEdges.cend(); ++it) {
        const int u = static_cast<int>(it.key() >> 32);
        const int v = static_cast<int>(it.key() & 0xFFFFFFFFu);
        graph.addEdge(u, v, it.value().weight);
    }
    return true;
}

bool MainWindow::buildGraphFromInput(QString& error) {
    if (m_storageType->currentIndex() == 0) {
        auto graph = std::make_unique<AdjMatrixGraph>();
        if (!parseAdjMatrix(m_input->toPlainText(), *graph, error)) {
            return false;
        }
        m_graph = std::move(graph);
    } else {
        auto graph = std::make_unique<AdjListGraph>();
        if (!parseAdjList(m_input->toPlainText(), *graph, error)) {
            return false;
        }
        m_graph = std::move(graph);
    }
    return true;
}

void MainWindow::drawGraph() {
    m_scene->clear();
    m_edgeItems.clear();
    m_vertexItems.clear();

    if (!m_graph || m_graph->vertexCount() == 0) {
        return;
    }

    const int n = m_graph->vertexCount();
    const QPointF center(250, 190);
    const auto& edges = m_graph->edges();
    const QVector<QPointF> vertexPos = buildWeightAwareLayout(edges, n, center);

    for (const auto& e : edges) {
        const QPointF a = vertexPos[e.u];
        const QPointF b = vertexPos[e.v];
        auto* line = m_scene->addLine(QLineF(a, b), QPen(Qt::gray, 1.2));
        m_edgeItems.insert(e.id, line);

        const QPointF mid = (a + b) / 2.0;
        auto* txt = m_scene->addSimpleText(formatWeight(e.weight));
        txt->setBrush(QBrush(Qt::darkBlue));
        txt->setPos(mid.x() + 3, mid.y() + 3);
    }

    constexpr double r = 16.0;
    for (int i = 0; i < n; ++i) {
        const QPointF p = vertexPos[i];
        auto* circle = m_scene->addEllipse(p.x() - r, p.y() - r, 2 * r, 2 * r, QPen(Qt::black), QBrush(Qt::white));
        m_vertexItems.insert(i, circle);

        auto* txt = m_scene->addSimpleText(QString::number(i));
        txt->setPos(p.x() - 5, p.y() - 10);
    }
}

void MainWindow::resetStepStyle() {
    for (auto* item : m_edgeItems) {
        item->setPen(QPen(Qt::gray, 1.2));
    }
    for (auto* item : m_vertexItems) {
        item->setBrush(QBrush(Qt::white));
    }
}

QVector<int> MainWindow::selectedEdgesUntilStep(int stepIndex) const {
    QVector<int> selected;
    if (m_currentSolution >= m_solutions.size()) {
        return selected;
    }
    const auto& trace = m_solutions[m_currentSolution].trace;
    for (int i = 0; i <= stepIndex && i < trace.size(); ++i) {
        if (trace[i].type == PrimStep::Type::ChooseEdge && trace[i].chosenEdgeId >= 0) {
            selected.push_back(trace[i].chosenEdgeId);
        }
        if (trace[i].type == PrimStep::Type::CompleteMST) {
            selected = trace[i].relatedEdgeIds;
        }
    }
    return selected;
}

void MainWindow::renderCurrentStep() {
    if (m_currentSolution >= m_solutions.size()) {
        return;
    }

    const auto& trace = m_solutions[m_currentSolution].trace;
    if (m_currentStep < 0 || m_currentStep >= trace.size()) {
        return;
    }

    resetStepStyle();
    const PrimStep& step = trace[m_currentStep];

    for (int v : step.verticesInTree) {
        if (m_vertexItems.contains(v)) {
            m_vertexItems[v]->setBrush(QBrush(QColor(202, 255, 191)));
        }
    }

    for (int edgeId : selectedEdgesUntilStep(m_currentStep)) {
        if (m_edgeItems.contains(edgeId)) {
            m_edgeItems[edgeId]->setPen(QPen(QColor(24, 140, 60), 2.4));
        }
    }

    if (step.type == PrimStep::Type::CandidateMinEdges) {
        for (int edgeId : step.relatedEdgeIds) {
            if (m_edgeItems.contains(edgeId)) {
                m_edgeItems[edgeId]->setPen(QPen(QColor(227, 151, 48), 2.4));
            }
        }
    }

    if (step.chosenEdgeId >= 0 && m_edgeItems.contains(step.chosenEdgeId)) {
        m_edgeItems[step.chosenEdgeId]->setPen(QPen(QColor(210, 35, 35), 3.2));
    }

    m_stepInfo->setText(
        QString("步骤 %1/%2：%3（当前代价=%4）")
            .arg(m_currentStep + 1)
            .arg(trace.size())
            .arg(step.note)
            .arg(formatWeight(step.currentCost)));
}

void MainWindow::onParseAndSolve() {
    QString error;
    m_solutions.clear();
    m_branchNodes.clear();
    m_branchRootId = -1;
    m_branchTree->clear();
    m_branchTreeItemsById.clear();
    m_visibleBranchNodeIds.clear();
    m_visibleBranchIndexById.clear();
    m_currentVisibleBranchIndex = -1;
    updateBranchNavigationButtons();
    m_currentStep = 0;
    m_hasRenderedCurrentStep = false;
    m_currentSolution = 0;
    if (m_timer->isActive()) {
        m_timer->stop();
        m_playButton->setText("自动播放");
    }

    if (!buildGraphFromInput(error)) {
        m_scene->clear();
        m_status->setText(QString("解析失败：%1").arg(error));
        m_nextStepButton->setEnabled(false);
        m_playButton->setEnabled(false);
        return;
    }

    drawGraph();

    const int n = m_graph->vertexCount();
    m_startVertex->setRange(0, qMax(0, n - 1));
    if (n == 0) {
        m_status->setText("图为空");
        m_nextStepButton->setEnabled(false);
        m_playButton->setEnabled(false);
        return;
    }

    const int start = m_randomStart->isChecked() ? QRandomGenerator::global()->bounded(n) : m_startVertex->value();

    PrimAllMSTSolver solver;
    SolverLimits limits;
    limits.maxSolutions = 200;
    limits.maxExpandedStates = 100000;
    const SolveResult solveResult = solver.solveAll(*m_graph, start, limits);
    m_solutions = solveResult.solutions;
    m_branchNodes = solveResult.branchNodes;
    m_branchRootId = solveResult.branchRootId;
    rebuildBranchTree();

    if (m_solutions.isEmpty()) {
        m_status->setText("未找到 MST（图可能不连通）");
        m_nextStepButton->setEnabled(false);
        m_playButton->setEnabled(false);
        m_stepInfo->setText("步骤：-");
        return;
    }

    if (solveResult.stats.isTruncated()) {
        m_status->setText("结果已截断，请缩小图规模（仍可查看当前结果）");
    } else {
        m_status->setText("已求解，可点击“下一步”查看 Prim 过程");
    }
    m_nextStepButton->setEnabled(true);
    m_nextStepButton->setText("下一步");
    m_playButton->setEnabled(true);
    m_currentStep = 0;
    m_hasRenderedCurrentStep = false;
}

void MainWindow::onNextStep() {
    if (m_currentSolution >= m_solutions.size()) {
        return;
    }

    const int traceSize = m_solutions[m_currentSolution].trace.size();
    if (traceSize == 0) {
        if (m_timer->isActive()) {
            m_timer->stop();
            m_playButton->setText("自动播放");
        }
        m_nextStepButton->setText("已完成");
        m_status->setText("无可展示步骤（trace 为空）");
        m_stepInfo->setText("步骤：-");
        return;
    }

    // 首次进入：先渲染当前步，不推进索引。
    if (!m_hasRenderedCurrentStep) {
        renderCurrentStep();
        m_hasRenderedCurrentStep = true;
        if (traceSize == 1) {
            if (m_timer->isActive()) {
                m_timer->stop();
                m_playButton->setText("自动播放");
            }
            m_nextStepButton->setText("已完成");
            m_status->setText("动画播放完成（当前展示第1棵 MST 的求解过程）");
        } else {
            m_nextStepButton->setText("下一步");
        }
        return;
    }

    // 常规点击：先判断是否可前进，可前进则推进后渲染。
    if (m_currentStep < traceSize - 1) {
        ++m_currentStep;
        renderCurrentStep();
        if (m_currentStep == traceSize - 1) {
            if (m_timer->isActive()) {
                m_timer->stop();
                m_playButton->setText("自动播放");
            }
            m_nextStepButton->setText("已完成");
            m_status->setText("动画播放完成（当前展示第1棵 MST 的求解过程）");
        }
        return;
    }

    if (m_timer->isActive()) {
        m_timer->stop();
        m_playButton->setText("自动播放");
    }
    m_nextStepButton->setText("已完成");
    m_status->setText("动画播放完成（当前展示第1棵 MST 的求解过程）");
}

void MainWindow::onToggleAutoPlay() {
    if (!m_playButton->isEnabled()) {
        return;
    }
    if (m_timer->isActive()) {
        m_timer->stop();
        m_playButton->setText("自动播放");
    } else {
        m_timer->start();
        m_playButton->setText("停止播放");
    }
}
