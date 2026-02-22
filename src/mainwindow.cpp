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
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSpinBox>
#include <QStringList>
#include <QSplitter>
#include <QTextEdit>
#include <QTimer>
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
    return "0 1 2\n"
           "0 2 3\n"
           "1 2 1\n"
           "1 3 4\n"
           "2 3 5";
}

bool parseWeightToken(const QString& token, int& value, bool& hasEdge) {
    const QString t = token.trimmed().toUpper();
    if (t == "INF" || t == "X" || t == "-1") {
        hasEdge = false;
        value = -1;
        return true;
    }
    bool ok = false;
    const int w = t.toInt(&ok);
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

    int minWeight = std::numeric_limits<int>::max();
    int maxWeight = std::numeric_limits<int>::min();
    for (const auto& e : edges) {
        minWeight = std::min(minWeight, e.weight);
        maxWeight = std::max(maxWeight, e.weight);
    }

    const auto mapWeightToTargetDistance = [&](int weight) {
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
    auto* rightLayout = new QVBoxLayout(right);
    m_output = new QTextEdit(right);
    m_output->setReadOnly(true);
    rightLayout->addWidget(m_output);

    splitter->addWidget(left);
    splitter->addWidget(right);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

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

bool MainWindow::parseAdjMatrix(const QString& text, AdjMatrixGraph& graph, QString& error) const {
    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty()) {
        error = "邻接矩阵输入为空";
        return false;
    }

    QVector<QVector<int>> matrix;
    for (const auto& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList tokens = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        QVector<int> row;
        for (const QString& token : tokens) {
            int value = -1;
            bool hasEdge = false;
            if (!parseWeightToken(token, value, hasEdge)) {
                error = QString("无法解析矩阵元素: %1").arg(token);
                return false;
            }
            row.push_back(hasEdge ? value : -1);
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
            const int a = matrix[i][j];
            const int b = matrix[j][i];
            if (a != b) {
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

    struct Row {
        int u;
        int v;
        int w;
    };

    QVector<Row> rows;
    int maxV = -1;

    for (const auto& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList tokens = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (tokens.size() != 3) {
            error = QString("邻接表每行应为: u v w，错误行: %1").arg(line);
            return false;
        }

        bool okU = false;
        bool okV = false;
        bool okW = false;
        const int u = tokens[0].toInt(&okU);
        const int v = tokens[1].toInt(&okV);
        const int w = tokens[2].toInt(&okW);
        if (!okU || !okV || !okW || u < 0 || v < 0 || w <= 0) {
            error = QString("邻接表含非法值，错误行: %1").arg(line);
            return false;
        }
        rows.push_back({u, v, w});
        maxV = qMax(maxV, qMax(u, v));
    }

    if (maxV < 0) {
        error = "邻接表未解析出有效边";
        return false;
    }

    graph.reset(maxV + 1);
    for (const auto& row : rows) {
        graph.addEdge(row.u, row.v, row.w);
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

QString MainWindow::mstToString(const MSTSolution& mst) const {
    QStringList edgeTexts;
    for (int id : mst.edgeIds) {
        edgeTexts << QString::number(id);
    }
    return QString("cost=%1, edgeIds=[%2]").arg(mst.totalCost).arg(edgeTexts.join(", "));
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
    const QVector<QPointF> vertexPos = buildWeightAwareLayout(m_graph->edges(), n, center);

    for (const auto& e : m_graph->edges()) {
        const QPointF a = vertexPos[e.u];
        const QPointF b = vertexPos[e.v];
        auto* line = m_scene->addLine(QLineF(a, b), QPen(Qt::gray, 1.2));
        m_edgeItems.insert(e.id, line);

        const QPointF mid = (a + b) / 2.0;
        auto* txt = m_scene->addSimpleText(QString::number(e.weight));
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
        QString("步骤 %1/%2：%3（当前代价=%4）").arg(m_currentStep + 1).arg(trace.size()).arg(step.note).arg(step.currentCost));
}

void MainWindow::onParseAndSolve() {
    QString error;
    m_output->clear();
    m_solutions.clear();
    m_currentStep = 0;
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

    m_output->append(
        QString("输入解析成功：顶点=%1, 边=%2, 起点=%3").arg(m_graph->vertexCount()).arg(m_graph->edgeCount()).arg(start));
    m_output->append(QString("最小生成树数量: %1").arg(m_solutions.size()));

    for (int i = 0; i < m_solutions.size(); ++i) {
        m_output->append(QString("MST #%1 -> %2").arg(i + 1).arg(mstToString(m_solutions[i])));
    }

    m_output->append(QString("搜索扩展节点数: %1").arg(solveResult.stats.expandedStates));
    if (solveResult.stats.isTruncated()) {
        QStringList reasons;
        if (solveResult.stats.truncatedBySolutionLimit) {
            reasons << QString("达到结果数量上限(%1)").arg(limits.maxSolutions);
        }
        if (solveResult.stats.truncatedByStateLimit) {
            reasons << QString("达到搜索节点上限(%1)").arg(limits.maxExpandedStates);
        }
        m_output->append(QString("结果已截断：%1。请缩小图规模或提高上限。").arg(reasons.join("，")));
    }

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
    m_playButton->setEnabled(true);
    m_currentStep = 0;
    renderCurrentStep();
}

void MainWindow::onNextStep() {
    if (m_currentSolution >= m_solutions.size()) {
        return;
    }

    const int traceSize = m_solutions[m_currentSolution].trace.size();
    if (traceSize == 0) {
        return;
    }

    renderCurrentStep();

    if (m_currentStep < traceSize - 1) {
        ++m_currentStep;
    } else {
        if (m_timer->isActive()) {
            m_timer->stop();
            m_playButton->setText("自动播放");
        }
        m_status->setText("动画播放完成（当前展示第1棵 MST 的求解过程）");
    }
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
