#include "mainwindow.h"

#include <QComboBox>
#include <QFormLayout>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QSpinBox>
#include <QSplitter>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>

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

    m_vertexCount = new QSpinBox(left);
    m_vertexCount->setRange(3, 30);
    m_vertexCount->setValue(8);

    m_minWeight = new QSpinBox(left);
    m_minWeight->setRange(1, 99);
    m_minWeight->setValue(1);

    m_maxWeight = new QSpinBox(left);
    m_maxWeight->setRange(1, 99);
    m_maxWeight->setValue(9);

    m_storageType = new QComboBox(left);
    m_storageType->addItems({"邻接矩阵", "邻接表"});

    m_runButton = new QPushButton("生成随机图并求所有MST", left);
    connect(m_runButton, &QPushButton::clicked, this, &MainWindow::onGenerateAndSolve);

    form->addRow("顶点数", m_vertexCount);
    form->addRow("最小权", m_minWeight);
    form->addRow("最大权", m_maxWeight);
    form->addRow("存储结构", m_storageType);

    leftLayout->addLayout(form);
    leftLayout->addWidget(m_runButton);

    m_status = new QLabel("就绪", left);
    leftLayout->addWidget(m_status);

    m_scene = new QGraphicsScene(left);
    auto* graphView = new QGraphicsView(m_scene, left);
    graphView->setMinimumSize(420, 320);
    leftLayout->addWidget(graphView);

    auto* right = new QWidget(splitter);
    auto* rightLayout = new QVBoxLayout(right);
    m_output = new QTextEdit(right);
    m_output->setReadOnly(true);
    rightLayout->addWidget(m_output);

    splitter->addWidget(left);
    splitter->addWidget(right);
    splitter->setStretchFactor(1, 1);

    root->addWidget(splitter);
    setCentralWidget(central);
    setWindowTitle("Prim 全部最小生成树演示");
    resize(980, 620);
}

QString MainWindow::mstToString(const MSTSolution& mst) const {
    QStringList edgeTexts;
    for (int id : mst.edgeIds) {
        edgeTexts << QString::number(id);
    }
    return QString("cost=%1, edges=[%2]").arg(mst.totalCost).arg(edgeTexts.join(", "));
}

void MainWindow::onGenerateAndSolve() {
    const int n = m_vertexCount->value();
    const int minW = m_minWeight->value();
    const int maxW = m_maxWeight->value();
    if (minW > maxW) {
        m_status->setText("参数错误：最小权不能大于最大权");
        return;
    }

    const int start = QRandomGenerator::global()->bounded(n);
    PrimAllMSTSolver solver;

    m_output->clear();
    m_scene->clear();

    QVector<MSTSolution> solutions;
    if (m_storageType->currentIndex() == 0) {
        auto g = GraphFactory::randomConnectedMatrixGraph(n, 0.4, minW, maxW);
        solutions = solver.solveAll(g, start);
        m_output->append(QString("使用邻接矩阵, 顶点=%1, 边=%2, 起点=%3")
                             .arg(g.vertexCount())
                             .arg(g.edgeCount())
                             .arg(start));
    } else {
        auto g = GraphFactory::randomConnectedListGraph(n, 0.4, minW, maxW);
        solutions = solver.solveAll(g, start);
        m_output->append(QString("使用邻接表, 顶点=%1, 边=%2, 起点=%3")
                             .arg(g.vertexCount())
                             .arg(g.edgeCount())
                             .arg(start));
    }

    m_output->append(QString("找到最小生成树数量: %1").arg(solutions.size()));
    for (int i = 0; i < solutions.size(); ++i) {
        m_output->append(QString("MST #%1 -> %2").arg(i + 1).arg(mstToString(solutions[i])));
    }

    m_status->setText(QString("完成：起点 %1，MST 数 %2").arg(start).arg(solutions.size()));
}
