#pragma once

#include "graph.h"
#include "prim_all_mst_solver.h"

#include <QMainWindow>

#include <QMap>
#include <memory>

class QCheckBox;
class QComboBox;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsScene;
class QLabel;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QTimer;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onParseAndSolve();
    void onNextStep();
    void onToggleAutoPlay();

private:
    void setupUi();
    QString mstToString(const MSTSolution& mst) const;
    void drawGraph();
    void renderCurrentStep();
    void resetStepStyle();
    QVector<int> selectedEdgesUntilStep(int stepIndex) const;

    bool buildGraphFromInput(QString& error);
    bool parseAdjMatrix(const QString& text, AdjMatrixGraph& graph, QString& error) const;
    bool parseAdjList(const QString& text, AdjListGraph& graph, QString& error) const;

    QComboBox* m_storageType = nullptr;
    QCheckBox* m_randomStart = nullptr;
    QSpinBox* m_startVertex = nullptr;
    QPushButton* m_solveButton = nullptr;
    QPushButton* m_nextStepButton = nullptr;
    QPushButton* m_playButton = nullptr;

    QTextEdit* m_input = nullptr;
    QTextEdit* m_output = nullptr;
    QLabel* m_status = nullptr;
    QLabel* m_stepInfo = nullptr;
    QGraphicsScene* m_scene = nullptr;
    QTimer* m_timer = nullptr;

    std::unique_ptr<IGraphStorage> m_graph;
    QVector<MSTSolution> m_solutions;
    int m_currentSolution = 0;
    // 始终表示“当前正在显示的步骤索引”。
    int m_currentStep = 0;
    bool m_hasRenderedCurrentStep = false;

    QMap<int, QGraphicsLineItem*> m_edgeItems;
    QMap<int, QGraphicsEllipseItem*> m_vertexItems;
};
