#pragma once

#include "graph.h"
#include "prim_all_mst_solver.h"

#include <QMainWindow>

class QComboBox;
class QGraphicsScene;
class QLabel;
class QPushButton;
class QTextEdit;
class QSpinBox;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onGenerateAndSolve();

private:
    void setupUi();
    QString mstToString(const MSTSolution& mst) const;

    QSpinBox* m_vertexCount = nullptr;
    QSpinBox* m_minWeight = nullptr;
    QSpinBox* m_maxWeight = nullptr;
    QComboBox* m_storageType = nullptr;
    QPushButton* m_runButton = nullptr;
    QTextEdit* m_output = nullptr;
    QLabel* m_status = nullptr;
    QGraphicsScene* m_scene = nullptr;
};
