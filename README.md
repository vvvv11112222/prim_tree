# Prim Tree Demo (Qt/C++)

用于演示“无向网 + 随机起点 + Prim思想求所有最小生成树（MST）”的 Qt Widgets 示例项目。

## 功能

- 随机生成连通无向网（先随机树保证连通，再按密度补边）。
- 支持两种存储结构：
  - 邻接矩阵 `AdjMatrixGraph`
  - 邻接表 `AdjListGraph`
- 使用 Prim 的“割边最小权”思想，并在最小权候选边上进行分支搜索，枚举所有 MST。
- 图形界面可调整参数并展示求解结果。

## 算法说明（所有 MST）

普通 Prim 在每步只选一条边，通常得到一棵树。
本项目做法是在每步中：

1. 计算当前割边集合（`U` 与 `V-U` 之间的边）
2. 找到最小权值 `w_min`
3. 对所有 `weight == w_min` 的候选边分别递归分支
4. 当边数达到 `n-1` 时得到一棵生成树
5. 通过边集 key 去重，保留全部最优代价树

## 构建

```bash
cmake -S . -B build
cmake --build build
./build/prim_tree_demo
```

> 需要 Qt6（Core + Widgets）以及 C++17 编译器。

## 目录

- `include/graph.h`：图抽象、邻接矩阵/邻接表、随机图工厂
- `include/prim_all_mst_solver.h`：Prim 多分支枚举器数据结构
- `src/prim_all_mst_solver.cpp`：求所有 MST 的核心实现
- `src/mainwindow.cpp`：Qt 界面与交互逻辑
