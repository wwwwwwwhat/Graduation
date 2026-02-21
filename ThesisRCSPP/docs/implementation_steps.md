# ThesisRCSPP 初始架构设计与模块拆分

## 0. 为什么要拆分成 8 个模块？

RCSPP 的求解流程可以分解为一条清晰的调用链：

```
输入(NavMesh/图) → 图表示 → 启发式计算 → 支配性检查 → 搜索 → 归一化预处理 → 输出(路径)
```

按照**单一职责原则**，每个环节独立成一个模块。这样做的好处：
- 每个模块可以独立测试、替换（比如 Pulse 被 ERCA* 替代做归一化求解，只需改 Normalizer，不动其他模块）
- 参考实现 `public_erca` 也是类似的模块化结构，方便对照
- 论文的每一章可以对应一个模块

---

## 第一步：创建 Graph + CostVector（最底层，所有模块的基础）

### 创建文件
```
include/graph/Graph.h      → 图抽象基类 + CostVector 定义
src/graph/Graph.cpp         → CostVector 运算符实现
include/graph/Roadmap.h     → 邻接表图实现
src/graph/Roadmap.cpp       → 节点/边增删 + DIMACS文件加载
```

### 为什么这样拆？

**Graph.h 定义抽象接口，Roadmap.h 是具体实现**——经典的接口-实现分离。

原因：论文中图的来源有两种：
1. 随机生成的网格图 → 直接用 Roadmap
2. NavMesh 转换的图 → 也用 Roadmap，但通过 NavMeshConverter 填充

把 `Graph` 定义为抽象类（虚函数 `GetSuccs/GetPreds/GetCost`），所有算法只依赖 `Graph*` 接口。这样 Dijkstra、ERCA*、Normalizer 都不需要知道图的具体存储方式。

**CostVector 放在 Graph.h 里**——因为它是图边的代价类型，也是搜索标签的核心字段，被所有模块使用。定义在最底层避免循环依赖。

```
CostVector 示例: [350, 120, 80]
                   │     │    │
                   │     │    └── 资源2（行动点）
                   │     └─── 资源1（体力）
                   └──── 主代价（距离）
```

---

## 第二步：创建 AVLTree + ParetoFrontier（ERCA* 的核心加速结构）

### 创建文件
```
include/frontier/AVLTree.h       → AVL 平衡二叉搜索树
src/frontier/AVLTree.cpp         → 插入/删除/旋转
include/frontier/ParetoFrontier.h → 基于 AVL 的 Pareto 前沿
src/frontier/ParetoFrontier.cpp   → Check(支配检查) + Update(更新前沿)
```

### 为什么这样拆？

**AVLTree 和 ParetoFrontier 分开**——AVL 树是通用数据结构，Pareto 前沿是特定于 RCSPP 的逻辑。

参考论文 ERCA* 的实现：每个节点维护一个 Pareto 前沿，到达该节点的新标签需要检查是否被已有标签"支配"（每个维度都不优）。如果被支配就剪枝。

用 AVL 树（而不是 std::set）是因为参考实现 `public_erca` 就用了自定义 BBST，方便实现 Pareto 支配性检查的特殊遍历逻辑。把 AVL 作为底层容器，ParetoFrontier 在其上封装"支配性检查"语义。

```
节点 v 的 Pareto 前沿:
  Label A: g = [100, 50, 30]   ← 代价低但资源高
  Label B: g = [150, 20, 10]   ← 代价高但资源低
  → A 和 B 互不支配，都保留

  新 Label C: g = [160, 60, 40]
  → C 被 A 支配（每维都 ≥ A），剪掉
```

---

## 第三步：创建 Dijkstra（启发式函数提供者）

### 创建文件
```
include/algorithm/Dijkstra.h → 单维度反向 Dijkstra
src/algorithm/Dijkstra.cpp   → 搜索 + 距离查询 + 路径重建
```

### 为什么独立成一个模块？

ERCA* 需要启发式函数 h(v)，对每个代价维度分别运行一次反向 Dijkstra 得到。

**反向 Dijkstra 从 target 开始搜索**（而不是从 source），因为 h(v) 需要的是"v 到 target 的最小代价"，反向搜索一次就得到所有节点的 h 值。

这个模块被两处调用：
1. **ERCA* 内部**：`InitHeuristic(target)` 对每个维度运行一次，提供 h(v)
2. **实验程序**：计算资源下界 LB（反向 Dijkstra 按资源维度）和代价最优路径（按主代价维度）

```
维度0 反向Dijkstra → h₀(v) = v到target的最小距离
维度1 反向Dijkstra → h₁(v) = v到target的最小体力
维度2 反向Dijkstra → h₂(v) = v到target的最小行动点
合并 → h(v) = [h₀(v), h₁(v), h₂(v)]
```

---

## 第四步：创建 ERCA*（核心搜索算法）

### 创建文件
```
include/algorithm/ERCA.h → SearchResult 结构体 + ERCA 类声明
src/algorithm/ERCA.cpp   → 主搜索循环
```

### 为什么是核心？

ERCA* 是整个论文的基线算法，所有优化策略都是在它上面叠加的：
- **纯 ERCA***（w=1.0）= 精确求解器 = 实验基线
- **ERCA* + 膨胀**（w=1.5）= 近似加速
- **ERCA* + 上界**（从归一化获得）= 剪枝加速
- **归一化图上的 ERCA*** = Normalizer 内部也调用 ERCA*

```
ERCA* 主循环:
  while Open 不空:
    取 f 值最小的标签 L
    if L 被 Pareto 前沿支配 → 跳过          (frontierCheck)
    if L 资源超限 or 主代价超上界 → 跳过     (resourceCheck)
    更新 Pareto 前沿
    if L 到达 target → 结束
    展开 L 的后继节点
```

**SearchResult** 结构体聚合了所有需要的输出：路径、代价、展开数、剪枝计数、耗时。后续实验直接从这里取数据。

---

## 第五步：创建 Pulse（初始上界求解方案）

### 创建文件
```
include/algorithm/Pulse.h → Pulse DFS 声明
src/algorithm/Pulse.cpp   → 深度优先搜索实现
```

### 为什么独立？

论文最初的设计：归一化后在单资源图上用 Pulse DFS 快速找一条可行路径作为上界。Pulse 是一种基于 DFS 的搜索策略，参考了 Lozano & Medaglia (2013) 的 Pulse 算法。

**后来被替代的原因**：Pulse DFS 在网格图上搜索空间指数爆炸（大量分支），所有测试都超时。改用 ERCA* 在归一化图上求解（`ComputeUpperBoundERCA`），更可靠。

保留 Pulse.cpp 是因为：
- 论文需要说明"尝试了什么，为什么不行"
- 代码仍可编译，作为对比参考

---

## 第六步：创建 Normalizer（论文的核心创新）

### 创建文件
```
include/algorithm/Normalizer.h → NormResult + Normalizer 类
src/algorithm/Normalizer.cpp   → 归一化系数计算 + 图构建 + 迭代权重调整
```

### 为什么最后创建？

Normalizer 依赖前面所有模块：
- 依赖 `Graph/Roadmap`：读取原图、构建归一化图
- 依赖 `ERCA*`：在归一化图上搜索
- 依赖 `Pulse`（初期）：最初用 Pulse 做 DFS 搜索

```
Normalizer 工作流:

原始图 (3维: 距离+体力+行动点)
    │
    ├── 采样所有边，计算分位数归一化系数
    │   coeffs = [avg(P20,P40,P60,P80) for each resource]
    │
    ├── 构建归一化图 (2维: 距离 + 加权资源和)
    │   normCost[r] = Σ(weight[r] * original[r] / coeff[r])
    │
    ├── 在归一化图上运行 ERCA* → 得到路径
    │
    ├── 验证路径在原图是否可行
    │   ├── 可行 → 返回上界（结束）
    │   └── 不可行 → 调整权重
    │       ├── 超限的资源: weight *= 2.0
    │       └── 宽裕的资源: weight *= 0.5
    │       └── 重试（最多5次）
    │
    └── 返回 NormResult {feasible, upperBound, path, iterations}
```

---

## 第七步：创建 NavMeshConverter（连接 RecastNavigation）

### 创建文件
```
include/graph/NavMeshConverter.h → 转换器声明
src/graph/NavMeshConverter.cpp   → NavMesh多边形 → Roadmap节点/边
```

### 为什么可选编译？

NavMeshConverter 依赖外部库 RecastNavigation（第三方），不是所有环境都能编译。通过 CMake 的 `BUILD_NAVMESH` 开关控制：
- 关闭（默认）：只编译纯算法部分，用随机图或 DIMACS 文件测试
- 打开：编译 NavMesh 转换器，可以读取 RecastNavigation 生成的导航网格

---

## 第八步：创建 Benchmark 框架 + 实验程序

### 创建文件
```
include/benchmark/Benchmark.h → 基准测试框架
src/benchmark/Benchmark.cpp   → 图生成 + 多方法对比 + 计时

app/main.cpp                  → 最小正确性验证（6节点手工图）
app/run_benchmark.cpp         → 随机图基准测试
app/navmesh_benchmark.cpp     → NavMesh 数据测试
app/pruning_analysis.cpp      → 剪枝效果分析（4种方法对比）
app/ablation_study.cpp        → 消融实验（6种配置）
app/expanded_benchmark.cpp    → 扩展实验（多种子+逐迭代分析）
app/smart_benchmark.cpp       → 智能阈值（α难度系数）
```

### 实验程序的演进顺序

```
main.cpp           "算法跑得对吗？"
    ↓
run_benchmark.cpp   "在不同规模的图上表现如何？"
    ↓
navmesh_benchmark   "在真实NavMesh数据上呢？"
    ↓
pruning_analysis    "每种剪枝手段各贡献了多少？"
    ↓
ablation_study      "去掉某个组件后性能下降多少？"
    ↓
expanded_benchmark  "多组数据取平均，归一化的逐次迭代各贡献多少？"
    ↓
smart_benchmark     "用更科学的难度参数化(α)测试"
```

---

## 模块依赖关系图

```
                    Graph.h (CostVector, Label, 图接口)
                   ╱       ╲
            Roadmap          NavMeshConverter [可选]
           ╱   │   ╲
    AVLTree    │    Dijkstra
       │       │       │
  ParetoFrontier       │
          ╲    │      ╱
           ╲   │     ╱
            ERCA* ←─┘
           ╱    ╲
       Pulse    Normalizer
      (弃用)    (调用ERCA*)
                    │
          ┌─────────┼──────────┬────────────┐
          │         │          │            │
     pruning   ablation   expanded     smart
     analysis    study    benchmark   benchmark
```

---

## 8 个模块一览表

| # | 模块 | 文件 | 职责 | 被谁调用 |
|---|------|------|------|----------|
| 1 | **Graph** | Graph.h/cpp, Roadmap.h/cpp | 图存储 + CostVector | 所有模块 |
| 2 | **AVLTree** | AVLTree.h/cpp | 平衡二叉搜索树 | ParetoFrontier |
| 3 | **ParetoFrontier** | ParetoFrontier.h/cpp | 支配性检查 | ERCA* |
| 4 | **Dijkstra** | Dijkstra.h/cpp | 启发式 h(v) + 路径重建 | ERCA*, 实验程序 |
| 5 | **ERCA*** | ERCA.h/cpp | 核心搜索算法 | Normalizer, 实验程序 |
| 6 | **Pulse** | Pulse.h/cpp | DFS上界搜索（已弃用） | Normalizer(旧) |
| 7 | **Normalizer** | Normalizer.h/cpp | 资源归一化+权重调整+上界计算 | 实验程序 |
| 8 | **NavMeshConverter** | NavMeshConverter.h/cpp | NavMesh→Roadmap转换 | navmesh_benchmark |
