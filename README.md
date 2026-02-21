# 毕业设计：带资源约束的NavMesh最短路径算法

**作者**：蔡以恒（3210103929）
**院系**：浙江大学计算机学院
**导师**：王锐
**课题**：带有资源约束的最短路径/时间问题（RCSPP）在导航网格上的研究

---

## 目录结构说明

```
Graduation/
├── ThesisRCSPP/               # 核心代码（自主实现）
│   ├── include/               # 头文件
│   │   ├── algorithm/         # 算法接口（Dijkstra, ERCA, Pulse, Normalizer）
│   │   ├── frontier/          # Pareto 前沿数据结构（AVLTree, ParetoFrontier）
│   │   ├── graph/             # 图数据结构（Graph, Roadmap, NavMeshConverter）
│   │   └── benchmark/         # 实验框架接口
│   ├── src/                   # 对应实现文件
│   ├── app/                   # 实验程序入口
│   │   ├── main.cpp           # 正确性验证（6节点手工图）
│   │   ├── run_benchmark.cpp  # 随机图基准测试
│   │   ├── pruning_analysis.cpp  # 剪枝效果分析
│   │   ├── ablation_study.cpp    # 消融实验
│   │   ├── expanded_benchmark.cpp # 扩展实验（多种子+逐迭代分析）
│   │   ├── smart_benchmark.cpp   # 智能阈值（α难度系数）实验
│   │   └── navmesh_benchmark.cpp # NavMesh数据测试
│   ├── docs/                  # 设计思路与优化策略文档
│   │   ├── 剪枝策略想法.md        # 最初的剪枝策略手稿
│   │   ├── implementation_steps.md   # 8模块架构设计说明
│   │   └── optimization_strategies.md # 所有优化策略汇总
│   ├── results/               # 实验结果
│   │   ├── result.md          # 完整实验结果记录
│   │   ├── ablation_results.csv
│   │   └── pruning_analysis.csv
│   └── CMakeLists.txt
│
├── NavMeshERCA_Experiment/    # 早期原型验证实验
│   ├── GraphConverter.h/.cpp  # NavMesh → 图转换器（原型版）
│   ├── main.cpp               # 基本流程验证
│   ├── main_en.cpp            # 英文版（避免MSVC中文编码问题）
│   └── README.md / EXPERIMENT_REPORT.md
│
├── 参考文献/                  # 核心参考论文（PDF）
│   ├── ren23_ERCA_TITS.pdf    # ERCA*算法原论文
│   ├── lozano2013.pdf         # Pulse算法原论文
│   └── ...
│
├── 蔡以恒开题报告.docx         # 开题报告正式版
├── 导航网格资源受限最短路径算法研究开题报告.docx  # 开题报告草稿
└── README.md                  # 本文件
```

---

## 项目背景与创新点

### 问题定义

在游戏AI场景（如博德之门3风格的导航）中，NPC寻路不仅需要找最短路径，还受多种资源约束（体力、行动点、法力等）。这是经典的**资源约束最短路径问题（RCSPP）**，属于NP-Hard问题。

### 核心算法：ERCA*

基于 Ren et al. (2023) 的增强型资源约束A*算法，主要特点：
- 基于 Pareto 前沿的支配性剪枝（BBST实现）
- A* 启发式加速
- 支持多维资源约束（1-N个资源）

### 自主创新：资源归一化预处理

将N个资源约束合并为1个归一化资源，在低维图上快速预求解上界，用于加速主搜索：

```
1. 归一化：用分位数系数将多资源压缩为单资源
2. 迭代权重调整：最多5轮，超限资源权重×2，宽裕资源权重×0.5
3. 上界用于 ERCA* 的额外剪枝
4. 配合启发式膨胀（w=1.5）实现最大加速
```

### 实验关键结论

| 方法 | 节点缩减 | 时间加速 | 解的质量 |
|------|---------|---------|---------|
| 纯 ERCA*（基线） | 0% | 1.0x | 精确 |
| 启发式膨胀（w=1.5） | 83-92% | 3-5x | ≤11%偏差 |
| 归一化+上界+膨胀 | 100%（宽松约束） | 1.37x（大图） | ≤11%偏差 |

- α=0.4 是搜索最难的"甜点"（84%可解率）
- 归一化预处理对 >1000 节点的图有正收益

---

## 开发环境

- 操作系统：Windows 11
- 编译器：MSVC 2022（Visual Studio）
- 构建系统：CMake 3.20+
- 依赖：RecastNavigation（navmesh_benchmark，可选编译）

## 构建方法

```bash
cd ThesisRCSPP
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

## 不在版本控制中的内容

以下内容被 `.gitignore` 排除，不上传至仓库：
- `build/` — 编译产物
- `public_erca/` — 参考实现（第三方代码）
- `recastnavigation/` — 第三方库
- `2026届/` — 学校行政模板文件
- `.vs/` — IDE 配置文件
