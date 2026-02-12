# NavMesh到ERCA*最小可行实验报告

## 实验目标

验证RecastNavigation生成的NavMesh可以转换为图结构，并使用ERCA*算法进行资源约束路径规划。

## 实验日期

2026-01-30

## 实验步骤

### 1. 生成NavMesh

使用RecastNavigation库生成简单的导航网格：

- **输入网格**：10x10的测试地形
  - 顶点数：121
  - 三角形数：200
  - 添加高度变化以创建多个导航多边形

- **NavMesh配置**：
  - 网格大小：33 x 33
  - 单元格大小：0.3
  - 生成多边形数：6个

### 2. 转换为图结构

成功将NavMesh转换为图结构：

- **节点数**：6个（每个多边形对应一个节点）
- **边数**：10条（连接相邻多边形）
- **资源类型**：
  - 资源1：体力消耗（基于距离和地形类型）
  - 资源2：行动点消耗（基于地形类型）

### 3. 导出ERCA*输入格式

#### 简化格式文件：erca_input.txt
```
6 10 2

# Edge list: from to cost resource1(stamina) resource2(action_point)
0 4 1.954 1.954 1
1 3 1.89341 1.89341 1
2 4 2.9413 2.9413 1
2 5 4.533 4.533 1
3 1 1.89341 1.89341 1
3 5 3.52278 3.52278 1
4 2 2.9413 2.9413 1
4 0 1.954 1.954 1
5 2 4.533 4.533 1
5 3 3.52278 3.52278 1

# Query: start end resource_limits(stamina action_point)
0 5 50 30
```

#### DIMACS格式文件（供ERCA*使用）

成功导出为DIMACS格式的三个文件：

#### 文件1：navmesh_cost.gr（成本文件）
```
c Cost file for NavMesh graph
p sp 6 10
a 1 5 195
a 2 4 189
a 3 5 294
a 3 6 453
a 4 2 189
a 4 6 352
a 5 3 294
a 5 1 195
a 6 3 453
a 6 4 352
```

#### 文件2：navmesh_res1.gr（资源1-体力）
```
c Resource 1 (Stamina) file for NavMesh graph
p sp 6 10
a 1 5 195
a 2 4 189
a 3 5 294
a 3 6 453
a 4 2 189
a 4 6 352
a 5 3 294
a 5 1 195
a 6 3 453
a 6 4 352
```

#### 文件3：navmesh_res2.gr（资源2-行动点）
```
c Resource 2 (Action Points) file for NavMesh graph
p sp 6 10
a 1 5 100
a 2 4 100
a 3 5 100
a 3 6 100
a 4 2 100
a 4 6 100
a 5 3 100
a 5 1 100
a 6 3 100
a 6 4 100
```

### 4. 运行ERCA*求解器

**命令**：
```bash
./run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt
```

**参数说明**：
- 起点：节点1（0-based索引中的节点0）
- 终点：节点6（0-based索引中的节点5）
- 时间限制：60秒
- 资源维度：3（1个成本+2个资源）
- 体力限制：5000（实际50单位 × 100）
- 行动点限制：3000（实际30单位 × 100）

## 实验结果

### 步骤1-4：NavMesh生成和图转换的实际输出

```
=== NavMesh to ERCA* Minimal Viable Experiment ===

[Step 1] Creating test mesh...
Created simple mesh: 121 vertices, 200 triangles

[Step 2] Building NavMesh...
NavMesh configuration:
  Grid size: 33 x 33
  Cell size: 0.3
Polygon mesh: 6 polygons
NavMesh built successfully!

[Step 3] Converting NavMesh to graph...
Created 6 nodes
Created 10 edges

[Step 4] Exporting ERCA* input file...
Successfully exported ERCA* input file: erca_input.txt
  Nodes: 6
  Edges: 10
  Start: 0, End: 5
  Resource limits: Stamina=50, ActionPoint=30
Successfully exported DIMACS format files:
  Cost file: navmesh_cost.gr
  Resource 1 file: navmesh_res1.gr
  Resource 2 file: navmesh_res2.gr

=== Experiment Complete ===
ERCA* input file generated: erca_input.txt
Next step: Use ERCA* solver to process this file
```

### ERCA*求解器实际运行日志

```bash
$ ./run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt

[INFO] input arg[0]=C:\Users\DELL\Graduation\public_erca\build\Release\run_erca.exe
[INFO] input arg[1]=1
[INFO] input arg[2]=6
[INFO] input arg[3]=60
[INFO] input arg[4]=3
[INFO] input arg[5]=navmesh_cost.gr
[INFO] input arg[6]=navmesh_res1.gr
[INFO] input arg[7]=navmesh_res2.gr
[INFO] input arg[8]=5000
[INFO] input arg[9]=3000
[INFO] input arg[10]=navmesh_result.txt
 get resource limit 5000
 get resource limit 3000
Running read_graph_file():
cost_fname 0: navmesh_cost.gr
cost_fname 1: navmesh_res1.gr
cost_fname 2: navmesh_res2.gr
p:num_nodes: 6
p:num_edges: 10
num_nodes: 6
num_edges: 10
cdims: 3
[INFO] RunERCA, M=3 time_limit = 60
[INFO] ERCAKd::_PostProcRes...
[INFO] ERCAKd::Search exit.
```

### ERCA*求解结果文件内容

```
Results:
rt_initHeu: 9.9e-06
rt_search: 3.74e-05
timeout: 0
N: 1
Label: 5
[942,942,300,]
1 5 3 6
```

### 结果解析

**性能指标**：
- 初始化时间：9.9微秒
- 搜索时间：37.4微秒
- 总求解时间：< 0.0001秒
- 超时状态：否

**找到的路径**：
- DIMACS格式（1-based）：1 → 5 → 3 → 6
- 实际节点ID（0-based）：0 → 4 → 2 → 5

**路径成本**：
- 总成本：942（DIMACS缩放值，实际 942/100 = 9.42单位）
- 体力消耗：942（DIMACS缩放值，实际 942/100 = 9.42单位）
- 行动点消耗：300（DIMACS缩放值，实际 300/100 = 3.0单位）

**资源约束满足情况**：
- 体力：9.42 < 50 ✓（满足约束）
- 行动点：3.0 < 30 ✓（满足约束）

### 路径详细分析

#### 路径各段成本分解

找到的最优路径：节点1 → 节点5 → 节点3 → 节点6（DIMACS 1-based）
对应实际节点：节点0 → 节点4 → 节点2 → 节点5（0-based）

| 路径段 | 起点 | 终点 | 段成本 | 段体力 | 段行动点 | 累计成本 | 累计体力 | 累计行动点 |
|--------|------|------|--------|--------|----------|----------|----------|------------|
| 第1段  | 0    | 4    | 1.954  | 1.954  | 1.0      | 1.954    | 1.954    | 1.0        |
| 第2段  | 4    | 2    | 2.9413 | 2.9413 | 1.0      | 4.8953   | 4.8953   | 2.0        |
| 第3段  | 2    | 5    | 4.533  | 4.533  | 1.0      | 9.4283   | 9.4283   | 3.0        |
| **总计** |    |      | **9.4283** | **9.4283** | **3.0** | - | - | - |

**验证**：
- ERCA*报告的成本：942 / 100 = 9.42
- 手工计算：1.954 + 2.9413 + 4.533 = 9.4283
- 误差：|9.42 - 9.4283| = 0.0083（由于DIMACS格式整数化导致的舍入误差）

## 图结构统计

### 图的拓扑结构

```
节点邻接关系（→ 表示有向边）：

节点0: → 4 (成本1.954)
节点1: → 3 (成本1.893)
节点2: → 4 (成本2.941), → 5 (成本4.533)
节点3: → 1 (成本1.893), → 5 (成本3.523)
节点4: → 0 (成本1.954), → 2 (成本2.941)
节点5: → 2 (成本4.533), → 3 (成本3.523)
```

**图的连通性分析**：
- 总节点数：6
- 总边数：10
- 平均出度：10/6 ≈ 1.67
- 图类型：强连通（所有节点可互相到达）

**从节点0到节点5的可能路径**：
1. ✅ **0 → 4 → 2 → 5**（总成本：1.954 + 2.941 + 4.533 = **9.428**）← ERCA*选择
2. 0 → 4 → 2 → ... → 5（其他间接路径，成本更高）

**路径选择说明**：
- 从节点0出发，只有一条出边指向节点4
- 从节点4可以到达节点0或节点2
- 从节点2可以到达节点4（回路）或节点5（目标）
- 因此0→4→2→5是最短路径（实际上也是唯一的简单路径）

### 节点信息
- 节点0-5：对应NavMesh中的6个多边形
- 每个节点存储：
  - 节点ID
  - 多边形中心点坐标
  - 地形类型（本实验中均为类型0）

### 边信息
10条有向边的实际统计（基于生成的文件）：

| 边序号 | 起点(0-based) | 终点(0-based) | 成本(实际值) | 体力消耗 | 行动点消耗 | DIMACS起点(1-based) | DIMACS终点 | DIMACS成本(×100) |
|--------|---------------|---------------|--------------|----------|------------|---------------------|------------|------------------|
| 1      | 0             | 4             | 1.954        | 1.954    | 1.0        | 1                   | 5          | 195              |
| 2      | 1             | 3             | 1.89341      | 1.89341  | 1.0        | 2                   | 4          | 189              |
| 3      | 2             | 4             | 2.9413       | 2.9413   | 1.0        | 3                   | 5          | 294              |
| 4      | 2             | 5             | 4.533        | 4.533    | 1.0        | 3                   | 6          | 453              |
| 5      | 3             | 1             | 1.89341      | 1.89341  | 1.0        | 4                   | 2          | 189              |
| 6      | 3             | 5             | 3.52278      | 3.52278  | 1.0        | 4                   | 6          | 352              |
| 7      | 4             | 2             | 2.9413       | 2.9413   | 1.0        | 5                   | 3          | 294              |
| 8      | 4             | 0             | 1.954        | 1.954    | 1.0        | 5                   | 1          | 195              |
| 9      | 5             | 2             | 4.533        | 4.533    | 1.0        | 6                   | 3          | 453              |
| 10     | 5             | 3             | 3.52278      | 3.52278  | 1.0        | 6                   | 4          | 352              |

**说明**：
- 所有行动点消耗均为1.0（默认地形类型）
- 体力消耗等于边的距离成本
- DIMACS格式使用1-based索引，内部图使用0-based索引
- DIMACS格式的成本值缩放了100倍以使用整数

## 技术栈

### 使用的库和工具

1. **RecastNavigation**
   - 版本：最新master分支
   - 用途：生成导航网格
   - 编译：CMake + MSVC

2. **ERCA***
   - 仓库：https://github.com/rap-lab-org/public_erca
   - 用途：资源约束路径规划
   - 编译：CMake + MSVC（修改为STATIC库）

3. **自定义代码**
   - GraphConverter：NavMesh到图的转换器
   - main_en.cpp：实验主程序

### 开发环境（实际使用）

- **操作系统**：Windows 10.0.22631
- **编译器**：Microsoft Visual Studio 2022 Community
  - MSVC版本：19.44.35220.0
  - .NET Framework MSBuild版本：17.14.23+b0019275e
- **CMake**：3.16.3+
- **Windows SDK**：10.0.26100.0
- **编程语言**：C++ (C++11标准)
- **构建配置**：Release
- **目标平台**：x64

## 核心代码结构

### GraphConverter类

```cpp
class GraphConverter {
public:
    // 将NavMesh转换为图结构
    static Graph convertNavMeshToGraph(const dtNavMesh* navMesh);

    // 导出ERCA*输入格式
    static bool exportToERCAFormat(...);

    // 导出DIMACS格式
    static bool exportToDIMACSFormat(...);

private:
    // 计算多边形中心点
    static void computePolygonCenter(...);

    // 获取地形成本
    static float getStaminaCost(unsigned char terrainType);
    static float getAPCost(unsigned char terrainType);
};
```

### 数据结构

```cpp
struct GraphNode {
    int id;                    // 节点ID
    float center[3];           // 中心点坐标
    unsigned char terrainType; // 地形类型
};

struct GraphEdge {
    int from;                  // 起始节点
    int to;                    // 目标节点
    float cost;                // 移动成本
    float resources[2];        // 资源消耗
};

struct Graph {
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
};
```

## 数据验证与一致性检查

### 文件格式转换验证

#### erca_input.txt vs DIMACS格式一致性检查

| 验证项 | erca_input.txt | DIMACS格式 | 状态 |
|--------|----------------|------------|------|
| 节点数 | 6 | 6 | ✓ 一致 |
| 边数 | 10 | 10 | ✓ 一致 |
| 资源维度 | 2 | 2 (res1+res2) | ✓ 一致 |

**边数据抽查**（边1：0→4）：
- erca_input.txt: `0 4 1.954 1.954 1`
- DIMACS cost: `a 1 5 195` (1.954 × 100 = 195.4 ≈ 195)
- DIMACS res1: `a 1 5 195` (1.954 × 100 = 195)
- DIMACS res2: `a 1 5 100` (1.0 × 100 = 100)
- 状态：✓ 一致（考虑索引偏移和缩放）

### ERCA*输出验证

**路径重构验证**：
```
ERCA*输出路径（DIMACS 1-based）: 1 5 3 6
转换为0-based: 0 4 2 5

验证路径连通性：
- 边(0,4)存在? ✓ (成本1.954)
- 边(4,2)存在? ✓ (成本2.941)
- 边(2,5)存在? ✓ (成本4.533)

总成本计算：
1.954 + 2.941 + 4.533 = 9.428
ERCA*报告：942/100 = 9.42
误差：0.008 (0.08%) ✓ 可接受
```

**资源消耗验证**：
```
体力消耗 = 路径成本 = 9.42
行动点消耗 = 路径段数 × 1.0 = 3 × 1.0 = 3.0
ERCA*报告：[942, 942, 300] → [9.42, 9.42, 3.0]
✓ 完全匹配
```

## 实验结论

### 成功验证的内容

1. ✅ **NavMesh生成**：RecastNavigation成功生成多边形导航网格
2. ✅ **图转换**：NavMesh可以正确转换为带权重的图结构
3. ✅ **格式导出**：成功导出DIMACS格式供ERCA*使用
4. ✅ **路径规划**：ERCA*成功在资源约束下找到最优路径
5. ✅ **性能表现**：求解速度极快（< 0.1毫秒）

### 技术可行性

本实验证明了以下技术路线的可行性：

```
游戏场景网格 → RecastNavigation → NavMesh → 图结构 → ERCA* → 资源约束路径
```

这为在游戏中实现带资源约束的路径规划提供了完整的技术方案。

## 实验中的问题与解决方案

### 问题1：初始NavMesh只生成1个多边形

**现象**：
```
Polygon mesh: 1 polygons
Created 1 nodes
Created 0 edges
```

**原因**：平面地形太简单，RecastNavigation将整个区域合并为单个大多边形

**解决方案**：在测试地形中添加高度变化
```cpp
// 添加高度变化强制生成多个多边形
if (x >= 3 && x <= 4 && z >= 3 && z <= 7) {
    height = 2.0f; // Raised platform 1
} else if (x >= 6 && x <= 8 && z >= 2 && z <= 5) {
    height = 1.5f; // Raised platform 2
}
```

**效果**：成功生成6个多边形和10条边

### 问题2：ERCA*编译链接错误

**错误信息**：
```
LINK : fatal error LNK1181: 无法打开输入文件"Release\emoa.lib"
```

**原因**：CMakeLists.txt配置为SHARED库，Windows下生成.dll需要导出符号

**解决方案**：修改为STATIC库
```cmake
# 修改前
add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})

# 修改后
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
```

### 问题3：中文编码导致编译失败

**错误信息**：
```
warning C4819: 该文件包含不能在当前代码页(936)中表示的字符
error C2001: 常量中有换行符
```

**原因**：中文注释在MSVC下导致编译错误

**解决方案**：
1. 创建`_en.cpp`版本，使用纯英文注释
2. 或将文件保存为UTF-8 with BOM格式

### 问题4：DIMACS格式索引不一致

**问题**：内部图使用0-based索引，DIMACS格式要求1-based索引

**解决方案**：在导出时进行转换
```cpp
// 0-based → 1-based
costOut << "a " << (edge.from + 1) << " " << (edge.to + 1) << " "
        << static_cast<int>(edge.cost * 100) << std::endl;
```

## 关键经验与最佳实践

### 数据转换经验

1. **索引转换**：注意不同系统的索引约定
   - C++图结构：0-based
   - DIMACS格式：1-based
   - 需要在接口处明确转换

2. **数值缩放**：
   - DIMACS格式要求整数权重
   - 浮点数成本需要缩放（如×100）
   - 结果需要反向缩放以恢复实际值

3. **格式对齐**：
   - 所有资源文件必须包含相同的边集
   - 边的顺序必须完全一致
   - 不能有边在某个文件中存在而在其他文件中不存在

### 性能优化建议

1. **NavMesh配置**：
   - 较大的cell size可减少多边形数量
   - 较小的cell size可提高精度
   - 需要在性能和精度间权衡

2. **图结构优化**：
   - 预计算多边形中心点
   - 缓存边的成本计算
   - 考虑使用邻接表而非边列表

3. **ERCA*参数调优**：
   - 时间限制根据实际需求设定
   - 资源限制影响搜索空间
   - 启发式函数可显著提升性能

## 下一步工作

### 短期目标

1. **扩大规模测试**
   - 使用更大、更复杂的地形
   - 测试数百到数千个多边形的情况

2. **多种地形类型**
   - 实现不同地形的资源消耗差异
   - 草地、道路、水域、泥地等

3. **可视化**
   - 将ERCA*找到的路径绘制回NavMesh
   - 使用Unity/Unreal或OpenGL可视化

### 中期目标

1. **集成到游戏引擎**
   - Unity插件开发
   - Unreal Engine集成

2. **性能优化**
   - 大规模图的预处理
   - 缓存和增量更新

3. **动态障碍物**
   - 支持动态环境变化
   - 实时重规划

### 长期目标

1. **多代理协调**
   - 多个角色同时规划
   - 避免碰撞

2. **机器学习增强**
   - 学习最优资源分配策略
   - 预测玩家行为

## 参考文献

1. RecastNavigation: https://github.com/recastnavigation/recastnavigation
2. ERCA* Paper: "ERCA*: A New Approach for the Resource Constrained Shortest Path Problem"
   - Authors: Zhongqiang Ren, Zachary B. Rubinstein, Stephen F. Smith, Sivakumar Rathinam, Howie Choset
3. ERCA* Implementation: https://github.com/rap-lab-org/public_erca

## 附录

### 实际生成文件清单

#### 输出数据文件（生成于 2026-01-30 18:44）

| 文件名 | 大小 | 用途 | 格式 |
|--------|------|------|------|
| erca_input.txt | 361 bytes | 简化格式的图文件 | 自定义文本格式 |
| navmesh_cost.gr | 152 bytes | DIMACS成本图 | DIMACS图格式 |
| navmesh_res1.gr | 168 bytes | DIMACS资源1图（体力） | DIMACS图格式 |
| navmesh_res2.gr | 174 bytes | DIMACS资源2图（行动点） | DIMACS图格式 |
| navmesh_result.txt | ~100 bytes | ERCA*求解结果 | 文本格式 |

#### 所有文件的完整内容

##### erca_input.txt (361 bytes)
```
6 10 2

# Edge list: from to cost resource1(stamina) resource2(action_point)
0 4 1.954 1.954 1
1 3 1.89341 1.89341 1
2 4 2.9413 2.9413 1
2 5 4.533 4.533 1
3 1 1.89341 1.89341 1
3 5 3.52278 3.52278 1
4 2 2.9413 2.9413 1
4 0 1.954 1.954 1
5 2 4.533 4.533 1
5 3 3.52278 3.52278 1

# Query: start end resource_limits(stamina action_point)
0 5 50 30
```

##### navmesh_cost.gr (152 bytes)
```
c Cost file for NavMesh graph
p sp 6 10
a 1 5 195
a 2 4 189
a 3 5 294
a 3 6 453
a 4 2 189
a 4 6 352
a 5 3 294
a 5 1 195
a 6 3 453
a 6 4 352
```

##### navmesh_res1.gr (168 bytes)
```
c Resource 1 (Stamina) file for NavMesh graph
p sp 6 10
a 1 5 195
a 2 4 189
a 3 5 294
a 3 6 453
a 4 2 189
a 4 6 352
a 5 3 294
a 5 1 195
a 6 3 453
a 6 4 352
```

##### navmesh_res2.gr (174 bytes)
```
c Resource 2 (Action Points) file for NavMesh graph
p sp 6 10
a 1 5 100
a 2 4 100
a 3 5 100
a 3 6 100
a 4 2 100
a 4 6 100
a 5 3 100
a 5 1 100
a 6 3 100
a 6 4 100
```

##### navmesh_result.txt (~100 bytes)
```
Results:
rt_initHeu: 9.9e-06
rt_search: 3.74e-05
timeout: 0
N: 1
Label: 5
[942,942,300,]
1 5 3 6
```

### 项目文件结构

```
NavMeshERCA_Experiment/
├── main_en.cpp                 # 主程序（实际运行版本）
├── main.cpp                    # 主程序（中文版本，未使用）
├── GraphConverter.h            # 图转换器头文件
├── GraphConverter_en.cpp       # 图转换器实现
├── GraphConverter.cpp          # 图转换器实现（中文版本）
├── CMakeLists.txt             # 构建配置
├── EXPERIMENT_REPORT.md       # 本报告
└── build/
    ├── CMakeFiles/            # CMake生成文件
    ├── navmesh_erca_experiment.vcxproj  # Visual Studio项目文件
    └── Release/
        ├── navmesh_erca_experiment.exe  # 可执行程序
        ├── erca_input.txt      # 简化格式输出 (361 bytes)
        ├── navmesh_cost.gr     # DIMACS成本文件 (152 bytes)
        ├── navmesh_res1.gr     # DIMACS资源1文件 (168 bytes)
        ├── navmesh_res2.gr     # DIMACS资源2文件 (174 bytes)
        └── [输出数据文件]

public_erca/
├── build/
│   └── Release/
│       ├── run_erca.exe       # ERCA*求解器
│       ├── emoa.lib           # ERCA*静态库
│       ├── navmesh_cost.gr    # 复制过来的图文件
│       ├── navmesh_res1.gr    # 复制过来的图文件
│       ├── navmesh_res2.gr    # 复制过来的图文件
│       └── navmesh_result.txt # ERCA*求解结果
└── [ERCA*源代码]

recastnavigation/
└── build/
    └── Release/
        ├── Recast.lib         # Recast静态库
        ├── Detour.lib         # Detour静态库
        └── [其他库文件]
```

### 编译命令

```bash
# 编译RecastNavigation
cd recastnavigation/build
cmake .. -DRECASTNAVIGATION_DEMO=OFF -DRECASTNAVIGATION_TESTS=OFF
cmake --build . --config Release

# 编译实验程序
cd NavMeshERCA_Experiment/build
cmake ..
cmake --build . --config Release

# 编译ERCA*
cd public_erca/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

### 完整实验重现步骤

#### 步骤1：准备环境

```bash
# 克隆RecastNavigation
cd c:/Users/DELL/Graduation
git clone https://github.com/recastnavigation/recastnavigation.git

# 克隆ERCA*
git clone https://github.com/rap-lab-org/public_erca.git
```

#### 步骤2：编译RecastNavigation

```bash
cd recastnavigation
mkdir build
cd build
cmake .. -DRECASTNAVIGATION_DEMO=OFF -DRECASTNAVIGATION_TESTS=OFF
cmake --build . --config Release
```

**预期输出**：
```
Detour.vcxproj -> C:\Users\DELL\Graduation\recastnavigation\build\Detour\Release\Detour.lib
Recast.vcxproj -> C:\Users\DELL\Graduation\recastnavigation\build\Recast\Release\Recast.lib
```

#### 步骤3：编译实验程序

```bash
cd ../../NavMeshERCA_Experiment
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

**预期输出**：
```
navmesh_erca_experiment.vcxproj -> C:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release\navmesh_erca_experiment.exe
```

#### 步骤4：运行实验程序

```bash
cd Release
./navmesh_erca_experiment.exe
```

**实际输出**：
```
=== NavMesh to ERCA* Minimal Viable Experiment ===

[Step 1] Creating test mesh...
Created simple mesh: 121 vertices, 200 triangles

[Step 2] Building NavMesh...
NavMesh configuration:
  Grid size: 33 x 33
  Cell size: 0.3
Polygon mesh: 6 polygons
NavMesh built successfully!

[Step 3] Converting NavMesh to graph...
Created 6 nodes
Created 10 edges

[Step 4] Exporting ERCA* input file...
Successfully exported ERCA* input file: erca_input.txt
  Nodes: 6
  Edges: 10
  Start: 0, End: 5
  Resource limits: Stamina=50, ActionPoint=30
Successfully exported DIMACS format files:
  Cost file: navmesh_cost.gr
  Resource 1 file: navmesh_res1.gr
  Resource 2 file: navmesh_res2.gr

=== Experiment Complete ===
```

**生成的文件**：
- ✓ erca_input.txt (361 bytes)
- ✓ navmesh_cost.gr (152 bytes)
- ✓ navmesh_res1.gr (168 bytes)
- ✓ navmesh_res2.gr (174 bytes)

#### 步骤5：编译ERCA*

```bash
cd ../../../public_erca

# 修改CMakeLists.txt（第16行）
# 将 add_library(${PROJECT_NAME} SHARED ...)
# 改为 add_library(${PROJECT_NAME} STATIC ...)

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

**预期输出**：
```
emoa.vcxproj -> C:\Users\DELL\Graduation\public_erca\build\Release\emoa.lib
run_erca.vcxproj -> C:\Users\DELL\Graduation\public_erca\build\Release\run_erca.exe
```

#### 步骤6：复制图文件

```bash
cd Release
cp ../../../NavMeshERCA_Experiment/build/Release/navmesh_*.gr ./
```

**验证文件存在**：
```bash
ls navmesh_*.gr
# 应该看到：
# navmesh_cost.gr
# navmesh_res1.gr
# navmesh_res2.gr
```

#### 步骤7：运行ERCA*求解器

```bash
./run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt
```

**实际输出**：
```
[INFO] input arg[0]=C:\Users\DELL\Graduation\public_erca\build\Release\run_erca.exe
[INFO] input arg[1]=1
[INFO] input arg[2]=6
[INFO] input arg[3]=60
[INFO] input arg[4]=3
[INFO] input arg[5]=navmesh_cost.gr
[INFO] input arg[6]=navmesh_res1.gr
[INFO] input arg[7]=navmesh_res2.gr
[INFO] input arg[8]=5000
[INFO] input arg[9]=3000
[INFO] input arg[10]=navmesh_result.txt
 get resource limit 5000
 get resource limit 3000
Running read_graph_file():
cost_fname 0: navmesh_cost.gr
cost_fname 1: navmesh_res1.gr
cost_fname 2: navmesh_res2.gr
p:num_nodes: 6
p:num_edges: 10
num_nodes: 6
num_edges: 10
cdims: 3
[INFO] RunERCA, M=3 time_limit = 60
[INFO] ERCAKd::_PostProcRes...
[INFO] ERCAKd::Search exit.
```

#### 步骤8：查看结果

```bash
cat navmesh_result.txt
```

**实际输出**：
```
Results:
rt_initHeu: 9.9e-06
rt_search: 3.74e-05
timeout: 0
N: 1
Label: 5
[942,942,300,]
1 5 3 6
```

### 实验总耗时

| 阶段 | 耗时 |
|------|------|
| 编译RecastNavigation | ~30秒 |
| 编译实验程序 | ~15秒 |
| 运行实验程序 | < 1秒 |
| 编译ERCA* | ~20秒 |
| 运行ERCA*求解 | < 0.001秒 |
| **总计** | **~66秒** |

---

**实验完成时间**：2026-01-30 18:44
**实验状态**：成功 ✅
**实验地点**：c:\Users\DELL\Graduation\
**实验人员**：
