# NavMesh到ERCA*最小可行实验

## 快速开始

这是一个验证RecastNavigation生成的NavMesh可以转换为图结构，并使用ERCA*算法进行资源约束路径规划的最小可行实验。

### 实验结果概览

✅ **实验状态**：成功完成
📅 **完成时间**：2026-01-30 18:44
⏱️ **总耗时**：约66秒

### 关键数据

- **NavMesh**：6个多边形（从121顶点、200三角形生成）
- **图结构**：6个节点、10条边
- **ERCA*求解时间**：37.4微秒
- **找到的最优路径**：节点0 → 4 → 2 → 5
- **路径成本**：9.42单位（满足资源约束）

## 快速运行

### 1. 生成NavMesh和图文件

```bash
cd NavMeshERCA_Experiment/build/Release
./navmesh_erca_experiment.exe
```

生成文件：
- ✓ erca_input.txt (361 bytes)
- ✓ navmesh_cost.gr (152 bytes)
- ✓ navmesh_res1.gr (168 bytes)
- ✓ navmesh_res2.gr (174 bytes)

### 2. 运行ERCA*求解

```bash
cd ../../../public_erca/build/Release

# 复制图文件
cp ../../../NavMeshERCA_Experiment/build/Release/navmesh_*.gr ./

# 运行ERCA*
./run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt

# 查看结果
cat navmesh_result.txt
```

预期输出：
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

## 文件说明

### 源代码

| 文件 | 说明 |
|------|------|
| [main_en.cpp](main_en.cpp) | 主程序，生成NavMesh并转换为图 |
| [GraphConverter.h](GraphConverter.h) | 图转换器接口定义 |
| [GraphConverter_en.cpp](GraphConverter_en.cpp) | 图转换器实现 |
| [CMakeLists.txt](CMakeLists.txt) | CMake构建配置 |

### 输出数据

| 文件 | 格式 | 说明 |
|------|------|------|
| erca_input.txt | 自定义 | 简化格式的图文件 |
| navmesh_cost.gr | DIMACS | 成本图（最小化目标） |
| navmesh_res1.gr | DIMACS | 资源1图（体力消耗） |
| navmesh_res2.gr | DIMACS | 资源2图（行动点消耗） |
| navmesh_result.txt | 文本 | ERCA*求解结果 |

### 文档

| 文件 | 说明 |
|------|------|
| [EXPERIMENT_REPORT.md](EXPERIMENT_REPORT.md) | 完整实验报告（含所有实际输出） |
| [README.md](README.md) | 本文件，快速指南 |

## 实验验证了什么？

1. ✅ RecastNavigation可以从简单地形生成NavMesh
2. ✅ NavMesh可以正确转换为带权重的图结构
3. ✅ 图数据可以导出为DIMACS格式
4. ✅ ERCA*可以在资源约束下找到最优路径
5. ✅ 整个流程性能优秀（求解时间< 0.1毫秒）

## 技术栈

- **RecastNavigation**：导航网格生成
- **ERCA***：资源约束A*算法 ([GitHub](https://github.com/rap-lab-org/public_erca))
- **编译器**：MSVC 19.44 (Visual Studio 2022)
- **构建工具**：CMake 3.16+
- **语言**：C++11

## 编译依赖

### RecastNavigation
```bash
cd recastnavigation/build
cmake .. -DRECASTNAVIGATION_DEMO=OFF -DRECASTNAVIGATION_TESTS=OFF
cmake --build . --config Release
```

### ERCA*
```bash
cd public_erca/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

### 本项目
```bash
cd NavMeshERCA_Experiment/build
cmake ..
cmake --build . --config Release
```

## 核心代码示例

### NavMesh转图结构

```cpp
Graph graph = GraphConverter::convertNavMeshToGraph(navMesh);
// 生成6个节点，10条边
```

### 导出DIMACS格式

```cpp
GraphConverter::exportToDIMACSFormat(
    graph,
    "navmesh_cost.gr",    // 成本文件
    "navmesh_res1.gr",    // 资源1（体力）
    "navmesh_res2.gr"     // 资源2（行动点）
);
```

### 运行ERCA*

```bash
./run_erca.exe [起点] [终点] [时限] [资源维度] [成本文件] [资源1文件] [资源2文件] [资源1上限] [资源2上限] [结果文件]
```

## 实际结果

### 找到的路径

```
DIMACS格式（1-based）: 1 → 5 → 3 → 6
实际节点（0-based）:   0 → 4 → 2 → 5
```

### 路径成本

| 指标 | 值 | 约束 | 状态 |
|------|-----|------|------|
| 总成本 | 9.42 | - | - |
| 体力消耗 | 9.42 | < 50 | ✓ 满足 |
| 行动点消耗 | 3.0 | < 30 | ✓ 满足 |

### 性能

- 初始化时间：9.9微秒
- 搜索时间：37.4微秒
- 总求解时间：**< 0.1毫秒**

## 下一步工作

- [ ] 测试更大规模的NavMesh（数百到数千个多边形）
- [ ] 实现不同地形类型的差异化成本
- [ ] 添加路径可视化（绘制到NavMesh上）
- [ ] 集成到游戏引擎（Unity/Unreal）
- [ ] 性能优化和缓存策略

## 问题反馈

如果遇到编译或运行问题，请参考 [EXPERIMENT_REPORT.md](EXPERIMENT_REPORT.md) 中的"实验中的问题与解决方案"章节。

## 许可证

本实验代码用于学术研究目的。

- RecastNavigation：使用其原始许可证
- ERCA*：学术和非商业用途
- 本项目代码：MIT License

## 引用

如果这个实验对您的研究有帮助，请引用：

```bibtex
@misc{navmesh_erca_experiment_2026,
  title={NavMesh to ERCA* Minimal Viable Experiment},
  author={Your Name},
  year={2026},
  note={Validation of resource-constrained pathfinding on navigation meshes}
}
```

ERCA*论文：
```bibtex
@article{ren2023erca,
  title={ERCA*: A New Approach for the Resource Constrained Shortest Path Problem},
  author={Ren, Zhongqiang and Rubinstein, Zachary B. and Smith, Stephen F. and Rathinam, Sivakumar and Choset, Howie},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  year={2023}
}
```

---

**实验成功！** 🎉

完整报告请查看 [EXPERIMENT_REPORT.md](EXPERIMENT_REPORT.md)
