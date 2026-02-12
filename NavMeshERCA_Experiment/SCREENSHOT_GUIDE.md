# 实验截图操作指南

本文档提供所有需要截图的命令和预期输出，方便记录实验过程。

## 目录

1. [环境信息](#1-环境信息)
2. [运行NavMesh实验程序](#2-运行navmesh实验程序)
3. [查看生成的图文件](#3-查看生成的图文件)
4. [运行ERCA*求解器](#4-运行erca求解器)
5. [查看求解结果](#5-查看求解结果)
6. [可视化路径](#6-可视化路径可选)

---

## 1. 环境信息

### 📸 截图1：系统信息

```bash
# Windows系统信息
systeminfo | findstr /B /C:"OS Name" /C:"OS Version" /C:"System Type"

# 或者简单版本
ver
```

### 📸 截图2：编译器版本

```bash
# 查看MSVC版本
cl

# 查看CMake版本
cmake --version
```

---

## 2. 运行NavMesh实验程序

### 📸 截图3：进入实验目录并运行程序

```bash
# 进入构建目录
cd c:/Users/DELL/Graduation/NavMeshERCA_Experiment/build/Release

# 列出当前目录文件
dir

# 运行实验程序
./navmesh_erca_experiment.exe
```

**预期输出**（完整）：
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

### 📸 截图4：验证生成的文件

```bash
# 查看生成的文件及大小
dir *.txt *.gr

# 或者用ls（如果有Git Bash）
ls -lh *.txt *.gr
```

**预期输出**：
```
-rw-r--r-- 1 DELL 197121 361  erca_input.txt
-rw-r--r-- 1 DELL 197121 152  navmesh_cost.gr
-rw-r--r-- 1 DELL 197121 168  navmesh_res1.gr
-rw-r--r-- 1 DELL 197121 174  navmesh_res2.gr
```

---

## 3. 查看生成的图文件

### 📸 截图5：查看简化格式图文件

```bash
# 查看erca_input.txt
type erca_input.txt

# 或者
cat erca_input.txt
```

**预期输出**：
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

### 📸 截图6：查看DIMACS成本文件

```bash
# 查看成本图
type navmesh_cost.gr

# 或者
cat navmesh_cost.gr
```

**预期输出**：
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

### 📸 截图7：查看DIMACS资源文件

```bash
# 查看资源1（体力）
type navmesh_res1.gr

# 查看资源2（行动点）
type navmesh_res2.gr
```

---

## 4. 运行ERCA*求解器

### 📸 截图8：复制文件到ERCA*目录

```bash
# 进入ERCA*构建目录
cd c:/Users/DELL/Graduation/public_erca/build/Release

# 复制图文件（如果还没复制）
copy c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release\navmesh_*.gr .

# 验证文件
dir navmesh_*.gr
```

### 📸 截图9：运行ERCA*求解器

```bash
# 运行ERCA*
./run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt
```

**预期输出**（完整）：
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

---

## 5. 查看求解结果

### 📸 截图10：查看ERCA*求解结果

```bash
# 查看结果文件
type navmesh_result.txt

# 或者
cat navmesh_result.txt
```

**预期输出**：
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

### 📸 截图11：解析结果（可选）

创建一个简单的Python脚本来解析和可视化结果：

```bash
# 创建解析脚本
cat > parse_result.py << 'EOF'
# -*- coding: utf-8 -*-
# 解析ERCA*结果

print("=" * 60)
print("ERCA* 求解结果分析")
print("=" * 60)

# 读取结果文件
with open('navmesh_result.txt', 'r') as f:
    lines = f.readlines()

# 解析性能指标
for line in lines:
    if 'rt_initHeu' in line:
        init_time = float(line.split(':')[1].strip())
        print(f"初始化时间: {init_time*1e6:.2f} 微秒")
    elif 'rt_search' in line:
        search_time = float(line.split(':')[1].strip())
        print(f"搜索时间: {search_time*1e6:.2f} 微秒")
        print(f"总求解时间: {(init_time + search_time)*1e6:.2f} 微秒")
    elif line.startswith('['):
        # 解析成本
        costs = line.strip()[1:-2].split(',')
        print(f"\n路径成本（DIMACS格式）:")
        print(f"  总成本: {costs[0]}")
        print(f"  体力消耗: {costs[1]}")
        print(f"  行动点消耗: {costs[2]}")
        print(f"\n路径成本（实际值）:")
        print(f"  总成本: {int(costs[0])/100:.2f} 单位")
        print(f"  体力消耗: {int(costs[1])/100:.2f} 单位")
        print(f"  行动点消耗: {int(costs[2])/100:.2f} 单位")
    elif line[0].isdigit() and ' ' in line:
        # 解析路径
        path = line.strip().split()
        print(f"\n找到的路径（DIMACS 1-based）:")
        print(f"  {' → '.join(path)}")
        path_0based = [str(int(p)-1) for p in path]
        print(f"\n找到的路径（0-based）:")
        print(f"  {' → '.join(path_0based)}")

print("\n" + "=" * 60)
print("验证约束条件:")
print("  体力限制: 50 单位")
print("  行动点限制: 30 单位")
print(f"  体力消耗: {int(costs[1])/100:.2f} < 50 ✓")
print(f"  行动点消耗: {int(costs[2])/100:.2f} < 30 ✓")
print("=" * 60)
EOF

# 运行解析脚本
python parse_result.py
```

**预期输出**：
```
============================================================
ERCA* 求解结果分析
============================================================
初始化时间: 9.90 微秒
搜索时间: 37.40 微秒
总求解时间: 47.30 微秒

路径成本（DIMACS格式）:
  总成本: 942
  体力消耗: 942
  行动点消耗: 300

路径成本（实际值）:
  总成本: 9.42 单位
  体力消耗: 9.42 单位
  行动点消耗: 3.00 单位

找到的路径（DIMACS 1-based）:
  1 → 5 → 3 → 6

找到的路径（0-based）:
  0 → 4 → 2 → 5

============================================================
验证约束条件:
  体力限制: 50 单位
  行动点限制: 30 单位
  体力消耗: 9.42 < 50 ✓
  行动点消耗: 3.00 < 30 ✓
============================================================
```

---

## 6. 可视化路径（可选）

### 📸 截图12：创建路径可视化图表

创建一个简单的可视化脚本：

```bash
# 创建可视化脚本
cat > visualize_graph.py << 'EOF'
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 读取图数据
edges = []
with open('erca_input.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
        if line.strip() and not line.startswith('#'):
            parts = line.strip().split()
            if len(parts) == 5:
                edges.append({
                    'from': int(parts[0]),
                    'to': int(parts[1]),
                    'cost': float(parts[2])
                })

# 读取路径
path = [0, 4, 2, 5]  # ERCA*找到的路径

# 节点位置（手动布局，可以调整）
node_positions = {
    0: (0, 2),
    1: (2, 0),
    2: (4, 2),
    3: (2, 4),
    4: (2, 2),
    5: (6, 2)
}

# 创建图形
fig, ax = plt.subplots(1, 1, figsize=(12, 8))
ax.set_xlim(-1, 7)
ax.set_ylim(-1, 5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title('NavMesh图结构与ERCA*找到的最优路径', fontsize=16, fontweight='bold')

# 绘制所有边（灰色）
for edge in edges:
    start = node_positions[edge['from']]
    end = node_positions[edge['to']]
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='->', color='gray', lw=1, alpha=0.5))
    # 添加成本标签
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    ax.text(mid_x, mid_y, f'{edge["cost"]:.2f}',
            fontsize=8, color='gray', ha='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

# 高亮显示路径边（红色）
for i in range(len(path) - 1):
    start = node_positions[path[i]]
    end = node_positions[path[i+1]]
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='->', color='red', lw=3))

# 绘制节点
for node_id, pos in node_positions.items():
    if node_id in path:
        # 路径上的节点用红色
        color = 'red' if node_id == path[0] or node_id == path[-1] else 'orange'
        label = '起点' if node_id == path[0] else ('终点' if node_id == path[-1] else '')
        circle = plt.Circle(pos, 0.3, color=color, ec='darkred', linewidth=2, zorder=10)
        ax.add_patch(circle)
        ax.text(pos[0], pos[1], str(node_id), ha='center', va='center',
                fontsize=14, fontweight='bold', color='white', zorder=11)
        if label:
            ax.text(pos[0], pos[1]-0.6, label, ha='center', va='top',
                    fontsize=10, fontweight='bold', color='red')
    else:
        # 其他节点用蓝色
        circle = plt.Circle(pos, 0.3, color='lightblue', ec='blue', linewidth=2, zorder=10)
        ax.add_patch(circle)
        ax.text(pos[0], pos[1], str(node_id), ha='center', va='center',
                fontsize=14, fontweight='bold', color='blue', zorder=11)

# 添加图例
legend_elements = [
    mpatches.Patch(color='red', label='最优路径: 0 → 4 → 2 → 5'),
    mpatches.Patch(color='gray', label='其他可用边'),
    mpatches.Circle((0, 0), 0.3, color='red', label='起点/终点'),
    mpatches.Circle((0, 0), 0.3, color='orange', label='路径中间节点'),
    mpatches.Circle((0, 0), 0.3, color='lightblue', label='非路径节点')
]
ax.legend(handles=legend_elements, loc='upper left', fontsize=10)

# 添加统计信息
info_text = f'''路径统计:
总成本: 9.42 单位
体力消耗: 9.42 单位
行动点消耗: 3.0 单位
求解时间: 37.4 微秒'''

ax.text(6.5, 4.5, info_text, fontsize=9,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.8),
        verticalalignment='top')

plt.tight_layout()
plt.savefig('navmesh_path_visualization.png', dpi=150, bbox_inches='tight')
print("可视化图表已保存为: navmesh_path_visualization.png")
plt.show()
EOF

# 运行可视化脚本（需要matplotlib）
python visualize_graph.py
```

如果没有matplotlib，可以安装：
```bash
pip install matplotlib
```

---

## 7. 完整性检查

### 📸 截图13：验证所有文件

```bash
# 回到实验目录
cd c:/Users/DELL/Graduation/NavMeshERCA_Experiment/build/Release

# 列出所有相关文件
echo "=== NavMesh实验生成的文件 ==="
dir *.txt *.gr

# 进入ERCA*目录
cd c:/Users/DELL/Graduation/public_erca/build/Release

echo ""
echo "=== ERCA*求解器生成的文件 ==="
dir navmesh_*.gr navmesh_result.txt
```

### 📸 截图14：显示项目结构

```bash
# 显示项目树形结构（需要tree命令）
cd c:/Users/DELL/Graduation/NavMeshERCA_Experiment
tree /F /A

# 或者简化版本
dir /S /B *.cpp *.h *.txt *.gr
```

---

## 8. 性能对比测试（可选）

### 📸 截图15：多次运行测试稳定性

```bash
# 创建测试脚本
cat > benchmark.bat << 'EOF'
@echo off
echo ========================================
echo ERCA* 性能基准测试
echo ========================================
echo.

for /L %%i in (1,1,5) do (
    echo 运行测试 %%i/5...
    run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 result_%%i.txt > nul
    findstr "rt_search" result_%%i.txt
)

echo.
echo 测试完成！
EOF

# 运行基准测试
benchmark.bat
```

---

## 9. 清理和归档

### 📸 截图16：归档结果

```bash
# 创建结果目录
mkdir results_archive

# 复制所有结果文件
copy navmesh_*.gr results_archive\
copy navmesh_result.txt results_archive\
copy *.png results_archive\ 2>nul

# 显示归档内容
dir results_archive
```

---

## 快速命令清单

如果想一次性运行所有命令并保存输出：

```bash
# 创建自动运行脚本
cat > run_all.bat << 'EOF'
@echo off
echo ======================================== > output_log.txt
echo NavMesh to ERCA* 实验完整日志 >> output_log.txt
echo 运行时间: %date% %time% >> output_log.txt
echo ======================================== >> output_log.txt
echo. >> output_log.txt

echo [步骤1] 运行NavMesh实验程序... >> output_log.txt
cd c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release
navmesh_erca_experiment.exe >> output_log.txt 2>&1

echo. >> output_log.txt
echo [步骤2] 列出生成的文件... >> output_log.txt
dir *.txt *.gr >> output_log.txt

echo. >> output_log.txt
echo [步骤3] 运行ERCA*求解器... >> output_log.txt
cd c:\Users\DELL\Graduation\public_erca\build\Release
copy c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release\navmesh_*.gr . >> output_log.txt 2>&1
run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt >> output_log.txt 2>&1

echo. >> output_log.txt
echo [步骤4] 显示求解结果... >> output_log.txt
type navmesh_result.txt >> output_log.txt

echo. >> output_log.txt
echo ======================================== >> output_log.txt
echo 实验完成！ >> output_log.txt
echo ======================================== >> output_log.txt

move output_log.txt c:\Users\DELL\Graduation\NavMeshERCA_Experiment\
echo 日志已保存到: c:\Users\DELL\Graduation\NavMeshERCA_Experiment\output_log.txt
EOF

# 运行脚本
run_all.bat
```

---

## 截图建议

### 推荐的截图序列：

1. **环境信息** - 系统和编译器版本
2. **运行实验程序** - 完整的控制台输出
3. **文件列表** - 生成的所有文件
4. **图文件内容** - erca_input.txt 和 DIMACS文件
5. **ERCA*运行** - 求解器的完整输出
6. **结果文件** - navmesh_result.txt内容
7. **可视化图表** - 路径可视化（如果生成）
8. **项目结构** - 完整的文件树

### 截图工具推荐：

- **Windows自带**：Win + Shift + S
- **PowerToys**：更强大的截图工具
- **Snipaste**：带标注功能
- **ShareX**：自动上传和管理

### 注意事项：

- 确保终端字体大小适中（推荐12-14pt）
- 使用深色或浅色主题保持一致
- 重要输出可以用彩色高亮
- 每个截图包含完整的命令和输出
- 截图保存为PNG格式（清晰度高）

---

**文档创建时间**: 2026-01-30
**适用版本**: Windows 10/11
**预计截图数量**: 10-16张
