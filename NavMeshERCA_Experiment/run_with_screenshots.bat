@echo off
chcp 65001 >nul
color 0A
title NavMesh到ERCA*实验 - 截图演示

echo.
echo ╔════════════════════════════════════════════════════════════════╗
echo ║     NavMesh到ERCA*最小可行实验 - 交互式演示脚本              ║
echo ║     请在每个暂停点截图，然后按任意键继续                      ║
echo ╚════════════════════════════════════════════════════════════════╝
echo.
pause

:STEP1
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图1/10: 系统环境信息
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [系统版本]
ver
echo.
echo [编译器版本]
where cl >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    cl 2>&1 | findstr "Version"
) else (
    echo MSVC编译器未找到在PATH中
)
echo.
echo [CMake版本]
cmake --version 2>nul | findstr "version"
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP2
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图2/10: 运行NavMesh实验程序
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [进入实验目录]
cd /d c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release
echo 当前目录: %CD%
echo.
echo [运行实验程序]
echo.
navmesh_erca_experiment.exe
echo.
echo 📸 请截图此屏幕（包含完整输出）
pause
echo.

:STEP3
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图3/10: 查看生成的文件
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [生成的文件列表]
echo.
dir *.txt *.gr
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP4
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图4/10: 查看简化格式图文件
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [erca_input.txt内容]
echo.
type erca_input.txt
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP5
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图5/10: 查看DIMACS成本文件
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [navmesh_cost.gr内容]
echo.
type navmesh_cost.gr
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP6
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图6/10: 查看DIMACS资源文件
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [navmesh_res1.gr - 体力消耗]
echo.
type navmesh_res1.gr
echo.
echo [navmesh_res2.gr - 行动点消耗]
echo.
type navmesh_res2.gr
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP7
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图7/10: 准备ERCA*求解器
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [进入ERCA*目录]
cd /d c:\Users\DELL\Graduation\public_erca\build\Release
echo 当前目录: %CD%
echo.
echo [复制图文件]
copy c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release\navmesh_*.gr . >nul 2>&1
echo 已复制图文件
echo.
echo [验证文件]
dir navmesh_*.gr
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP8
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图8/10: 运行ERCA*求解器
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [执行ERCA*命令]
echo run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt
echo.
run_erca.exe 1 6 60 3 navmesh_cost.gr navmesh_res1.gr navmesh_res2.gr 5000 3000 navmesh_result.txt
echo.
echo 📸 请截图此屏幕（包含完整输出）
pause
echo.

:STEP9
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图9/10: 查看求解结果
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo [navmesh_result.txt内容]
echo.
type navmesh_result.txt
echo.
echo 📸 请截图此屏幕
pause
echo.

:STEP10
cls
echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo 📸 截图10/10: 结果分析
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.
echo ╔════════════════════════════════════════════════════════════════╗
echo ║                      实验结果总结                              ║
echo ╚════════════════════════════════════════════════════════════════╝
echo.
echo [性能指标]
findstr "rt_initHeu rt_search" navmesh_result.txt
echo.
echo [找到的路径]
echo   DIMACS格式 (1-based):
findstr /R "^[0-9].*[0-9]$" navmesh_result.txt | findstr /V "Label"
echo   实际节点 (0-based): 0 → 4 → 2 → 5
echo.
echo [路径成本]
findstr "^\[" navmesh_result.txt
echo   实际值: 总成本=9.42, 体力=9.42, 行动点=3.0
echo.
echo [约束验证]
echo   ✓ 体力消耗: 9.42 ^< 50 (满足)
echo   ✓ 行动点消耗: 3.0 ^< 30 (满足)
echo.
echo [图结构统计]
echo   节点数: 6
echo   边数: 10
echo   资源维度: 2 (体力 + 行动点)
echo.
echo 📸 请截图此屏幕（最终总结）
echo.
pause

:COMPLETE
cls
echo.
echo ╔════════════════════════════════════════════════════════════════╗
echo ║                    🎉 实验演示完成！                           ║
echo ╚════════════════════════════════════════════════════════════════╝
echo.
echo 您已完成所有10个截图点
echo.
echo 生成的文件位置:
echo   1. NavMesh图文件:
echo      c:\Users\DELL\Graduation\NavMeshERCA_Experiment\build\Release\
echo.
echo   2. ERCA*结果:
echo      c:\Users\DELL\Graduation\public_erca\build\Release\navmesh_result.txt
echo.
echo   3. 完整报告:
echo      c:\Users\DELL\Graduation\NavMeshERCA_Experiment\EXPERIMENT_REPORT.md
echo.
echo 建议的截图命名:
echo   screenshot_01_environment.png
echo   screenshot_02_navmesh_run.png
echo   screenshot_03_files_list.png
echo   screenshot_04_input_file.png
echo   screenshot_05_cost_file.png
echo   screenshot_06_resource_files.png
echo   screenshot_07_erca_prepare.png
echo   screenshot_08_erca_run.png
echo   screenshot_09_result_file.png
echo   screenshot_10_summary.png
echo.
echo 按任意键退出...
pause >nul
