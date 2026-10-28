# 第四阶段收尾：待验收 PID 候选配置

## 来源和变更边界

本次只在候选分支 candidate/stage4-pid-20261027 的单一配置文件
firmware/freertos_demo/config/robot_defaults.json 中修改六轴仿真默认：

| 参数 | 原默认 | 试验候选 |
|---|---:|---:|
| 每轴 Kp | 0.2 | 0.25 |
| 每轴 Ki | 0 | 0 |
| 每轴 Kd | 0 | 0.025 |

上述改动不影响 Stage4 冻结版本 backup/stage4-verified-2026-10-27、
主线 main，也不改变算法模块中任何公式。限速、加速度、积分与输出
限幅依然来自同一 JSON。

参数是**UR5/PyBullet 软件仿真候选**，不等于实物电机的安全或最佳 PID。

## 选择依据

来自 review/stage4-closure-evidence-20261027 的增益搜索：
- 先以六轴5°运动、模拟执行器输出上限150及60 N·m评估候选，
  再用未用于排序的 J2 ±2°场景交叉复核；
- 搜索中 Kp 等于原权威默认的1.25倍，Ki=0，Kd=0.025；
- 六组观测全部落在原小步测试门限内；最坏门限比为0.76733；
- 原小步指标阈值是最终误差0.25°、J2 RMSE 0.25°、J2超调0.15°。
  将同一门限扩展到5°是**更严格的软件对照策略**，并不代表工业规格。

云端原始结果：
[增益搜索及保留工况验证](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40491927341)

注意：这些工况尚没有证明任何增益在所有工作空间或实体负载下都稳定。
如果新的统一 CI 或 QEMU 闭环回归失败，该候选应当保留为“尚未批准”，
不得以定性感觉或肉眼运动顺畅替代回归记录。

## Windows 本机一次性复验

在仓库根目录先确认 git status --short 没有需要保护的未提交改动：

~~~powershell
git fetch origin
git switch --track origin/candidate/stage4-pid-20261027
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline -IncludeGuiReplay
~~~

如果已有本地候选分支，改用：

~~~powershell
git switch candidate/stage4-pid-20261027
git pull --ff-only
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline -IncludeGuiReplay
~~~

该入口验证：原 Stage4 Host 算法测试、C 桥接 smoke、18组敏感度矩阵、
搜索及保留工况、已有 QEMU/FreeRTOS Stage4 自检。实验产物写入
build/stage4-closure-local/evidence/，QEMU 原回归日志仍在原工具定义路径。

**单独复验真实 ARM QEMU UART ↔ PyBullet 多段闭环**
（不同于本工具的原生 C 算法桥接）：

~~~powershell
cmake -S . -B build/stage4-candidate-local-uart -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake' -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON -DROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
cmake --build build/stage4-candidate-local-uart --target freertos_demo
.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_trace_replay.py --elf build/stage4-candidate-local-uart/firmware/freertos_demo/freertos_demo.elf --scenario six_axis --output-dir build/stage4-candidate-local-gui
~~~

GUI 是**真实 QEMU/FreeRTOS/PyBullet DIRECT 实测角度的独立回放**，
不声称 GUI 与实时固件控制同线程运行。执行前必须确认本候选分支的
GitHub Actions 中 ARM/QEMU 端到端回归已经完成并成功，不能先写结论。

## 云端 QEMU 候选版本正式回归（2026-10-27）

[GitHub Actions Run #40492735418](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40492735418) 已在候选增益 Kp=0.25、Ki=0、Kd=0.025 下完成：
- Stage4 原生 C 与 PyBullet 测试、原有 Host 6/6、C桥接Smoke：成功；
- 重新编译的 ARM QEMU FreeRTOS UART 一次关节运动：PASS，最大最终关节误差0.01°；
- 真实 QEMU UART ↔ PyBullet DIRECT 三段六轴运动：3/3完成，677目标帧，最大分段最终误差0.08°，控制和规划故障计数均0；
- 试验配置仍只在本候选分支，冻结基线和主线未更改。

以上是 Linux GitHub Actions 实测，不是对 Windows GUI 的观察。Windows 本地仍需运行上面的脚本，带 -IncludeGuiReplay 时，它会**重新编译候选 ELF**，录制真正的 UART 物理闭环，再独立慢速回放实测姿态。不得复用旧分支下的 ELF。

## 保留的工程限制

- 已实现并有回归：三次/五次、梯形、笛卡尔直线/圆弧的算法，
  原生 C/PyBullet 关节控制、FreeRTOS 调度。
- 待第五阶段：笛卡尔直线/圆弧完整 UART 提交、末端空间路径的
  真实系统级物理误差验证、复杂工况碰撞检查。
- 长时间高负载、整系统 CPU 利用率、真实硬件 WCET、实物伺服与
  急停安全没有得到本次试验认证。

**阶段正式封版前门槛**：基线回归通过、候选新增回归通过、
本机可以复现、所有例外和证据归档。不能因程序可以运行就把
阶段4写成“实物工业控制验收通过”。
