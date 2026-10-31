# 第五阶段参数登记簿（初始版，2026-10-27）

该文档继承 `docs/part4_archive/00_parameter_register.md` 的单一来源规则。第五阶段**不得**为UR5 DH、角度编码、PID、关节安全限位再复制一份参数。

## 当前已实现的C人工势场 API

| 符号 / 数据 | 语义、单位 | 定义位置 | 分类 / 作用 |
|---|---|---|---|
| `ROBOT_APF_MAX_OBSTACLES` | 一次静态规划最多8个长方体 | `algorithms/avoidance/potential_field.h` | 固定接口容量，不是作业场景障碍物数量 |
| `robot_apf_point_t.mm[3]` | 基座坐标系末端点，mm | 同一API头文件 | 每次调用输入/输出，不缓存运行态 |
| `robot_apf_aabb_t` | 静态长方体min/max，mm | 同一API头文件 | 输入障碍物的几何描述，不是UR5本体DH |
| `robot_apf_config_t.attraction_gain` | 吸引项权重 | 同一API头文件声明字段 | 调用者注入的策略，不在算法文件设默认值 |
| `repulsion_gain` | 距离势场排斥项权重 | 同上 | 仅用于C APF中的方向求解 |
| `influence_distance_mm` | 障碍物排斥影响范围，mm | 同上 | 正值，由调用者配置 |
| `clearance_mm` | 末端点到长方体膨胀边界的额外余量，mm | 同上 | 仅点级别余量，**不是整臂碰撞安全保证** |
| `maximum_step_mm` | 单步位移上限，mm | 同上 | 限制APF航点变化，不能代替关节速度/加速度门禁 |
| `goal_tolerance_mm` | 目标可接受距离，mm | 同上 | 达标后保持原位置，避免超过步长强制吸附 |
| `robot_apf_step_info_t` | 最近障碍表面距离、选定步长、目标到达标记 | 同一API头文件 | 每次诊断输出，不存全局状态 |

**定义关系**：算法所有策略值均来自 `robot_apf_step(...,config,...)` 实参。生产固件当前尚无 Stage5 动态应用配置或默认增益，因此不在Model或Control Task中添加第二套常数。测试源码中的 `normal` / PyBullet 中的 `Config(...)` 是**不同测试场景的夹具输入**，不属于上线控制参数。

## 静态场景夹具与一致性

- `simulation/pybullet_ur5/src/stage5_static_obstacle_probe.py`：试验长方体尺寸、相对EE位置、起终点偏移量仅在该文件声明；运行时通过 `pybullet.getAABB()` 获得实际场景几何，再转 mm 传给 **生产C APF源码** 的宿主共享库，不手抄另一个障碍 AABB。
- `tests/avoidance/potential_field_test_main.c`：边界故障用例的AABB、控制参数是独立测试输入；不用于实际场景。
- `algorithms/avoidance/potential_field.c`：不保存障碍物、参数、当前运动状态，禁止 `static` 可变运行态。
- `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` 仍默认 OFF，Stage5此次未连接Driver/FreeRTOS。

## 未解决且不得伪装的事实

- 末端点避免AABB穿越，**不等于**六个连杆避障；
- APF单步不会自动保证目标可达、IK连续、关节运动安全，也不能解决局部极小；
- 实际碰撞检测需要在PyBullet中对连杆/环境逐个检验，再在嵌入式轨迹边界重做策略校验；
- 随着Stage5通信协议扩展，当前Python/C协议镜像一致性检查需进一步升级为单一wire schema；不能凭字面相同视为串口功能联调。


## 2026-10-31 后续接口权威位置（不重复定义参数值）

此登记簿上方章节是 2026-10-27 的历史首批切片状态；不可用“尚未接入FreeRTOS”描述目前 Stage5 的真实双向 QEMU UART 仿真。最新范围与签收见 [17_stage5_final_acceptance_2026-10-31.md](17_stage5_final_acceptance_2026-10-31.md)。Stage6 继承以下**唯一权威来源**，不得复制新默认值：

| 类型 | 唯一权威位置 | 分层要求 |
|---|---|---|
| 固件 PID/任务周期/电机动作开关 | `firmware/freertos_demo/config/robot_defaults.json` 与其既有生成产物 | 控制参数不写入 GUI 或场景脚本；实体电机命令默认 OFF |
| UR5 几何/关节限位/正逆运动学 | `firmware/freertos_demo/model/` 与加载的 URDF、生产 C IK | GUI 不扩大 URDF 或 C 的行程；运行时核对一致性 |
| C APF 策略/IK 精度上限 | `algorithms/avoidance/` 与生产 C IK/FK 接口 | Python 只负责传递场景输入并执行几何门禁，不写同义第二套 APF |
| 静态障碍和路径策略 | `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json` | 测试具名场景是输入，不注入控制任务或算法内部默认 |
| UART 编码/序列和反馈 | 生产固件通信协议及 `qemu_uart_receiver.py` 一致性测试 | ACK 是请求确认，不能代替 PyBullet 实际到位 |
| GUI/JOG 的观察与门禁阈值 | `stage5_qemu_uart_visual_demo.py`、`stage5_safe_gui_jog.py` 当前既有常量 | 单独的仿真测试/观测策略；不要在报告、Stage6 app 重复定义数值 |

CPU/ARM map 量测按 [15 号技术验收](15_stage5_exit_review_2026-10-30.md) 和最终签收记录归档；任何 Stage6 扩展先查本登记簿，并分离常量、派生量与运行态。
