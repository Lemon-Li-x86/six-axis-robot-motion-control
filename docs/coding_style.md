# 工程代码与注释规范

适用范围：`firmware/freertos_demo/` 中的 C/H 文件、`simulation/` 与 `tools/` 下的 Python 脚本和测试。关联自动检查：[test_comment_style.py](../tools/tests/test_comment_style.py)、[test_repository_architecture.py](../tools/tests/test_repository_architecture.py)。

## 注释格式

| 类型 | 写法 | 说明重点 |
|---|---|---|
| C/H 文件头 | `/** @file ... @brief ... */`（按多行 Doxygen 排列） | 模块的职责和数据边界 |
| 公开 C API | `/** @brief ... @param[in/out] ... @return ... */` | 输入输出、单位、状态码、重要副作用 |
| 内部算法说明 | `/* ... */` | 推导依据、边界情形和数据一致性 |
| 结构体与常量 | 紧邻定义的简短 `/* ... */` | 数据单位、值域、来源 |
| Python 模块 | 模块首行文档字符串 `"""..."""` | 脚本用途、与其他模块的关系 |
| Python 函数 | 常规 docstring | 不直观的参数、返回数据和副作用 |
| 测试 | 简要说明断言语义 | 输入、期望行为、判定条件 |

文件和函数注释说明代码**做什么、为什么这样做**，避免重复翻译代码本身。日常实现注释采用正常陈述句，少用大写强调、装饰性长横线和宽泛的警告段落。涉及异步请求、碰撞检查和运动授权时，要准确描述先后关系：APF 提案、UART ACK 和物理到位分别代表不同状态。

空行用于区分逻辑段落；C/H 文件避免连续两个以上的空白行。保留结构体布局、公开符号和计算逻辑，不为了整理注释移动函数或重命名参数。

## 单一来源与调用层次

- 固件默认 PID、周期、速度、加速度、超时：`config/robot_defaults.json`。
- UR5 DH、坐标转换、关节限位：`model/ur5_model.c`。
- 仿真障碍与场景选择：`simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json`。
- 应用层调用算法、维护共享反馈；算法层不包含 UART、驱动或 FreeRTOS 头文件。
- `@param` 写清数组容量和单位；实际运动安全检查与规划候选计算分别描述。

## 验证

在仓库根目录运行：

```powershell
python -m unittest discover -s tools/tests -p "test_comment_style.py" -v
python -m unittest discover -s tools/tests -p "test_repository_architecture.py" -v
```

测试成功时输出 `OK`，返回码为 0。这两项是静态检查；嵌入式固件编译和 QEMU/PyBullet 运动验证仍按 [README](../README.md) 中的独立命令执行。
