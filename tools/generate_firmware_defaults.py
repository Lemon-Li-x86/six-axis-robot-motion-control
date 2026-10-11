
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
文件：generate_firmware_defaults.py

用途：
1. 读取 robot_defaults.json。
2. 检查配置字段是否完整。
3. 检查各项参数的数据类型与取值范围。
4. 生成供 Cortex-M4 固件编译使用的 C 头文件。

设计原则：
- JSON 只在开发计算机上读取。
- Cortex-M4 固件不需要 JSON 解析器。
- 不依赖第三方 Python 软件包。
- 配置非法时停止生成，避免错误参数进入固件。
"""

import argparse
import json
from pathlib import Path


# ==========================================================
# Configuration Schema
# ==========================================================

# 当前 FreeRTOSConfig.h 设置：
# configMAX_PRIORITIES = 5
# 因此有效任务优先级为 0~4。
MAX_TASK_PRIORITY = 4

PRIORITY_NAMES = (
    "control",
    "protocol_rx",
    "trajectory",
    "protocol_tx",
    "monitor",
)

PERIOD_NAMES = (
    "control",
    "trajectory",
    "monitor",
    "protocol_tx",
)

STACK_NAMES = PRIORITY_NAMES

EXPECTED_KEYS = {
    "schema_version",
    "task_priority",
    "task_period_ms",
    "task_stack_depth_words",
    "timer_frequency_hz",
    "parser_benchmark_samples",
    "initial_joint_targets_centideg",
}


# ==========================================================
# Validation Helpers
# ==========================================================

def validate_integer(value, name, minimum, maximum):
    """
    验证参数必须为整数且处于指定范围。

    显式使用 type(value) is int，
    避免 Python 将 bool 当作整数接受。
    """
    if type(value) is not int:
        raise ValueError(
            f"{name} must be an integer"
        )

    if not minimum <= value <= maximum:
        raise ValueError(
            f"{name} must be within "
            f"[{minimum}, {maximum}]"
        )

    return value


def validate_section(
    config,
    section_name,
    expected_names,
    minimum,
    maximum,
):
    """
    验证一个配置分组。

    要求：
    1. 必须是 JSON Object。
    2. 字段不能缺失。
    3. 不允许出现未知字段。
    4. 每个字段必须满足取值范围。
    """
    section = config[section_name]

    if not isinstance(section, dict):
        raise ValueError(
            f"{section_name} must be an object"
        )

    if set(section) != set(expected_names):
        raise ValueError(
            f"Invalid fields in {section_name}"
        )

    return {
        name: validate_integer(
            section[name],
            f"{section_name}.{name}",
            minimum,
            maximum,
        )
        for name in expected_names
    }


# ==========================================================
# Generate Header
# ==========================================================

def generate_header(config):
    """
    根据 JSON 参数生成完整 C 头文件。

    返回：
        str：可直接保存为 .h 的源代码。
    """
    if not isinstance(config, dict):
        raise ValueError(
            "Configuration root must be an object"
        )

    if set(config) != EXPECTED_KEYS:
        raise ValueError(
            "Configuration has missing or unknown fields"
        )

    if (
        type(config["schema_version"]) is not int
        or config["schema_version"] != 1
    ):
        raise ValueError(
            "Unsupported configuration schema version"
        )

    priorities = validate_section(
        config,
        "task_priority",
        PRIORITY_NAMES,
        0,
        MAX_TASK_PRIORITY,
    )

    periods = validate_section(
        config,
        "task_period_ms",
        PERIOD_NAMES,
        1,
        60000,
    )

    # 栈深度单位是 StackType_t 元素数量，
    # 不是字节。这里为当前工程设置保守范围。
    stacks = validate_section(
        config,
        "task_stack_depth_words",
        STACK_NAMES,
        128,
        4096,
    )

    timer_frequency = validate_integer(
        config["timer_frequency_hz"],
        "timer_frequency_hz",
        1,
        1000,
    )

    benchmark_samples = validate_integer(
        config["parser_benchmark_samples"],
        "parser_benchmark_samples",
        1,
        100000,
    )

    # 固件公共关节角使用 int16_t：
    # 1 unit = 0.01 degree。
    # 规范角度范围为 [-180°, 180°)。
    joint_targets = config[
        "initial_joint_targets_centideg"
    ]

    if (
        not isinstance(joint_targets, list)
        or len(joint_targets) != 6
    ):
        raise ValueError(
            "Exactly six joint targets are required"
        )

    for index, value in enumerate(joint_targets):
        validate_integer(
            value,
            f"joint_targets[{index}]",
            -18000,
            17999,
        )

    lines = [
        "/*",
        " * Auto-generated firmware configuration.",
        " * Source: config/robot_defaults.json",
        " * Do not edit this generated file.",
        " */",
        "#ifndef FIRMWARE_DEFAULTS_GENERATED_H",
        "#define FIRMWARE_DEFAULTS_GENERATED_H",
        "",
    ]

    # FreeRTOS Task Priorities
    for name in PRIORITY_NAMES:
        lines.append(
            f"#define FW_DEFAULT_TASK_PRIORITY_"
            f"{name.upper()} {priorities[name]}U"
        )

    lines.append("")

    # FreeRTOS Task Periods
    for name in PERIOD_NAMES:
        lines.append(
            f"#define FW_DEFAULT_TASK_PERIOD_"
            f"{name.upper()}_MS {periods[name]}U"
        )

    lines.append("")

    # FreeRTOS Task Stack Depths
    for name in STACK_NAMES:
        lines.append(
            f"#define FW_DEFAULT_TASK_STACK_"
            f"{name.upper()}_WORDS {stacks[name]}U"
        )

    lines.append("")

    lines.append(
        "#define FW_DEFAULT_TIMER_FREQUENCY_HZ "
        f"{timer_frequency}U"
    )

    lines.append(
        "#define FW_DEFAULT_PARSER_BENCHMARK_SAMPLES "
        f"{benchmark_samples}U"
    )

    # Initial Joint Targets
    for index, value in enumerate(joint_targets):
        lines.append(
            f"#define FW_DEFAULT_JOINT_{index + 1}"
            f"_TARGET_CENTIDEG ({value})"
        )

    lines.extend([
        "",
        "#endif /* FIRMWARE_DEFAULTS_GENERATED_H */",
        "",
    ])

    return "\n".join(lines)


# ==========================================================
# Program Entry
# ==========================================================

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Validate firmware JSON configuration "
            "and generate a C header."
        )
    )

    parser.add_argument(
        "--input",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--output",
        type=Path,
        required=True,
    )

    args = parser.parse_args()

    try:
        config = json.loads(
            args.input.read_text(encoding="utf-8")
        )

        generated_header = generate_header(config)

        args.output.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        args.output.write_text(
            generated_header,
            encoding="utf-8",
        )

    except (
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        parser.error(str(error))

    print(
        f"Configuration generated: {args.output}"
    )


if __name__ == "__main__":
    main()
