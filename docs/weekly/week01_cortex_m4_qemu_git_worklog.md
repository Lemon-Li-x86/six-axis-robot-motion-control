# Week 1 工作记录：Cortex-M4 / QEMU 裸机环境搭建与 Git 工程管理

> 项目：六轴工业机器人嵌入式运动控制固件开发与仿真验证  
> 日期：2026-09-28  

---

## 1. 当前项目目标

项目总体目标是基于 **Cortex-M4 + FreeRTOS + Python/PyBullet**，逐步完成六轴工业机器人的嵌入式运动控制固件与仿真验证。

当前理解的系统链路：

```text
Cortex-M4 / QEMU
        ↓
     FreeRTOS
        ↓
嵌入式控制固件（C）
        ↓
UART 二进制通信协议
        ↓
     Python
        ↓
    PyBullet
        ↓
   UR5 / 六轴机器人
```

固件侧主要负责：

```text
驱动层
├── UART
├── GPIO
└── Timer
        ↓
通信层
├── 二进制协议
└── 命令解析
        ↓
算法层
├── 运动学
├── 轨迹规划
└── PID
        ↓
应用层
└── 机器人任务逻辑
```

目前只推进到 **Cortex-M4 裸机环境、QEMU 仿真、UART 输出和 Git 工程管理**，FreeRTOS、PyBullet 和正式二进制协议尚未开始实现。

---

## 2. 到目前为止实际完成的内容

### 2.1 GitHub 项目初始化

已建立私有 GitHub 仓库：

```text
six-axis-robot-motion-control
```

项目 README 已包含：

- 项目目标；
- Week 1 Goals；
- Cortex-M4 / QEMU 环境进度说明。

---

### 2.2 Cortex-M4 基础概念梳理

已完成第一轮理解：

- ARM 是 CPU 架构和内核设计体系；
- Cortex-M 是面向 MCU 的 ARM 内核系列；
- Cortex-M4 是本项目关注的 CPU 内核；
- MCU 不等于 CPU，MCU 通常还包含 Flash、RAM、UART、GPIO、Timer 等；
- STM32F4 是大量采用 Cortex-M4 内核的一类 MCU；
- QEMU 用于在 PC 上模拟 ARM/Cortex-M 平台，从而在没有真实开发板的情况下先运行固件。

关系可粗略表示为：

```text
ARM
└── Cortex-M
    ├── M0
    ├── M3
    ├── M4
    └── M7

STM32F4 MCU
├── Cortex-M4 CPU Core
├── Flash
├── RAM
├── UART
├── GPIO
├── Timer
└── 其他外设
```

---

### 2.3 ARM GNU Toolchain 安装与 PATH 配置

安装了 **Arm GNU Toolchain**，实际编译器路径位于类似：

```text
D:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin
```

首先使用完整路径验证：

```powershell
"D:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc.exe" --version
```

成功输出：

```text
arm-none-eabi-gcc.exe (Arm GNU Toolchain 15.3.Rel1 ...)
```

随后将 `bin` 目录加入 Windows 用户 PATH。

验证命令：

```powershell
arm-none-eabi-gcc --version
```

含义：

> 在 Windows/x86 PC 上运行 ARM 交叉编译器，并生成给 ARM Cortex-M 使用的程序。

---

### 2.4 QEMU 安装与 Cortex-M4 板型验证

安装 QEMU 后，将包含：

```text
qemu-system-arm.exe
```

的目录加入 Windows PATH。

验证：

```powershell
qemu-system-arm --version
```

确认 QEMU 可正常调用。

随后检查 QEMU 支持的 MPS2 平台：

```powershell
qemu-system-arm -machine help | findstr mps2
```

确认存在：

```text
mps2-an386
```

因此当前项目选择：

```text
QEMU mps2-an386
```

作为 Cortex-M4 仿真平台。

---

## 3. 第一个 Cortex-M4 裸机程序

当前裸机测试工程：

```text
firmware/
└── hello/
    ├── main.c
    ├── startup.s
    ├── linker.ld
    └── hello.elf   # 本地编译产物，不提交 Git
```

三个源码/配置文件的职责：

```text
startup.s
CPU Reset 后如何进入 main()

main.c
真正的 C 程序逻辑

linker.ld
程序代码和数据应该放进哪块内存
```

---

## 4. `main.c`：UART 输出测试程序

当前代码：

```c
#include <stdint.h>

/* UART0 外设的基地址。
 * 对这个 QEMU mps2-an386 平台来说，UART0 的寄存器从这个地址开始。
 */
#define UART0_BASE 0x40004000UL

/* UART0 数据寄存器：
 * 往这里写数据，就相当于让 UART 发送字符。
 */
#define UART0_DATA    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))

/* UART0 状态寄存器：
 * 可以用来查看发送缓冲区是否已满等状态。
 */
#define UART0_STATE   (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

/* UART0 控制寄存器：
 * 用来开启或关闭 UART 的发送、接收功能。
 */
#define UART0_CTRL    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))

/* UART0 波特率分频寄存器：
 * 用于设置串口通信速率相关参数。
 */
#define UART0_BAUDDIV (*(volatile uint32_t *)(UART0_BASE + 0x010UL))

/* UART_STATE 的 bit0：
 * 为 1 时表示发送缓冲区已满。
 */
#define UART_TX_FULL (1U << 0)

/* 初始化 UART0 */
static void uart_init(void)
{
    /* 设置波特率分频值。
     * 当前用于最小测试。
     */
    UART0_BAUDDIV = 16U;

    /* CTRL bit0 = 1，开启 UART 发送功能。 */
    UART0_CTRL = 1U;
}

/* 发送一个字符 */
static void uart_putc(char c)
{
    /* 如果发送缓冲区满了，就一直等待。 */
    while (UART0_STATE & UART_TX_FULL)
    {
    }

    /* 把字符写入 UART 数据寄存器。 */
    UART0_DATA = (uint32_t)c;
}

/* 发送一整个字符串 */
static void uart_puts(const char *s)
{
    /* 逐个读取字符串中的字符，
     * 直到遇到字符串结尾 '\0'。
     */
    while (*s)
    {
        uart_putc(*s++);
    }
}

/* C 程序入口 */
int main(void)
{
    /* 先初始化 UART。 */
    uart_init();

    /* 通过 UART 输出测试字符串。 */
    uart_puts("Hello from Cortex-M4!\r\n");

    /* 嵌入式程序一般不会“运行完退出”，
     * 所以这里保持无限循环。
     */
    while (1)
    {
    }
}
```

### 4.1 当前理解

这里第一次实际接触到 **Memory-Mapped I/O（内存映射外设）**。

例如：

```c
UART0_DATA = (uint32_t)c;
```

并不是普通变量赋值，而是在向 UART 外设对应的寄存器地址写数据。

`volatile` 的作用可以先理解为：

> 这个地址中的内容可能由硬件改变，编译器不要擅自省略相关读写操作。

---

## 5. `startup.s`：Cortex-M4 启动代码

当前代码：

```asm
.syntax unified

/* 指定目标 CPU 为 Cortex-M4 */
.cpu cortex-m4

/* Cortex-M 系列只运行 Thumb 指令 */
.thumb

/* 把这两个符号暴露给链接器 */
.global Reset_Handler
.global _estack

/* 中断向量表 */
.section .isr_vector, "a", %progbits

/* 按 4 字节对齐 */
.align 2

/* 向量表第一个值：
 * CPU 复位后使用的初始栈顶地址
 */
.word _estack

/* 向量表第二个值：
 * CPU 复位后首先执行的函数地址
 */
.word Reset_Handler

/* Reset_Handler 代码区域 */
.section .text.Reset_Handler

/* 告诉汇编器 Reset_Handler 是一个函数 */
.type Reset_Handler, %function

/* 明确说明这是 Thumb 函数 */
.thumb_func

Reset_Handler:
    /* 跳转到 C 语言的 main() 函数 */
    bl main

LoopForever:
    /* 如果 main() 意外返回，就停在这里无限循环 */
    b LoopForever
```

### 5.1 启动流程

第一轮只需要理解：

```text
CPU Reset
    ↓
读取 Vector Table
    ↓
获得初始栈顶 _estack
    ↓
找到 Reset_Handler
    ↓
Reset_Handler 调用 main()
```

其中 `.thumb_func` 是后来排错时补上的重要设置，用于明确 `Reset_Handler` 是 Thumb 函数。

---

## 6. `linker.ld`：链接脚本

当前代码：

```ld
/* 指定程序入口函数 */
ENTRY(Reset_Handler)

/* 定义目标系统中的内存区域 */
MEMORY
{
    /* FLASH：
     * 用来放程序代码和只读数据。
     * rx = readable + executable
     */
    FLASH (rx) :
        ORIGIN = 0x00000000,
        LENGTH = 4M

    /* RAM：
     * 用来放运行时数据。
     * rwx = readable + writable + executable
     */
    RAM (rwx) :
        ORIGIN = 0x20000000,
        LENGTH = 4M
}

/* 设置初始栈顶地址。
 * 栈从 RAM 顶部开始向下增长。
 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 指定不同代码/数据段应该放到哪里 */
SECTIONS
{
    /* 中断向量表放到 FLASH 最前面 */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    /* 程序代码和只读数据放到 FLASH */
    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* 已初始化的全局/静态变量放到 RAM */
    .data :
    {
        *(.data*)
    } > RAM

    /* 未初始化的全局/静态变量放到 RAM */
    .bss :
    {
        *(.bss*)
        *(COMMON)
    } > RAM
}
```

### 6.1 当前理解

链接脚本用于回答：

> 程序的不同部分最终应该放进 MCU 的哪块内存？

当前粗略关系：

```text
FLASH
├── .isr_vector
├── .text
└── .rodata

RAM
├── .data
├── .bss
└── stack
```

---

## 7. 交叉编译命令

实际使用：

```powershell
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -nostdlib -T linker.ld startup.s main.c -o hello.elf
```

参数含义：

| 参数 | 含义 |
|---|---|
| `arm-none-eabi-gcc` | ARM 裸机交叉编译器 |
| `-mcpu=cortex-m4` | 目标 CPU 为 Cortex-M4 |
| `-mthumb` | 使用 Thumb 指令集 |
| `-nostdlib` | 不使用普通桌面程序的标准启动和链接环境 |
| `-T linker.ld` | 使用自定义链接脚本 |
| `startup.s main.c` | 参与编译/链接的源文件 |
| `-o hello.elf` | 输出 ARM ELF 文件 |

成功后生成：

```text
hello.elf
```

---

## 8. QEMU 运行命令

实际使用：

```powershell
qemu-system-arm -M mps2-an386 -kernel hello.elf -serial stdio -monitor none
```

参数含义：

| 参数 | 含义 |
|---|---|
| `qemu-system-arm` | 启动 ARM 系统模拟器 |
| `-M mps2-an386` | 模拟 Cortex-M4 的 MPS2-AN386 平台 |
| `-kernel hello.elf` | 加载刚刚编译出的 ARM 固件 |
| `-serial stdio` | 把虚拟串口输出连接到当前终端 |
| `-monitor none` | 关闭 QEMU monitor，避免和 UART 输出抢终端 |

最终实际输出：

```text
Hello from Cortex-M4!
```

这证明当前链路：

```text
main.c
   ↓
ARM 交叉编译
   ↓
hello.elf
   ↓
QEMU / Cortex-M4
   ↓
UART
   ↓
终端输出
```

已实际跑通。

---

## 9. VS Code 工程化配置

使用 VS Code 管理整个工程。

VS Code 当前承担：

```text
项目文件管理
+ C/汇编/链接脚本编辑
+ ARM IntelliSense
+ 集成 PowerShell
+ Build/Run Task
+ Git Source Control
```

### 9.1 IntelliSense

将 VS Code C/C++ 扩展的 Compiler Path 指向：

```text
arm-none-eabi-gcc.exe
```

从而让编辑器能够识别 ARM 工具链中的：

```c
#include <stdint.h>
```

等头文件。

---

## 10. `.vscode/tasks.json`

当前任务配置：

```jsonc
{
    "version": "2.0.0",

    "tasks": [
        {
            // 编译 Cortex-M4 程序
            "label": "Build Cortex-M4",
            "type": "shell",

            // ARM 交叉编译器
            "command": "arm-none-eabi-gcc",

            "args": [
                "-mcpu=cortex-m4",
                "-mthumb",
                "-nostdlib",

                "-T",
                "linker.ld",

                "startup.s",
                "main.c",

                "-o",
                "hello.elf"
            ],

            // 编译命令在 hello 文件夹里执行
            "options": {
                "cwd": "${workspaceFolder}/firmware/hello"
            },

            // 识别 GCC 编译报错
            "problemMatcher": "$gcc"
        },

        {
            // 编译完成后，在 QEMU 中运行
            "label": "Run Cortex-M4",
            "type": "shell",

            "command": "qemu-system-arm",

            "args": [
                "-M",
                "mps2-an386",

                "-kernel",
                "hello.elf",

                "-serial",
                "stdio",

                "-monitor",
                "none"
            ],

            "options": {
                "cwd": "${workspaceFolder}/firmware/hello"
            },

            // 先执行 Build Cortex-M4
            "dependsOn": "Build Cortex-M4"
        }
    ]
}
```

作用：

```text
Run Cortex-M4
      ↓
自动执行 Build Cortex-M4
      ↓
arm-none-eabi-gcc 编译
      ↓
生成 hello.elf
      ↓
QEMU 启动
      ↓
UART 输出
```

---

## 11. F6 快捷运行

配置 VS Code 快捷键后，可使用：

```text
F6
```

完成：

```text
保存代码
↓
F6
↓
自动编译
↓
自动启动 QEMU
↓
查看 UART 输出
```

停止当前 QEMU 可在终端使用：

```text
Ctrl + C
```

---

## 12. `.gitignore`

当前内容：

```gitignore
# 编译产物
*.elf
*.o
*.bin
*.hex

# 构建目录
build/
```

目的：

```text
源码、文档、工程配置
        ↓
       Git

编译出来的临时文件
        ↓
      不提交
```

因此 `hello.elf` 只存在本地，不应进入远程仓库。

---

# 13. Git 工作流

当前正常开发流程：

```text
开始工作
↓
git pull
↓
修改代码 / 文档
↓
git status
↓
git add
↓
git commit
↓
git push
```

常用命令如下。

---

## 13.1 查看仓库状态

```powershell
git status
```

作用：

> 查看哪些文件被修改、哪些已暂存、哪些尚未被 Git 跟踪。

---

## 13.2 暂存文件

```powershell
git add .
```

作用：

> 把当前准备提交的改动放进 staging area（暂存区）。

---

## 13.3 提交版本

```powershell
git commit -m "Add Cortex-M4 QEMU hello example"
```

作用：

> 把当前暂存区保存为一个 Git commit。

---

## 13.4 推送到 GitHub

```powershell
git push origin main
```

作用：

> 将本地 `main` 分支的新 commit 推送到 GitHub 远程仓库。

---

# 14. 这次非常艰难的 Git 推库过程

本次并不是一次顺滑的：

```text
git add
git commit
git push
```

而是完整经历了一次真实的 Git 分支同步和冲突处理。

---

## 14.1 第一个问题：Git 不知道提交者是谁

第一次 commit 时 Git 提示未配置用户身份。

设置：

```powershell
git config --global user.name "..."
git config --global user.email "..."
```

检查：

```powershell
git config --global user.name
git config --global user.email
```

---

## 14.2 第二个问题：同一个文件既 staged 又 modified

曾出现：

```text
Changes to be committed
    main.c

Changes not staged for commit
    main.c
```

原因是：

```text
修改 main.c
↓
git add
↓
暂存版本 A
↓
又继续修改 main.c
↓
工作区变成版本 B
```

因此 Git 同时看到：

```text
暂存区：版本 A
工作区：版本 B
```

当时使用：

```powershell
git restore --staged .
```

清空暂存区但保留本地文件修改，然后重新：

```powershell
git add .
```

---

## 14.3 第三个问题：`hello.elf` 被误加入暂存区

由于 `hello.elf` 是编译产物，不应该进入 Git。

因此加入 `.gitignore`：

```gitignore
*.elf
*.o
*.bin
*.hex

build/
```

随后重新整理暂存区，确认：

```powershell
git status
```

中不再出现 `hello.elf`。

---

## 14.4 第四个问题：`git push` 被拒绝

出现：

```text
! [rejected] main -> main (fetch first)
```

以及：

```text
non-fast-forward
```

原因：

> 在本地 clone 之后，GitHub 网页端又发生了新的 commit，因此远端 `main` 比本地更靠前，Git 不允许本地直接覆盖远端历史。

解决思路：

```text
先获取并整合远端
↓
再重新推送本地提交
```

使用：

```powershell
git pull --rebase origin main
```

---

## 14.5 第五个问题：`.gitignore` 冲突

rebase 时出现：

```text
both added: .gitignore
```

原因：

> 远端和本地都各自新增了一份 `.gitignore`，Git 无法自动决定保留哪份。

使用 VS Code 手动整理 `.gitignore`，保留需要的规则，然后：

```powershell
git add .gitignore
```

检查：

```powershell
git status
```

确认没有：

```text
Unmerged paths
```

之后继续：

```powershell
git rebase --continue
```

---

## 14.6 第六个问题：突然被 Vim 劫持

`git rebase --continue` 后，Git 打开了 Vim 要求确认 commit message。

看到：

```text
-- INSERT --
```

使用：

```text
Esc
:wq
Enter
```

保存并退出。

过程中还遇到：

```text
E163: There is only one file to edit
Press ENTER or type command to continue
```

最终再次：

```text
Enter
Esc
:q
Enter
```

退出 Vim。

为了以后不再被 Vim 突然绑架，设置：

```powershell
git config --global core.editor "code --wait"
```

之后 Git 需要编辑提交信息时默认使用 VS Code。

---

## 14.7 最终成功推送

rebase 完成后：

```powershell
git push origin main
```

最终成功。

远端最新相关 commit：

```text
Add Cortex-M4 QEMU hello example
```

此时远端仓库中已经包含：

```text
firmware/hello/main.c
firmware/hello/startup.s
firmware/hello/linker.ld
.vscode/tasks.json
.gitignore
```

而 `hello.elf` 没有进入仓库。

---

# 15. 本阶段踩坑记录

## 15.1 ARM GCC 安装成功但命令不可用

现象：

```text
'arm-none-eabi-gcc' 不是内部或外部命令
```

原因：

> 工具本体存在，但所在目录没有加入 PATH。

解决：

- 找到 `arm-none-eabi-gcc.exe`；
- 将其所在 `bin` 目录加入 Windows PATH；
- 重新打开终端。

---

## 15.2 QEMU 安装成功但命令不可用

现象：

```text
'qemu-system-arm' 不是内部或外部命令
```

原因同样是 PATH。

解决方式相同。

---

## 15.3 QEMU 看似“卡死”

最初运行：

```powershell
qemu-system-arm -M mps2-an386 -kernel hello.elf -nographic
```

程序似乎没有返回。

需要区分：

```text
没有返回终端
→ 可能正常，因为 main() 最后是 while(1)

没有打印 Hello
→ UART/启动代码仍需要排查
```

随后修正：

- UART 初始化；
- UART 状态等待；
- `startup.s` 中加入 `.thumb_func`；
- 使用更明确的串口参数。

最终：

```powershell
qemu-system-arm -M mps2-an386 -kernel hello.elf -serial stdio -monitor none
```

成功输出：

```text
Hello from Cortex-M4!
```

---

## 15.4 `startup.s` 的换行 Warning

编译曾提示：

```text
Warning: end of file not at end of a line; newline inserted
```

含义：

> 文件最后缺少换行符，汇编器自动补上。

这不是编译错误。

处理方法：

> 在文件最后按一次 Enter 并保存。
