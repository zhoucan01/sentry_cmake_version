# infantry_25 — STM32F407 机器人控制固件

> 本项目由 **Keil MDK** 编译链迁移至 **CMake + Ninja** 构建系统，支持 GCC / Clang 双工具链。

---

## 项目概述

基于 STM32F407VGTx（Cortex-M4 + FPU）的 RoboMaster 竞赛机器人综合控制固件，包含：

| 模块               | 功能                                                                          |
| ------------------ | ----------------------------------------------------------------------------- |
| **姿态估计** | 四元数扩展卡尔曼滤波 (EKF)，融合陀螺仪+加速度计，带陀螺零偏在线估计与卡方检验 |
| **底盘控制** | 麦克纳姆轮 / 全向轮运动学解算                                                 |
| **云台控制** | 大/小云台随动控制                                                             |
| **导航决策** | 场上行为决策                                                                  |
| **裁判系统** | RoboMaster 裁判系统通信协议                                                   |
| **视觉通信** | Nautilus Vision 目标检测数据交互                                              |
| **CAN 通信** | 电机控制、传感器数据收发                                                      |

**硬件**

- MCU：STM32F407VGTx（168MHz Cortex-M4, FPU）
- IMU：BMI088（SPI）
- 系统：FreeRTOS + HAL

---

## Keil → CMake 迁移说明

本项目最初在 Keil MDK 下开发，已完成为以下迁移：

| 迁移项   | Keil 原方式           | CMake 方式                                             |
| -------- | --------------------- | ------------------------------------------------------ |
| 构建系统 | `.uvprojx` 工程文件 | `CMakeLists.txt` + `CMakePresets.json`             |
| 工具链   | ARMCC (Keil)          | `arm-none-eabi-gcc`（默认）/ `starm-clang`（可选） |
| 启动文件 | Keil 内置             | `startup_stm32f407xx.s`（GCC 汇编语法）              |
| 链接脚本 | Keil scatter file     | `STM32F407XX_FLASH.ld`（GNU ld 语法）                |
| HAL 配置 | Keil RTE              | CubeMX 生成的`cmake/stm32cubemx/`                    |
| DSP 库   | Keil Pack             | CMSIS-DSP 源码直接编译                                 |
| 烧录     | Keil 内置             | ST-LINK_CLI + CMake`flash` 目标                      |

**编译警告处理**：Keil 编译器 (ARMCC) 对 `__packed`、部分初始化等语法比 GCC 宽松，`cmake/gcc-arm-none-eabi.cmake` 中已针对性屏蔽以下无害警告：

- `-Wno-attributes` — `__packed` 属性语法差异
- `-Wno-missing-field-initializers` — HAL 结构体部分初始化
- `-Wno-unused-parameter` — FreeRTOS 任务函数未用参数
- `-Wno-missing-braces` — 二维数组初始化大括号风格

---

## 快速开始

### 前置依赖

| 工具                            | 用途         |
| ------------------------------- | ------------ |
| `arm-none-eabi-gcc` (>= 10.3) | 交叉编译器   |
| `ninja`                       | 构建后端     |
| `cmake` (>= 3.22)             | 构建系统生成 |
| `STM32 ST-LINK Utility`       | 烧录（可选） |

确保 `arm-none-eabi-gcc`、`ninja`、`cmake` 均在 `PATH` 中。

### 构建

```bash
# 配置 (Debug)
cmake --preset Debug

# 构建
cmake --build build/Debug

# 或一步到位
cmake --preset Debug && cmake --build build/Debug
```

输出文件：`build/Debug/infantry_25.elf`、`infantry_25.hex`

### Release 构建

```bash
cmake --preset Release
cmake --build build/Release
```

### 烧录 (ST-Link)

```bash
cmake --build build/Debug --target flash
```

或使用 VS Code 任务 "Flash STM32 (ST-Link)"。

---

## 目录结构

```
.
├── CMakeLists.txt              # 顶层构建脚本
├── CMakePresets.json           # CMake 预设 (Debug / Release)
├── STM32F407XX_FLASH.ld        # 链接脚本 (GNU ld)
├── startup_stm32f407xx.s       # 启动文件 (GCC 汇编)
│
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # GCC 工具链配置
│   ├── starm-clang.cmake       # Clang 工具链配置 (可选)
│   └── stm32cubemx/            # CubeMX 生成的 HAL/FreeRTOS 构建
│
├── application/                # ── 应用层 ──
│   ├── CAN_receive.c/h         #   CAN 接收
│   ├── CAN_transmit.c/h        #   CAN 发送
│   ├── ins_task.c/h            #   IMU 惯导任务
│   └── remote_control.c/h      #   遥控器解析
│
├── branch/                     # ── 功能层 ──
│   ├── chassis_control.c/h     #   底盘控制
│   ├── decision.c/h            #   决策
│   ├── gimbal_big_yaw.c/h      #   大云台
│   ├── small_gimbal.c/h        #   小云台
│   ├── motor.c/h               #   电机驱动
│   ├── navigation.c/h          #   导航
│   └── system.c/h              #   系统管理
│
├── bsp/boards/                 # ── 板级支持包 ──
│   ├── bsp_can.c/h             #   CAN 驱动
│   ├── bsp_pid.c/h             #   PID 控制器
│   ├── bsp_PWM.c/h             #   PWM 输出
│   ├── bsp_dwt.c/h             #   DWT 延时
│   ├── kalman.c/h              #   卡尔曼滤波器基础库
│   └── ...
│
├── Components/                 # ── 组件 ──
│   ├── kalman_filter.c/h       #   通用卡尔曼滤波器
│   ├── Algorithm/
│   │   └── QuaternionEKF.c/h   #   ★ 四元数 EKF 姿态估计
│   ├── Controller/
│   │   └── controller.c/h      #   串级控制器
│   └── Devices/
│       ├── BMI088driver.c/h    #   BMI088 IMU 驱动
│       └── ...
│
├── referee/                    # 裁判系统通信
├── Nautilus_UI/                # 操作界面
├── Nautilus_Vision/            # 视觉通信
├── essemi/                     # SWD 调试打印
│
├── Core/                       # HAL 库生成的初始化代码
├── Drivers/                    # CMSIS + HAL 驱动
└── Middlewares/                 # FreeRTOS 等中间件
```

---

## 任务间通信框架 — `message_center`

### 背景

项目使用 **10 个 FreeRTOS 抢占式任务**并发运行（`configUSE_PREEMPTION=1`），改造前所有任务间通信依赖 **裸全局变量**（15+ 个跨模块 `extern` 结构体），无任何同步保护。

改造后引入 `modules/message_center/` 伪发布/订阅机制（源自岳麓战队 `basic_framework`），并添加 **FreeRTOS 互斥锁** 适配抢占式多任务环境。

### 架构

```
ISR (USART/CAN)
  │ 写入原始缓冲区
  ▼
桥接任务 (system_task / REFEREE_Task / big_yaw_run)
  │ PubPushMessage()
  ▼
┌─────────────────────────────────────────┐
│           message_center                │
│  ┌─────────┐  ┌──────────┐             │
│  │ Topic A │──│ Sub 队列1 │ → Task 2   │
│  │ 链表头  │──│ Sub 队列2 │ → Task 3   │
│  │         │──│ Sub 队列3 │ → Task 4   │
│  └─────────┘  └──────────┘             │
│  ┌─────────┐  ┌──────────┐             │
│  │ Topic B │──│ Sub 队列1 │ → Task 5   │
│  └─────────┘  └──────────┘             │
│        ▲ 互斥锁保护                      │
└─────────────────────────────────────────┘
```

### Topic 清单（14 个话题）

| # | 话题名 | 数据类型 | 发布者 | 订阅者 |
|---|--------|---------|--------|--------|
| 1 | `sentry_system` | `sentry_system_t` | system_task | chassis, big_yaw, small_gimbal, transmit |
| 2 | `ins_data` | `INS_t` | INSTask | system, big_yaw, transmit, decision |
| 3 | `rc_ctrl` | `RC_ctrl_t` | system_task (桥接) | big_yaw, small_gimbal, decision, chassis |
| 4 | `gimbal_state` | `gimbal_t` | big_yaw | motor, transmit |
| 5 | `yaw_motor_data` | `DM_motor_measure_t` | big_yaw | motor, transmit |
| 6 | `decision_data` | `decision_t` | Auto_run | system, big_yaw, transmit, chassis, UI |
| 7 | `vision_data` | `receive_gimbal_data_t` | big_yaw (桥接) | decision |
| 8 | `chassis_state` | `sentry_chassis_t` | chassis | transmit |
| 9 | `small_gimbal_state` | `small_gimbal_t` | small_gimbal | transmit |
| 10 | `game_state` | `game_state_t` | REFEREE | 7 个任务 |
| 11 | `robot_status` | `robot_status_t` | REFEREE | 6 个任务 |
| 12 | `power_heat` | `power_heat_data_t` | REFEREE | transmit, decision |
| 13 | `nav_rx` | `navigation_rx_t` | system_task (桥接) | chassis, decision, big_yaw |
| 14 | `nav_tx` | `navigation_tx_t` | decision | navigation 模块 |

### 使用示例

**发布者**（以 IMU 任务为例）：

```c
#include "message_center.h"
#include "big_yaw_topics.h"

static Publisher_t *ins_pub = NULL;

void StartINSTask(void) {
    ins_pub = PubRegister(TOPIC_INS_DATA, sizeof(INS_t));  // 注册

    for (;;) {
        INS_Task();
        PubPushMessage(ins_pub, (void *)&INS);             // 发布
        osDelay(1);
    }
}
```

**订阅者**（以底盘任务为例）：

```c
static Subscriber_t *sentry_sub;

void chassis_task_run(void) {
    sentry_sub = SubRegister(TOPIC_SENTRY_SYSTEM, sizeof(sentry_system_t));

    sentry_system_t sys_local;              // 本地副本
    for (;;) {
        SubGetMessage(sentry_sub, &sys_local);  // 拉取最新数据
        // 后续使用 sys_local 替代全局变量...
        vTaskDelay(1);
    }
}
```

### ISR 桥接策略

ISR 中不能调用 `PubPushMessage`（涉及互斥锁和堆内存）。采取的策略：

- **USART3 ISR**（遥控器）→ 写入 `rc_ctrl` 原始缓冲 → `system_task` 开头发布
- **CAN1 ISR**（视觉/电机）→ 写入原始结构体 → `big_yaw_run` 发布
- **USART6 ISR**（裁判系统）→ 写入 `referee_fifo` → `REFEREE_Task` 解包后发布
- **USB CDC**（导航）→ 保持 `navigation_rx_handle` → `system_task` 发布

### 互斥锁保护

原 `message_center` 设计假设所有 pub/sub 在**同一任务串行执行**，不需要锁。

本项目所有任务以 **同优先级抢占** 运行（`osPriorityNormal`），同一订阅者队列存在"发布者写 / 订阅者读"的竞争（`temp_size` 和 `queue[]` 的读-改-写非原子）。因此 `message_center.c` 中添加了全局互斥锁，4 个 API 内部均以 `xSemaphoreTake/Give` 保护。

### 关键文件

| 文件 | 说明 |
|------|------|
| `modules/message_center/message_center.h` | API 声明 + 数据结构定义 |
| `modules/message_center/message_center.c` | 链表管理 + 互斥锁 + `pvPortMalloc` |
| `application/big_yaw_topics.h` | 14 个话题名宏定义 + 通信关系文档 |

---

## 姿态估计算法

核心文件 `Components/Algorithm/QuaternionEKF.c`，实现 **六状态扩展卡尔曼滤波**：

$$
\mathbf{x} = [q_0,\ q_1,\ q_2,\ q_3,\ b_x,\ b_y]^\top
$$

- **预测**：陀螺仪角速度积分（去偏后）驱动四元数运动学模型
- **观测**：加速度计归一化重力方向向量
- **特色**：
  - 陀螺零偏在线估计（x, y 轴）
  - 卡方检验 (Chi-Square Test) 异常检测
  - 自适应增益缩放 (Adaptive Gain)
  - 渐消滤波 (Fading Filter) 防过度收敛
  - CMSIS-DSP 矩阵运算加速

---

## 常见问题

### Q: 编译报 `undefined reference to ...`？

先确认 `CMakeLists.txt` 中是否已添加对应的 `.c` 源文件，路径相对于项目根目录。

### Q: 能用 Keil 打开吗？

不能直接使用 `.uvprojx`。本项目已完全迁移至 CMake，Keil 工程不再维护。

### Q: 如何切换 Clang 工具链？

修改 `CMakePresets.json` 中 `toolchainFile` 为 `cmake/starm-clang.cmake`，需安装 STM32 Clang 工具链。

### Q: 烧录失败？

检查 `CMakeLists.txt` 末尾 `flash` 目标中 `ST-LINK_CLI.exe` 路径是否正确。

---

## License

本仓库代码仅供学习参考。
