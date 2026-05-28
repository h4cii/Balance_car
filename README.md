# BalancingCar

基于 STM32F103 的 WHEELTEC B570 智能自平衡小车工程。项目使用 CubeMX/HAL 初始化底层外设，业务代码集中在 `Core/CODE`，并使用轻量 C++ 架构实现自平衡控制、蓝牙遥控、超声波模式、OLED 状态显示和电池保护。

## 功能概览

- MPU6050 姿态采样，PA12 外部中断触发 5 ms 控制周期。
- 软件 I2C 读取 MPU6050 原始加速度和陀螺仪数据。
- Kalman 滤波估计俯仰角。
- TIM2/TIM4 编码器测速。
- LQR 控制计算左右轮 PWM。
- TIM1 PWM 驱动电机，GPIO 控制正反转。
- USART3 蓝牙遥控前进、后退、转向、停止和模式切换。
- TIM3 输入捕获超声波测距，支持避障和跟随模式。
- OLED 显示运行状态、角度、电池电压和 PWM。
- ADC 电池电压检测，低压保护。
- 倾角异常、MPU6050 异常自动停机。

## 硬件与外设

| 功能 | 外设/引脚 |
| --- | --- |
| MCU | STM32F103C8Tx |
| IMU | MPU6050 |
| IMU I2C | PB8/PB9 软件 I2C |
| IMU INT | PA12 EXTI |
| 左编码器 | TIM2 |
| 右编码器 | TIM4 |
| 电机 PWM | TIM1 CH1/CH4 |
| 电机方向 | PB14/PB15, PB13/PB12 |
| 蓝牙 | USART3 |
| 超声波输入捕获 | TIM3 CH3 |
| 超声波触发 | PB1 |
| 电池 ADC | ADC1, 1/11 分压 |
| 用户按键 | `User_key` |

具体引脚以 `BalancingCar.ioc` 和 CubeMX 生成的 `main.h` 为准。

## 代码结构

```text
BalancingCar/
  Core/
    CODE/                 用户业务代码
      car_main.cpp        my_main 入口
      car_control.cpp     顶层控制器和 5 ms 控制循环
      car_config.hpp      参数和硬件常量
      car_mpu6050_raw.*   MPU6050 原始数据读取
      car_soft_i2c.*      软件 I2C
      car_kalman.*        Kalman 滤波
      car_encoder.*       编码器
      car_lqr_control.*   LQR 控制
      car_motor.*         电机 PWM 和方向
      car_remote_control.*蓝牙遥控
      car_ultrasonic.*    超声波测距
      car_status_display.*OLED 显示
      car_battery.*       电池电压检测
      car_time.*          微秒延时
    Src/                  CubeMX 生成代码
    Inc/                  CubeMX 生成头文件
  Drivers/                STM32 HAL/CMSIS
  CMakeLists.txt          CMake 构建脚本
```

`Core/CODE` 中的模块采用 `car_模块.cpp/.hpp` 命名，模块对象位于 `namespace car`，例如 `car::controller`、`car::motor`、`car::lqr`。

## 程序流程

1. `main.c` 完成 HAL、GPIO、ADC、TIM、USART 初始化。
2. `main.c` 调用 `my_main()`。
3. `my_main()` 初始化 `car::controller` 并进入前台循环。
4. 前台循环负责按键启停、超声波触发和 OLED 刷新。
5. MPU6050 INT 通过 PA12 触发外部中断。
6. 中断回调进入 5 ms 闭环控制：读取传感器、滤波、更新 LQR 状态、输出 PWM。

## 构建

需要安装 ARM GCC 工具链，并确保 CMake 使用 `arm-none-eabi-gcc` 和 `arm-none-eabi-g++`。

在项目上级目录执行：

```powershell
cmake --build BalancingCar\cmake-build-debug
```

构建成功后生成：

```text
BalancingCar/cmake-build-debug/BalancingCar.elf
BalancingCar/cmake-build-debug/BalancingCar.hex
BalancingCar/cmake-build-debug/BalancingCar.bin
```

当前已验证构建通过，资源占用约为：

```text
RAM   4784 B / 20 KB
FLASH 45840 B / 64 KB
```

## 下载与上电

1. 使用 ST-Link 或开发环境下载 `BalancingCar.hex` 或 `BalancingCar.bin`。
2. 上电后电机默认停止。
3. 确认小车架空时，检查 OLED 或调试数据中的角度是否合理。
4. 按用户按键启用平衡控制。
5. 首次落地测试应限制功率，并扶住车体，避免电机方向或编码器方向错误导致冲出。

## 蓝牙遥控

USART3 用于蓝牙命令。当前支持参考小车常见命令：

| 命令 | 功能 |
| --- | --- |
| `0x41` / `0x01` | 前进 |
| `0x45` / `0x05` | 后退 |
| `0x42/0x43/0x44` 或 `0x02/0x03/0x04` | 一侧转向 |
| `0x46/0x47/0x48` 或 `0x06/0x07/0x08` | 另一侧转向 |
| `0x5A` / `0x00` | 停止 |
| `0x58` | 快速档 |
| `0x59` | 慢速档 |
| ASCII `0` | 普通模式 |
| ASCII `1` | 超声波避障模式 |
| ASCII `2` | 超声波跟随模式 |

## 模式说明

普通模式：蓝牙速度和转向命令直接映射为 LQR 目标速度和目标偏航速度。

超声波避障模式：当前方距离小于 `BALANCE_ULTRASONIC_AVOID_MM` 时自动后退。

超声波跟随模式：距离大于 `BALANCE_ULTRASONIC_FOLLOW_FAR_MM` 时前进，距离小于 `BALANCE_ULTRASONIC_FOLLOW_NEAR_MM` 时后退，中间距离停止。

## 调参与方向修正

主要参数位于：

```text
Core/CODE/car_config.hpp
```

优先检查和调整：

| 参数 | 说明 |
| --- | --- |
| `BALANCE_TARGET_ANGLE_RAD` | 平衡目标角 |
| `BALANCE_RATIO_ACCEL` | 加速度到 PWM 的比例 |
| `BALANCE_LQR_K1` 到 `BALANCE_LQR_K6` | LQR 增益 |
| `BALANCE_LEFT_ENCODER_SIGN` | 左编码器方向 |
| `BALANCE_RIGHT_ENCODER_SIGN` | 右编码器方向 |
| `BALANCE_LEFT_MOTOR_SIGN` | 左电机方向 |
| `BALANCE_RIGHT_MOTOR_SIGN` | 右电机方向 |
| `BALANCE_PWM_MAX` | PWM 限幅 |
| `BALANCE_BATTERY_LOW_CV` | 低压保护阈值，单位 0.01 V |

方向不对时优先修改 `car_config.hpp` 中的 sign 宏，不要直接改 CubeMX 生成代码。

## 安全检查

首次调试建议按顺序检查：

1. MPU6050 `WHO_AM_I` 是否为 `0x68`。
2. PA12 中断周期是否约 5 ms。
3. TIM2/TIM4 编码器读数方向是否符合前进为正的约定。
4. 低 PWM 下左右电机正反转是否正确。
5. 架空状态下角度输出是否平滑。
6. 小功率落地测试，再逐步调大参数。

异常停机原因包括用户停止、倾角过大、低电压、MPU6050 错误。

## 开发约定

- `Core/CODE` 使用 C++17，普通模块不要再添加 C 风格 wrapper。
- 只有 `my_main()` 和 HAL 回调使用 `extern "C"`。
- 不使用动态内存、异常、RTTI、STL 容器。
- CMake 已关闭 C++ exceptions、RTTI 和 thread-safe statics。
- CubeMX 生成代码尽量不改，必要修改放在用户代码区。
- 参数优先集中在 `car_config.hpp`。

