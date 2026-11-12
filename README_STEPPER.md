# 28 步进电机驱动（TB6600 + STM32F103C8T6）

PC(USB-TTL) ⇄ USART3 ⇄ STM32F103C8T6 ⇄ TB6600 ⇄ 28 步进电机
　　　　　　　　　　　　　　　└─ OLED(PA8/PA9)　└─ IN8/IN9 限位(PB0/PB1)

- 脉冲：TIM3_CH1 → **PA6**（PWM2 + 低极性，固定 10 µs 脉冲，频率=转速）
- 方向：**PA12** 推挽输出
- 串口：**USART3 115200-8N1（PB10=TX，PB11=RX）**，RXNE 中断收字节
- 限位：PB0=IN8 近端，PB1=IN9 远端，光耦低有效，EXTI 下降沿**硬急停**
- OLED：SSD1306 128×64，软件 I2C（PA8=SCL，PA9=SDA）

> 控制台从 USART2(PA2/PA3) 改到 USART3 的原因：实测 **PA2 空闲为 0 V**
> （UART 空闲必须是 3.3 V），发线拔掉仍为 0 V，判定该引脚损坏；
> 而 PA3、模块回环、时钟、共地都正常。USART3 的 PB10/PB11 空闲且不冲突。

---

## 1. 相对原工程改了什么

| 位置 | 原状态 | 现在 |
|---|---|---|
| DIR 引脚 | **没有配置** | 新增 PA12 推挽输出（`.ioc` 同步加了 PA12） |
| TIM3 | PWM1、1 kHz 定频、50% 占空 | PWM2 + 低极性，频率在线可调，脉冲固定 10 µs |
| TIM3 中断 | 未开启 | 开启，优先级 2，用于精确数脉冲 |
| `OLED.c` | 标准库（SPL）代码，**编译不过**，且用的是 PB8/PB9 | 改成 HAL 版、PA8/PA9、带 1 KB 显存和脏页刷新 |
| `OLED.c` 参与编译 | 没有 | 已加入 `CMakeLists.txt` |
| PA8/PA9 | 开漏无上拉 | 开漏 + 内部上拉 |
| `OLED_Font.h` | 二维数组，`-Wmissing-braces` 警告 | 改一维平铺，零警告 |
| 构建产物 | 只有 `.elf` | 额外生成 `.bin` / `.hex` |
| 控制台串口 | USART2（PA2/PA3），DMA+IDLE | **USART3（PB10/PB11），RXNE 中断**（PA2 实测损坏） |

> `tim.c` 里的 PWM 模式和极性我也改了，但 `Stepper_Init()` 会再强制配置一次，
> 所以即使 CubeMX 重新生成把 `tim.c` 覆盖回去，驱动仍然正确。

---

## 2. 接线

### TB6600 端子

| TB6600 | 接到 | 说明 |
|---|---|---|
| PUL+ | 3.3V | 与 DIR+、ENA+ 并到板上 3.3V |
| PUL- | PA6 | 脉冲（TIM3_CH1） |
| DIR+ | 3.3V | |
| DIR- | PA12 | 方向 |
| ENA+ | 3.3V | |
| ENA- | 悬空 | 驱动器常使能（保持力矩） |
| VCC | 12~24V 正 | 粗线，不要用杜邦线 |
| GND | 电源负 + STM32 GND | **必须共地** |
| A+ / A- | 电机 A 相 | 用接线端子压接 |
| B+ / B- | 电机 B 相 | 用接线端子压接 |

### DB9（电机侧分线）

| 针 | 功能 | 接到 |
|---|---|---|
| 1 | 24V | 开关电源正（按实际电源电压） |
| 4 | GND | 电源负 + STM32 GND |
| 2 | IN8 近端限位 | PB0 |
| 3 | IN9 远端限位 | PB1 |
| 5 | 悬空 | 6 线电机中心抽头，绝缘 |
| 6/7 | +AX / -AX | TB6600 A+ / A- |
| 8/9 | +BX / -BX | TB6600 B+ / B- |

> 这个 DB9 只是把电机相线、电源、限位分出来，**不是 RS-485 差分信号**，不需要 MAX485。

### 其它

| 器件 | 接线 |
|---|---|
| OLED | VCC→3.3V，GND→GND，SCL→PA8，SDA→PA9 |
| USB-TTL | **TX→PB11**（USART3_RX），**RX→PB10**（USART3_TX），GND→GND |

> USART2 的 PA2/PA3 已不再使用（PA2 损坏）。PA3 上的旧线可以拆掉。

---

## 3. 串口指令（回车结束，大小写不敏感）

| 指令 | 作用 | 例子 |
|---|---|---|
| `A<角度>` | 走到**绝对角度** | `A90`、`A-45.5` |
| `R<角度>` | 走**相对角度** | `R10`、`R-10` |
| `V<转速>` | 设置转速，0.1~120 rpm | `V6` |
| `+` / `-` | 连续点动，`S` 停止 | |
| `G` | 回零（找 PB0 近端限位） | |
| `Z` | 当前位置清零 | |
| `S` | 停止 | |
| `C` | 清除报警 | |
| `K<每圈步数>` | 200~12800，默认 3200 | `K3200` |
| `?` | 状态 | |
| `H` | 帮助 | |

**单键指令**（`? H S G Z C + -`）收到一个字符立刻执行，串口助手不必勾“末尾加回车换行”；
带参数的 `A/R/V/K` 需要回车（串口助手里勾上 **末尾加回车换行** 最省事）。

上电后会打印：

```
== STEP TABLE CTRL ==
H help   ? status
ST=READY POS=+000.0 TGT=+000.0 V=6.0 SPR=3200 LIM=0/0
```

`?` 状态行里：`POS` 当前角度、`TGT` 目标、`V` 转速、`SPR` 每圈步数、
`LIM` 近端/远端限位（1=压着）、`RX` 累计收到字节数、`FLT` 故障位（1 近端 2 远端 4 回零失败）。

---

## 4. OLED 界面

```
READY     6.0rpm     ← 状态 + 转速
POS +012.5deg        ← 当前位置
TGT +090.0deg        ← 目标位置
LIM 0/0   K3200      ← 限位 / 每圈步数
```

串口连通性用 `?` 里的 `RX` 计数判断：发一个 `?` 它应 +3（`?`+CR+LF），
不变说明 STM32 没收到数据。

状态：`READY` / `MOVE` / `HOME` / `FAULT`。
PA5 接的 LED：待机 0.5 s 闪、运动中常亮、报警 0.1 s 快闪。

---

## 5. 首次上电测试顺序（建议照做）

1. **只接 USB-TTL + OLED**，TB6600 先不接 12/24V。上电应看到 OLED 四行和串口欢迎信息。
   发 `?` 有回显 → 串口通路 OK。
   > 注意：这个阶段发 `A/R/+/-` 会让程序照样数脉冲、POS 跟着变，但电机没通电不会动，
   > 属于正常现象；先只测 `?` 和 `H`。
2. 接好 TB6600 电源（共地！），发 `V2`、`A10`，看电机转向。
3. 发 `-`（点动负向），应当朝 **IN8 近端限位**方向转，`S` 停。
   方向相反时：**交换电机 A+ / A-**（或 B+/B-）任意一组即可，不用改代码。
4. 手动压一下近端限位：应立即停，串口打印 `!LIMIT NEAR (PB0)`。
   松开后发 `C` 清报警。
5. 发 `G` 回零：慢速找近端 → 停 120 ms → 反向退 2° → `DONE HOME POS +000.0`。
6. 正常使用 `A` / `R` 定位。

---

## 6. 参数与换算

- 每圈步数 `SPR = 360 / 步距角 × 细分`。1.8° 电机 + 1/16 细分 = **3200**。
  TB6600 拨码改了细分，就用 `K` 改：`K1600`（1/8）、`K6400`（1/32）。
- 转速 `V` 单位是 rpm。内部 `脉冲频率 = V × SPR / 60`。
  例：V6 + 3200 → 320 步/s，走 360° 需要 10 s。
- 回零退让量、回零速度、超时在 `app.c` 顶部：
  `HOME_BACKOFF_DEG10`（默认 2.0°）、`HOME_RPM10`（2.0 rpm）、`HOME_TIMEOUT_MS`（60 s）。
- 最大脉冲频率 40 kHz；角度分辨率 = 360/SPR（3200 时 0.1125°）。

---

## 7. 限制与注意

1. **匀速运行**（按你的选择），没有加减速。高速直接启动可能丢步/堵转，
   建议 `V ≤ 10`；确实需要更快时再加梯形加减速（状态机已经预留了扩展位置）。
2. **PUL+/DIR+ 必须接 3.3V，不要接 5V**。光耦阳极端 5V、阴极端 3.3V 时仍有约 1.7 V，
   LED 可能微弱导通导致多计步。要 5V 电平请加驱动管/光耦模块，不要直接接。
3. 位置**掉电不保存**。上电后先 `G` 回零或 `Z` 清零，`A` 才有意义。
4. 限位必须是“光耦输出对地开集”，PB0/PB1 是 3.3 V 输入，**不能灌 24 V**。
5. 驱动器和 STM32 必须共地，否则脉冲/限位都不稳。
6. IN8 被定义为**负方向（近端）**。如果实际装反了，`G` 会往远端跑然后报 `!HOME FAIL`。
7. 看门狗默认关闭（`main.c` 里 `APP_IWDG_ENABLE 0`），调试打断点不会复位；
   长期运行可改成 1（约 2 s 超时）。
8. **不要让 CubeMX 勾选 TIM3 中断**：`TIM3_IRQHandler` 写在 `stepper.c` 里，
   勾选后会生成重复定义。PA12 已写进 `.ioc`，重新生成不会丢。

---

## 8. 资源占用（Debug 构建）

| 项目 | 占用 |
|---|---|
| Flash | 26 252 B / 64 KB（40.1%） |
| RAM | 3 432 B / 20 KB（16.8%） |
| 最大栈帧 | 240 B（HAL UART 中断），1 KB 栈有充足余量 |

编译（零警告）：

```powershell
cmake --preset Debug
cmake --build build/Debug
# 产物：build/Debug/28generator_table.elf / .bin / .hex
```

### CLion 里构建

CLion 会给工程自动建一个 **普通 “Debug” 配置**（用自带 MinGW），直接 Build 会报：

```
unknown pseudo-op: `.syntax`
no such instruction: `cpsid i`
```

这是**工具链选错**，不是代码问题。三种解法任选：

1. 右上角配置下拉里选 **“Debug - Debug”**（来自 `CMakePresets.json`，生成目录 `build/Debug`）；
2. 或 `File > Reload CMake Project`：工程已在 `CMakeLists.txt` 里默认强制使用
   `cmake/gcc-arm-none-eabi.cmake`，重载后即使选普通 “Debug” 也会自动切到 ARM 编译器；
3. 仍报 `-loleaut32 / -luuid` 之类，是旧缓存残留的主机标准库：
   `Settings > Build,Execution,Deployment > CMake > Reset Cache and Reload Project`。

> 注意：`cmake-build-debug/` 是 CLion 的临时构建目录，和 `build/Debug` 互不影响。

---

## 9. 文件

| 文件 | 作用 |
|---|---|
| `Core/Inc/stepper.h`、`Core/Src/stepper.c` | 脉冲/方向/计数/限位/角度换算 |
| `Core/Inc/cli.h`、`Core/Src/cli.c` | USART2 收帧、指令解析、格式化 |
| `Core/Inc/app.h`、`Core/Src/app.c` | 定位、回零状态机、报警、OLED 界面 |
| `Core/Src/OLED.c`、`Core/Inc/OLED.h` | SSD1306 HAL 版软件 I2C |
| `Core/Src/main.c` | 初始化顺序 + 主循环 |
| `Core/Src/gpio.c`、`tim.c`、`stm32f1xx_it.c` | CubeMX 生成文件的小改动 |

> 说明：以上为编译、静态检查和整数运算仿真的结果，**尚未上实机验证**。
> 首次上电请按第 5 节顺序，先小角度低速试。
