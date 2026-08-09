# CMake 构建说明 / CMake Build Guide

## 状态 / Status

- 中文：目标 `standard_robot`（STM32F407IGHx），状态为 **已验证 / validated**。干净配置、完整编译、链接、产物生成（.elf/.bin/.hex/.map）均成功；AHRS.lib 与 arm_cortexM4lf_math.lib（ARMCC 预编译库）已与 GCC 链接并通过符号检查。
- 中文：工程已与 Keil 工程拆分——本目录为 **CMake/GCC 工程**；原 Keil 工程位于同级目录 `StandardRobotpp-keil`（源码为原始版本，本目录不含 MDK-ARM 工程文件）。
- English: Target `standard_robot` (STM32F407IGHx); status: **validated**. Clean configure, full build, link, and artifact generation (.elf/.bin/.hex/.map) succeeded; the prebuilt ARMCC libraries (AHRS.lib, arm_cortexM4lf_math.lib) link correctly with GCC and key symbols were verified.
- English: This directory is the **CMake/GCC project**, split from the Keil project; the original Keil project now lives in the sibling folder `StandardRobotpp-keil` (with pristine sources; this folder contains no MDK-ARM project files).

## 环境检查 / Environment Check

中文：在项目根目录执行以下命令，确认必需工具可用：

English: Run this command from the project root to confirm required tools are available:

```text
python check_build_env.py
```

要求：CMake、Ninja、Arm GNU 工具链（arm-none-eabi-gcc/objcopy/size/nm/readelf）、Python。若 CMake 不在 PATH（如使用 CLion 自带版本），请把其 bin 目录加入 PATH 或使用绝对路径。

Prerequisites: CMake, Ninja, Arm GNU toolchain (arm-none-eabi-gcc/objcopy/size/nm/readelf), Python. If CMake is not on PATH (e.g. the CLion-bundled one), add its bin directory to PATH or use the absolute path.

## 编译 / Build

中文：在项目根目录依次执行：

English: Run these commands from the project root:

```text
cmake -S . -B build -G Ninja "-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake"
cmake --build build
```

也可以直接运行 `build.bat`（Windows，自动先做环境检查）。

Or simply run `build.bat` (Windows; runs the environment check first).

## 编译成品 / Build Outputs

| 文件 / File | 简介 / Description |
| --- | --- |
| `build/standard_robot.elf` | 可调试的固件及符号 / Debuggable firmware and symbols |
| `build/standard_robot.bin` | 原始烧录镜像 / Raw programming image |
| `build/standard_robot.hex` | Intel HEX 烧录镜像 / Intel HEX programming image |
| `build/standard_robot.map` | 链接映射与内存分布 / Link map and memory layout |

尺寸（keil 对比）：Flash 载入约 103 KB（Keil 约 130 KB，-O1 + gc-sections 更小）；RAM 静态占用约 46 KB + 8 KB 堆 + 8 KB 栈（与 Keil 启动文件 Stack/Heap 0x2000 一致）。向量表位于 0x08000000，`_estack` = 0x20020000。

Size (vs Keil): Flash load ~103 KB (Keil ~130 KB; -O1 + gc-sections is smaller); static RAM ~46 KB + 8 KB heap + 8 KB stack (matching the Keil startup Stack/Heap 0x2000). Vector table at 0x08000000, `_estack` = 0x20020000.

## 备注 / Notes

- 中文：
  - Keil 工程已拆分为独立目录：同级 `StandardRobotpp-keil`（含 MDK-ARM/standard_robot.uvprojx 与全部原始源码，GCC 迁移文件不在其中）。后续如需对 CMake 工程重新做 parity 校验，可将该校验器指向 `../StandardRobotpp-keil/MDK-ARM/standard_robot.uvprojx`。
  - 本目录不含 MDK-ARM；构建所需的 GCC 启动文件与 FreeRTOS GCC 移植层为本目录新增（`Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/`、`Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/`）。
  - 保留目标：Keil 工程 `../StandardRobotpp-keil/MDK-ARM/standard_robot.uvprojx` 为原始版本，未做任何改动。
  - 编译器专属替换（GNU 替代 Keil/ARMCC）：启动文件 `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s`（替换 Keil 工程中 `MDK-ARM/startup_stm32f407xx.s`）；FreeRTOS 移植层 `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/`（替换 `portable/RVDS/ARM_CM4F/`，FreeRTOS V10.0.1 官方 GCC 移植）。
  - ARMCC 专属宏 `__CC_ARM` 未定义；`kalman_filter.h` 中强制定义 `__CC_ARM` 的代码已加 GCC 守卫（GCC 下定义会令 CMSIS 选择 cmsis_armcc.h 导致编译失败）。
  - 预编译库 AHRS.lib / arm_cortexM4lf_math.lib 为 ARMCC 产物，ELF 归档格式，VFPv4-D16 **hard-float** ABI；整套构建使用 `-mfloat-abi=hard -mfpu=fpv4-sp-d16` 保持一致。链接时使用 `--no-warn-mismatch` 放宽部分对象的 CPU 架构元数据差异（代码实测为 Thumb-2，可在 Cortex-M4 运行）。
  - 可移植性小修复（均不影响 Keil 行为）：`struct_typedef.h`/`AHRS_middleware.h` 的 stdint 重定义加 `__CC_ARM` 守卫；`__packed`/`__packed__` 宏冲突（newlib `sys/cdefs.h` 的 `__packed` 引用 `__packed__` 令牌）通过删除 `__packed__` 宏、改用 `__packed` 并在 GCC 下定义为 `__attribute__((packed))` 解决；`typedef __packed struct` 改为 `typedef struct __packed`（GCC 会忽略前者导致结构体不压缩）；`communication.c` 宏中 13 处尾随 `##` 修正；`bsp_flash.c` 的 `static` 前向声明与头文件一致化。
  - 运行时：newlib-nano + nosys（Keil 无 MicroLIB、无 fputc/_write；仅 fifo.c 断言使用整数 printf，与 Keil 一致，无宿主输出）。
  - 本项目路径含中文（桌面），GNU ld 无法解析非 ASCII 链接路径；链接脚本与预编译库在配置期拷贝进 build 目录并以相对名参与链接（`-Tstandard_robot.ld`、`-l:AHRS.lib` 等），编译侧不受影响。
  - 已知差异：GCC 下 `packed` 布局与 Keil 一致（已用 _Static_assert 验证）；编译器差异产生的警告（inline 未定义、unused 等）为应用既有代码在 GCC 下的显示，不影响行为。
- English:
  - The Keil project was split into a separate folder: sibling `StandardRobotpp-keil` (contains `MDK-ARM/standard_robot.uvprojx` and all pristine sources; no GCC migration files). For future parity checks, point the validator at `../StandardRobotpp-keil/MDK-ARM/standard_robot.uvprojx`.
  - This folder contains no MDK-ARM; the GCC startup and FreeRTOS GCC port are additive files here (`Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/`, `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/`).
  - The Keil project `../StandardRobotpp-keil/MDK-ARM/standard_robot.uvprojx` is the pristine original, untouched.
  - Compiler-specific replacements (GNU for Keil/ARMCC): startup `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s` (replaces the Keil `MDK-ARM/startup_stm32f407xx.s`); FreeRTOS port `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/` (replaces `portable/RVDS/ARM_CM4F/`; official V10.0.1 GCC port).
  - The ARMCC-only macro `__CC_ARM` is not defined; the forced `__CC_ARM` define in `kalman_filter.h` is now GCC-guarded (defining it under GCC makes CMSIS include cmsis_armcc.h and breaks the build).
  - Prebuilt AHRS.lib / arm_cortexM4lf_math.lib are ARMCC ELF archives with VFPv4-D16 **hard-float** ABI; the whole build uses `-mfloat-abi=hard -mfpu=fpv4-sp-d16` for consistency. `--no-warn-mismatch` relaxes CPU-architecture metadata differences in some objects (disassembly confirms Thumb-2 code, runnable on Cortex-M4).
  - Small portability fixes (Keil behavior unchanged): stdint redefinitions in `struct_typedef.h`/`AHRS_middleware.h` guarded by `__CC_ARM`; the `__packed__` macro conflict (newlib `sys/cdefs.h` `__packed` references the `__packed__` token) resolved by removing the `__packed__` macro, using `__packed` (defined as `__attribute__((packed))` under GCC); `typedef __packed struct` changed to `typedef struct __packed` (GCC ignores the former, leaving structs unpacked); 13 trailing `##` pastes fixed in `communication.c` macros; `bsp_flash.c` static forward declaration aligned with its header.
  - Runtime: newlib-nano + nosys (Keil: no MicroLIB, no fputc/_write; only the fifo.c assert uses integer printf — same as Keil, no host output).
  - This project path contains non-ASCII characters (桌面); GNU ld cannot open non-ASCII link paths, so the linker script and prebuilt libraries are copied into the build directory at configure time and linked by relative names (`-Tstandard_robot.ld`, `-l:AHRS.lib`, ...). Compile side is unaffected.
  - Known differences: packed layout verified identical to Keil via _Static_assert; remaining warnings (unused inline, etc.) are pre-existing application code shown under GCC and do not affect behavior.
