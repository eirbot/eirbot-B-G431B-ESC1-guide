# eirbot-B-G431B-ESC1-guide
An overview on how to use and control ST's B-G431B-ESC1 to control brushless motors such as the EC 45 flat (Maxon)

I am not by any means an expert on brushless motoors nor an electrical engineer. This is just a guide to help you get started with the B-G431B-ESC1 board. Most of this guide applies to any brushless motor but some parts are specific to the EC 45 flat motor.

This repository contains the following files:
- `EC 45 flat brushless motor documentation`: Documentation provided by Maxon for the EC 45 70W flat motor used by Eirbot
- `B-G431B-ESC1 User Manual`: Documentation provided by ST for the B-G431B-ESC1 board
- `An already tuned project for the B-G431B-ESC1 board in /project`


# Table of contents
1. [What is the B-G431B-ESC1 board?](#what-is-the-b-g431b-esc1-board)
2. [Software needed](#software-needed)
3. [Hardware needed](#hardware-needed)
4. [Connecting the motor to the board](#connecting-the-motor-to-the-board)
5. [Basic Workflow](#basic-workflow)
6. [Step by step guide](#step-by-step-guide)

## What is the B-G431B-ESC1 board?
The B-G431B-ESC1 is an electronic speed controller (ESC) board designed by ST to control brushless motors. It's main advantage is that it has embedded FOC (Field Oriented Control) capabilities while being pretty inexpensive.

While Eirbot used to control the EC 45 flat brushless motors with custom made ESCs, we decided to switch to the B-G431B-ESC1 board for its ease of use and reliability. Unfortunately, the documentation provided by ST is not very clear and it took a while to figure out how to use it. This guide is here to help you get started with the board and avoid the mistakes we made.

## Software needed
**A lot of time can be lost by figuring out wich version of the software is compatible with the board**. Here is a list of the software versions used to get the board working:

- [STM32CubeIDE v1.13.2](https://www.st.com/en/development-tools/stm32cubeide.html)
- [MotorControlWorkbench (MCSDK) v6.2.0](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)
- [STM32CubeMX v6.4.0](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32 FW v1.5.0](Firmware package version choosed at project generation in MCSDK, version might not be critical)
- [Motor Profiler v5.4.8](Old version (easier to use) obtainable by download an old version of MCSDK)

CubeIDE is the IDE used to write the code and flash it on the board. MCSDK is the an high level software that allows you to test, control and generate code for the board. CubeMX is a software that allows you to configure the board and generate code for it. Motor Profiler is a software that allows you to profile the motor to automatically generate some of the parameters for MCSDK.

## Hardware needed
- [B-G431B-ESC1 board](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html)
- [EC 45 flat brushless motor](https://www.maxongroup.com/maxon/view/product/motor/ecflat/405536)
- [Nucleo board such as the F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)
- [24V power supply]

## Connecting the motor to the board
A brushless motor works by having it's windings (coils) driven by 3 pwm signals. In order to control the motor, you need to connect the 3 pwm signals to the board. The board has 3 pins for the pwm signals: `U`, `V` and `W`. You should link each of these pins to the corresponding pin on the motor (In EC 45 falt documentation in this repository you can find the pinnout in 'Connection' at column V1). 

The powering of the B-G431B-ESC1 board is straightforward, you just need to connect the 24V power supply to the `VDC+` and `GND` pins.

Other gpio pins are needed to control the motor but you should only need to connect the PWM (PA15) and ground pins to another board (such as a Nucleo board) to control the motor.

## Basic Workflow
The basic workflow to get the board working is the following:

1. Profile the motor with Motor Profiler and save them, you should see your motor spin by the end of this step
2. Open MCSDK and create a new project with Algorithm = FOC, Hardware = inverter 
    2.1. Select the motor profiled in Motor Profiler in the Motor tab
    2.2. Select the B-G431B-ESC1 board in Inverter tab
3. Tune the parameters in MCSDK and **make sure the Electronic Speed Control parameter is on in the Stage Configuration tab (If not create a new project in MCSDK by modifying the exemple project for the B-G431B-ESC1 board)**
4. Generate the project in MCSDK, choose the right version of the firmware package, and CubeMX while targeting CubeIDE as the toolchain
5. Click the "RUN STM32CubeMX" button in MCSDK to open CubeMX after the project generation (Don't worry about the error 'The ST intranet updater server is unknown')
6. In CubeMX, click on the "Generate Code" button
7. Open the project in CubeIDE
8. Build the project in CubeIDE
9. Flash the project in CubeIDE

## Step by step guide
The following steps detail how to get the board working in order to have a **speed controlled** motor that can be latter on used for a robot.

### 1. Profile the motor with Motor Profiler

### 2. Finding the right Kp and Ki values

#### 2.1 PI tuning for dummies
#### 2.2. Troubleshooting with an integral term in CubeIDE
The Ki parameter is not implemented correctly in the firmware package provided by ST. At board reset, the Ki parameter will be set to 0. In order to fix it, you need to modify the code in the file `YOUR-PROJECT-NAME/Application/User/mc_tasks.c` at line 540. You need to replace the following code:
```c
                    PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
```
by this code:
```c
                    PID_SetIntegralTerm(&PIDSpeedHandle_M1, X);
```
With X an integer being the value of Ki. For me Ki=2 was a good value (Your Ki value can be found with the methodology listed above and will be stored in the macro PID_SPEED_KI_DEFAULT inside `drive_parameters.h`). **X isn't Ki / KiDivisor**
        
### 3. Spinning in the right direction


