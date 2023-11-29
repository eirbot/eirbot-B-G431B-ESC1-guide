# Eirbot-B-G431B-ESC1-guide

# **This guide is a work in progress**

An overview on how to use and control ST's B-G431B-ESC1 to control brushless motors such as the EC 45 flat (Maxon)

I am not by any means an expert on brushless motors nor an electrical engineer. This is just a guide to help you get started with the B-G431B-ESC1 board. Most of this guide applies to any brushless motor but some parts are specific to the EC 45 flat motor.

This repository contains the following files:
- `EC 45 Flat Brushless Motor Documentation`: Documentation provided by Maxon for the EC 45 70W flat motor used by Eirbot
- `B-G431B-ESC1 User Manual`: Documentation provided by ST for the B-G431B-ESC1 board
- `An already tuned project for the B-G431B-ESC1 board in /project`


# Table of contents
1. [What is the B-G431B-ESC1 board?](#what-is-the-b-g431b-esc1-board)
2. [Software needed](#software-needed)
3. [Hardware needed](#hardware-needed)
4. [Connecting the motor to the board](#connecting-the-motor-to-the-board)
5. [Basic Workflow](#basic-workflow)
6. [Speed command using PWM signal](#speed-command-using-pwm-signal)
7. [Step by step guide](#step-by-step-guide)

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
    - 2.1 Select the motor profiled in Motor Profiler in the Motor tab
    - 2.2 Select the B-G431B-ESC1 board in Inverter tab
3. Tune the parameters in MCSDK and **make sure the Electronic Speed Control parameter is on in the Stage Configuration tab (If not create a new project in MCSDK by modifying the exemple project for the B-G431B-ESC1 board)**
4. Generate the project in MCSDK, choose the right version of the firmware package, and CubeMX while targeting CubeIDE as the toolchain
5. Click the "RUN STM32CubeMX" button in MCSDK to open CubeMX after the project generation (Don't worry about the error 'The ST intranet updater server is unknown')
6. In CubeMX, click on the "Generate Code" button
7. Open the project in CubeIDE
8. Build the project in CubeIDE
9. Flash the project in CubeIDE

## Speed command using PWM signal
The motor can be controlled by sending a PWM signal to the board. The duty cycle of the PWM signal will determine the speed of the motor. The PWM signal can be generated by any board that can generate a PWM signal such as a Nucleo board. The PWM signal should be sent to the `PWM` pin of the B-G431B-ESC1 board. You need to connect the ground of the board to the ground of the Nucleo board as well.

The PWM generated should have a frequency of 490Hz. Easiest way to do this is to use Mbed OS and the PwmOut class. The following code will generate a PWM signal with a frequency of 490Hz and a basic speed command:
```cpp
    #include "mbed.h"
    #include <cstdio>

    PwmOut mot(PB_3);

    int main() {
        mot.period_us(2041); // 1/490Hz ~= 2041us
        
        mot.pulsewidth_us(1060); // arm the ESC, MANDATORY STEP
        ThisThread::sleep_for(1s);

        mot.pulsewidth_us(1200);
        ThisThread::sleep_for(1s);
        
        mot.pulsewidth_us(1400); 
        ThisThread::sleep_for(10s);

        mot.pulsewidth_us(1200);
        ThisThread::sleep_for(3s);

        mot.pulsewidth_us(800); // disarm the ESC
    }
```
What's note precised in the user manual of the B-G431B-ESC1 board is that you need to arm the ESC. Once this is done the speed command is given out by the duty cycle of the PWM signal (between 1060 and 1860 us as detailled at page 21 of the B-G431B-ESC1 User Manual). The speed command is by default given by the following formula:
```
    speed = min_speed + duty_cycle * (max_speed - min_speed) / (1860 - 1060)
```

## Step by step guide
The following steps detail how to get the board working in order to have a **speed controlled** motor that can be latter on used for a robot.

### 1. Profile the motor with Motor Profiler
I recommend you to use the old version of Motor Profiler (v5.4.8) so that you don't have to upload a custom firmware adapted to the Motor Profiler in MCSDK (In the latter case, select the Motor Profiler option inside the Application Configuration tab of MCSDK). In order to profile the motor, you need to connect the motor to the board and the board to a computer. You also need to power the board with a 24V power supply.

#### 1.1 Motor Profiler configuration
Inside Motor Profiler, you need to configure the following parameters:
- **Select Board**: B-G431B-ESC1
- **Pole pairs**: 8 for the EC 45 flat motor (Motor specific)
- **Max speed**: 6000 rpm (Motor specific)
- **Max current**: 6 A (Motor specific)
- **VBus**: 24V (Motor specific)
- **Magnetic**: SM-PMSM (Motor specific), most of the time you will have to select this option

#### 1.2 Motor Profiler procedure
Once all parameters are set, you can start the motor profiling procedure. The procedure is the following:
 - Click connect button
 - Click start button, the motor will start spinning to evaluate mechanical and electrical parameters
 - Save the parameters by clicking the save button once the profile is done (Give it a clear name)

### 2. Getting the right parameters
MCSDK comes with the capability to auto calculate the Kp and Ki parameters for the different PI controllers. However, the auto tuning for the speed PI controller is not very reliable and you will most likely have to tune it yourself. In order to do so, you'll have to first generate a project in MCSDK and flash it on the board so that you'll be able to use the Motor Pilot tool in MCSDK. The subsection 2.1 will detail the parameters that you'll probably only have to tune once. The subsection 2.2 will detail how to tune the PI controllers.

#### 2.1 Fine tuning MCSDK parameters

##### 2.1.1 PWM frequency
Generally, you'll want to have a PWM frequency as high as possible for your ESC in order to have a smooth control of the motor. Also, having a too low PWM frequency can create a lot of undesired noise. The choice has been made here to use 40kHz but anything above 30kHz should do the trick.

##### 2.1.2 Speed sensing
Sensorless PLL detailled soon

#### 2.2 PI tuning of the speed regulator
While a comprehensive understanding of Control Theory is not needed, it is important to have some [basic knowledge](https://en.wikipedia.org/wiki/Nonlinear_control) about it. I am not by any means an expert on Control Theory and will only give you a basic understanding of how to tune the PI controllers.

##### 2.2.1 PI tuning for dummies
![Speed PI](./img/Speed%20regulator.png)

In order to have a fast and accurate speed control, you need to tune the PI controller of the speed regulator. The P and I values are calculated by the following formulas:
```
    P = Kp / KpDivisor
    I = Ki / KiDivisor
```
Where Kp and Ki are integers and KpDivisor and KiDivisor are power of 2 integers. If you already have some Control Theory knowledge, you might be used to tune P and I through a single float value but the methodology here is still the same. The execution rate of the PI is 1ms.

The easiest way to determine the right values for Kp and Ki is to use the Motor Pilot tool through experimentation by plotting the speed sensed by the board after a set of step input.

*Special thanks to Léo V. and Vincent Q. for their help on this part.*

##### 2.2.2 Troubleshooting with an integral term in CubeIDE
The Ki parameter is not implemented correctly in the firmware package provided by ST. At board reset, the Ki parameter will be set to 0. In order to fix it, you need to modify the code in the file `YOUR-PROJECT-NAME/Application/User/mc_tasks.c` at line 540. You need to replace the following code:
```cpp
                    PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
```
by this code:
```cpp
                    PID_SetIntegralTerm(&PIDSpeedHandle_M1, X);
```
With X an integer being the value of Ki. For me Ki=2 was a good value (Your Ki value can be found with the methodology listed above and will be stored in the macro PID_SPEED_KI_DEFAULT inside `drive_parameters.h`). **X isn't Ki / KiDivisor**

### 3. Spinning in the right direction
At this point you should have a working motor that is able to spin at the right speed depending on the input command (given by the duty cycle of the PWM signal). However, in order to use the motor for a differential drive robot, you need to be able to spin both ways. Has the code generated previously is primarily designed to control drones, you need to modify it in order to be able to spin the motor in both directions.

My approach here was to modify the input processing function. By splitting in two the PWM duty cycle range, you can have a positive speed command for a duty cycle between 1060 and 1460 us and a negative speed command for a duty cycle between 1460 and 1860 us.


In order to change the direction you must set pHandle->hFinalSpeed to +hFinalSpeed or -hFinalSpeed inside function:
```cpp
__weak void MCI_ExecSpeedRamp(MCI_Handle_t *pHandle, int16_t hFinalSpeed, uint16_t hDurationms)
```
