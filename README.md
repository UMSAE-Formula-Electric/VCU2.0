# Vehicle Control Unit (VCU)

## Introduction
This repository contains the code for the Vehicle Control Unit (VCU) of the Formula Electric SAE Student Team UMSAE Electric. 
The VCU is responsible for the control of the vehicle, including the motor controller, the steering system, and the brake system. 
The VCU is also responsible for the communication with the other ECUs in the vehicle, such as the Accumulator Control Unit (ACU) and the Sensor Control Unit (SCU).

## Cloning the repository
### Option 1: Using `git clone --recurse-submodules` (for newer Git versions)
```bash
git clone --recurse-submodules git@github.com:UMSAE-Formula-Electric/VCU2.0.git
```

### Option 2: Using `git clone` and `git submodule update --init --recursive` (for older Git versions)
```bash
git clone git@github.com:UMSAE-Formula-Electric/VCU2.0.git
cd VCU2.0
git submodule update --init --recursive
```

### Option 3: If you have already cloned the repository without submodules
```bash
cd VCU2.0
git submodule update --init --recursive
```

## Building the code
The code is built using STM32CubeIDE.

### Prerequisites
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)(optional)
