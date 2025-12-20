# Hardware-Accelerated System for Inverse Kinematics Solution Based on LeRobot

[![FPGA](https://img.shields.io/badge/FPGA-PYNQ--Z2-blue)](https://www.pynq.io/)
[![HLS](https://img.shields.io/badge/HLS-Vitis-orange)](https://www.xilinx.com/products/design-tools/vitis.html)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

> **English** | **[中文文档](README_zh.md)**

## 📋 Overview

This system is an entry that won **the National First Prize** in the **AMD Track of the 2025 FPGA Innovation and System Design Competition**. The project builds upon and extends the Cholesky operator developed during the preliminary competition phase, applying it to real-world robotics applications.

In the preliminary round, our Cholesky operator optimization achieved remarkable results, securing the **best** in the Cholesky optimization task! See the performance comparison below (details in repository: [hlstrack2025_46387](https://github.com/E1pco/hlstrack2025_46387)):

#### Cholesky Operator Performance Comparison

| Performance Metric | Before Optimization | After Optimization | Improvement |
|-------------------|---------------------|--------------------|--------------|
| **Latency** | 4919 | 991 | 79.9% |
| **Execution Time** | 30871 | 4731 | 84.7% |
| **Initiation Interval (II)** | 696 | 124 | 82.2% |
| **Throughput** | 2.3e5 | 1.6e6 | 596% |

**Embodied intelligence** represents a crucial direction for the future of manufacturing, with **robotic arm control** as its foundational element. The core challenge lies in driving the robotic arm from its current pose to a target pose, which requires solving for the joint servo rotation angles based on pose differences—known as the **inverse kinematics (IK) problem**.

To address this challenge, we have designed a custom **IK solver IP core** deployed on the programmable logic (PL) of the PYNQ-Z2 board, leveraging its highly parallel computing capabilities to accelerate the IK solving process through hardware acceleration.

### 🎯 Key Features

- ✅ **Hardware Acceleration**: FPGA-based Levenberg-Marquardt inverse kinematics solver
- ✅ **High Performance**: Single-step iteration speed improved by approximately **10×** compared to pure software execution
- ✅ **Optimization Strategies**: Utilizes PIPELINE, UNROLL, and other HLS optimization techniques
- ✅ **Complete Solution**: Includes robotic arm control, teleoperation, and performance testing
- ✅ **Easy Deployment**: Based on PYNQ framework with Jupyter Notebook interactive development

### 🔬 Optimization Methodology

Our optimization strategy combines foundational techniques such as PIPELINE and UNROLL with insights gained from our previous work on Cholesky operator hardware acceleration. Through comparative testing, the optimized IK solver running on the PL achieves an average single-step iteration speed improvement of approximately **10×** compared to pure software execution (entirely on PS).

### 📊 Project Showcase

The project poster and flowchart are shown below:

<div align="center">
  <img src="others/poster.png" alt="Project Poster" width="80%">
  <p><em>Figure 1: Project Poster</em></p>
</div>

<div align="center">
  <img src="others/flowchart.png" alt="Design Flowchart" width="80%">
  <p><em>Figure 2: System Design Flowchart</em></p>
</div>

---

## 📁 Project Structure

```
Hardware-Accelerated_System_for_IK_Solution_Based_on_LeRobot/
├── notebook/                                    # Jupyter notebooks and bitstream files
│   ├── design_1.bit                            # FPGA bitstream file
│   ├── design_1.hwh                            # Hardware description file
│   ├── Dual_Arm_TCP_Receiver.ipynb             # Dual-arm teleoperation control notebook
│   ├── LM_SingleStep_Benchmark.ipynb           # Single-step IK performance test
│   ├── SO101_Hardware_IK_Demo.ipynb            # Complete IK demonstration
│   ├── SO101_IK_HW_vs_Python.ipynb             # Hardware vs Python performance comparison
│   └── test_results/                           # Performance test result charts
│       ├── benchmark_hw_vs_py.png              # Hardware vs Software comparison
│       ├── ik_repeat_test.png                  # Repeatability test results
│       ├── ik_solver_comparison.png            # Solver comparison
│       ├── ik_solver_distribution.png          # Solution distribution
│       ├── lm_scaling_benchmark.png            # LM algorithm scalability test
│       └── lm_single_step_benchmark.png        # Single-step performance benchmark
│
├── others/                                      # Auxiliary scripts, drivers, and tools
│   ├── driver/                                 # Servo drivers and configuration files
│   │   ├── ftservo_controller.py               # FeeTech servo controller
│   │   ├── ftservo_driver.py                   # FeeTech servo driver
│   │   ├── __init__.py                         # Python package initialization
│   │   ├── left_arm.json                       # Left arm configuration
│   │   └── right_arm.json                      # Right arm configuration
│   ├── hw_solver_wrapper.py                    # Hardware solver Python wrapper
│   ├── ik/                                     # Pure Python IK implementation
│   │   ├── base.py                             # Base classes and data structures
│   │   ├── et.py                               # Elementary transformations
│   │   ├── __init__.py                         # Package initialization
│   │   ├── robot.py                            # Robot kinematic model
│   │   ├── solvers.py                          # IK solver implementations
│   │   └── utils.py                            # Utility functions
│   ├── joycon_robotics/                        # JoyCon controller driver
│   ├── TCP.py                                  # TCP client (PC-side)
│   ├── poster.png                              # Project poster
│   └── flowchart.png                           # Flowchart
│
├── src/                                         # Source code
│   ├── ik_solver_hls/                          # HLS hardware IK solver implementation
│   │   ├── description.json                    # IP core description file
│   │   ├── hls_config.cfg                      # HLS configuration file
│   │   ├── lm_solver_so101.hpp                 # LM solver header file
│   │   ├── tb.cpp                              # C++ testbench
│   │   ├── vitis-comp.json                     # Vitis component configuration
│   │   ├── xf_solver_L1.hpp                    # Vitis L1 library header
│   │   └── xf_solver_lm_so101.cpp              # Hardware solver implementation
│   └── overlay/                                # Vivado project files
│       ├── design_1.tcl                        # Block Design build script
│       └── ik.tcl                              # Vivado build script
│
└── README.md                                    # Project documentation (this file)
```

---

## 🛠️ Hardware Requirements

### Required Hardware

| Hardware | Model/Specification | Description |
|----------|---------------------|-------------|
| **FPGA Board** | PYNQ-Z2 | Xilinx Zynq-7020 SoC |
| **Robotic Arm** | SO-101 (x2) | 5-DOF robotic arm (includes servos, driver board, and structure) |
| **Servo Driver Board** | FeeTech compatible | For controlling servos |
| **PC Computer** | With Bluetooth and Gigabit Ethernet | For development and teleoperation |

### Optional Hardware

| Hardware | Description |
|----------|-------------|
| **JoyCon Controllers** | A pair of Nintendo Switch JoyCon controllers for teleoperation demonstration |
| **Ethernet Cable** | Gigabit Ethernet cable for connecting PC and PYNQ board |

---

## 💻 Software Requirements

### PYNQ Environment

- **PYNQ Version**: 2.7 or higher (for PYNQ-Z2)
- **Python Version**: 3.6+
- **Jupyter Notebook**: Included in PYNQ image

### Python Dependencies

Main dependencies include:

```python
# Core dependencies
numpy>=1.19.0
scipy>=1.5.0

# Hardware interface
pynq>=2.7.0

# Servo control (custom)
driver.ftservo          # FeeTech servo driver

# Inverse kinematics (custom)
ik.robot                # Robot kinematic model
ik.utils                # Utility functions
ik.solvers              # IK solvers
ik.et                   # Elementary transformations

# Hardware solver wrapper (custom)
hw_solver_wrapper       # Top-level wrapper function
```

### Development Tools (Optional)

If you need to recompile the HLS IP core:

- **Vitis HLS**: 2021.2 or higher
- **Vivado**: 2021.2 or higher

---

## 🚀 Quick Start

### Step 1: Hardware Setup

1. **Bluetooth Connection** (optional): Pair JoyCon controllers with PC via Bluetooth
2. **Network Connection**: Connect PC and PYNQ-Z2 board using Gigabit Ethernet cable
3. **Servo Connection**: Connect PYNQ board to servo driver board via UART interface
4. **Power Supply**:
   - Power the PYNQ-Z2 board (5V/2A)
   - Power the servo driver board (according to servo specifications)

### Step 2: File Upload

Upload the following folders and files to the PYNQ-Z2 board (via Jupyter or SSH):

```bash
# Upload to /home/xilinx/ directory on the board
notebook/                    # Contains .bit, .hwh, and .ipynb files
others/driver/               # Servo drivers
others/ik/                   # Inverse kinematics library
others/hw_solver_wrapper.py  # Hardware solver wrapper
```

### Step 3: Launch Jupyter Notebook

1. Open PYNQ board's Jupyter Notebook in a browser:
   ```
   http://<PYNQ-IP>:9090
   ```
   Default password: `xilinx`

2. Navigate to the `notebook/` folder

### Step 4: Run Demonstrations

Select any of the following notebooks to run:

#### 🎮 Demo Notebooks

- **`SO101_Hardware_IK_Demo.ipynb`**  
  Complete hardware-accelerated inverse kinematics demonstration showing how to use FPGA to accelerate IK solving

- **`Dual_Arm_TCP_Receiver.ipynb`**  
  Dual-arm teleoperation demonstration (requires JoyCon controllers)

#### 📊 Performance Test Notebooks

- **`SO101_IK_HW_vs_Python.ipynb`**  
  Performance comparison between hardware acceleration and pure Python software

- **`LM_SingleStep_Benchmark.ipynb`**  
  Single-step LM iteration performance benchmark test

### Step 5: View Results

After running notebooks, performance test results will be saved in the `notebook/test_results/` folder.

---

## 📈 Performance Data

### Performance Improvement Summary

Comparative testing reveals that compared to pure software implementation (algorithm executing entirely on PS), the hardware-accelerated IK algorithm running in parallel on PL demonstrates significant performance improvements:

| Performance Metric | Pure Software (PS) | Hardware Accelerated (PL) | Speedup |
|-------------------|-------------------|---------------------------|---------|
| **Single-Step Iteration Speed** | Baseline | Improved | **~10×** |
| **Average Iteration Count** | Baseline | Reduced | **~4.7×** |
| **Average Iteration Time** | Baseline | Reduced | **~7.9×** |
| **Throughput** | Baseline | Improved | **~9.2×** |
| **Average Execution Time** | Baseline | Reduced | **~9.3×** |
| **Overall Speedup** | 1.0× | Accelerated | **~2.5×** |

> **Note**: Specific performance data may vary depending on test scenarios and target poses. For detailed charts and data, please refer to `notebook/test_results/`.

### Performance Test Methodology

All performance data were obtained by running the following notebooks:

- `LM_SingleStep_Benchmark.ipynb` - Single-step performance testing
- `SO101_IK_HW_vs_Python.ipynb` - Complete solver performance comparison

### Performance Charts

<div align="center">
  <img src="notebook/test_results/lm_single_step_benchmark.png" alt="Single-Step LM Benchmark" width="70%">
  <p><em>Figure 3: Single-Step LM Algorithm Performance Benchmark</em></p>
</div>

<div align="center">
  <img src="notebook/test_results/lm_scaling_benchmark.png" alt="LM Scaling Benchmark" width="70%">
  <p><em>Figure 4: LM Algorithm Scaling Performance</em></p>
</div>

<div align="center">
  <img src="notebook/test_results/ik_solver_comparison.png" alt="IK Solver Comparison" width="70%">
  <p><em>Figure 5: IK Solver Performance Comparison</em></p>
</div>

<div align="center">
  <img src="notebook/test_results/ik_solver_distribution.png" alt="IK Solver Distribution" width="70%">
  <p><em>Figure 6: IK Solution Distribution Analysis</em></p>
</div>

<div align="center">
  <img src="notebook/test_results/ik_repeat_test.png" alt="IK Repeatability Test" width="70%">
  <p><em>Figure 7: IK Solver Repeatability Test Results</em></p>
</div>

---

## 🏗️ System Architecture

### Hardware Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    PYNQ-Z2 Board                         │
│  ┌──────────────────────────┬──────────────────────┐   │
│  │      Processing System   │  Programmable Logic  │   │
│  │          (PS)            │        (PL)          │   │
│  │                          │                      │   │
│  │  • Python control code   │  • IK solver IP core │   │
│  │  • Jupyter Notebook      │  • AXI-Lite interface│   │
│  │  • Driver programs       │  • Parallel compute  │   │
│  │                          │  • Cholesky decomp   │   │
│  └────────┬─────────────────┴───────┬──────────────┘   │
│           │        AXI Bus          │                   │
└───────────┼─────────────────────────┼───────────────────┘
            │                         │
            ▼                         ▼
      UART Interface              MMIO Registers
            │                         │
            ▼                         ▼
     Servo Driver Board          Config/Control
```

### Software Architecture

```
┌─────────────────────────────────────────────────────┐
│              Jupyter Notebook Layer                  │
│  (User interaction, visualization, demos)            │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────┐
│           Python Application Layer                   │
│  • robot.py      - Robot model                      │
│  • solvers.py    - Software IK solvers              │
│  • hw_solver_wrapper.py - Hardware solver wrapper   │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────┐
│      Hardware Abstraction Layer (HAL)                │
│  • PYNQ Overlay  - Bitstream loading                │
│  • MMIO          - Register access                  │
│  • driver        - Servo control                    │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────┐
│              FPGA Hardware Layer                     │
│  • xf_solver_lm_so101 IP core                       │
│  • AXI-Lite slave interface                         │
└─────────────────────────────────────────────────────┘
```

---

## 🔧 IP Core Interface Specification

### AXI-Lite Register Map

The hardware IK solver is accessed via AXI-Lite interface with the following register map:

#### Input Registers

| Offset | Name | Type | Description |
|--------|------|------|-------------|
| 0x10 | sin0 | float | Sine of joint 0 angle |
| 0x18 | sin1 | float | Sine of joint 1 angle |
| 0x20 | sin2 | float | Sine of joint 2 angle |
| 0x28 | sin3 | float | Sine of joint 3 angle |
| 0x30 | sin4 | float | Sine of joint 4 angle |
| 0x38 | cos0 | float | Cosine of joint 0 angle |
| 0x40 | cos1 | float | Cosine of joint 1 angle |
| 0x48 | cos2 | float | Cosine of joint 2 angle |
| 0x50 | cos3 | float | Cosine of joint 3 angle |
| 0x58 | cos4 | float | Cosine of joint 4 angle |
| 0x60 | e0 | float | Error vector component 0 |
| 0x68 | e1 | float | Error vector component 1 |
| 0x70 | e2 | float | Error vector component 2 |
| 0x78 | e3 | float | Error vector component 3 |
| 0x80 | e4 | float | Error vector component 4 |
| 0x88 | e5 | float | Error vector component 5 |
| 0x90 | lambda | float | LM damping factor |

#### Output Registers

| Offset | Name | Type | Description |
|--------|------|------|-------------|
| 0x98 | d0 | float | Joint delta 0 |
| 0xA8 | d1 | float | Joint delta 1 |
| 0xB8 | d2 | float | Joint delta 2 |
| 0xC8 | d3 | float | Joint delta 3 |
| 0xD8 | d4 | float | Joint delta 4 |
| 0xE8 | status | int | Execution status code (0=success, 1=failure) |
| 0xF0 | state | int | Runtime state (bit flags) |

### Usage Example

```python
from pynq import Overlay

# Load bitstream
overlay = Overlay('design_1.bit')

# Access IK solver
solver_ip = overlay.xf_solver_lm_so101_0

# Write input data
solver_ip.write(0x10, sin_values[0])  # sin0
solver_ip.write(0x60, error[0])       # e0
solver_ip.write(0x90, lambda_value)   # lambda

# Start computation (write control register)
solver_ip.write(0x00, 0x01)

# Wait for completion
while not (solver_ip.read(0x00) & 0x02):
    pass

# Read results
delta = [solver_ip.read(0x98 + i*0x10) for i in range(5)]
status = solver_ip.read(0xE8)
```

---

## 📚 References

1. **Wei Liu** - PYNQ Chinese Documentation  
   GitHub: https://github.com/louisliuwei/PynqDocs

2. **Wei Liu** - FPGA Parallel Programming  
   GitHub: https://github.com/sazczmh/pp4fpgas-cn

3. **AMD, Inc.** - Vitis High-Level Synthesis User Guide (UG1399)  
   Documentation: https://docs.amd.com/r/en-US/ug1399-vitis-hls/HLS-Programmers-Guide

4. **LeRobot** - Hugging Face Robotics Learning Framework  
   GitHub: https://github.com/huggingface/lerobot

5. **JoyCon Robotics** - Nintendo Switch JoyCon Robot Control Library  
   GitHub: https://github.com/box2ai-robotics/joycon-robotics

6. **hlstrack2025** - Preliminary Cholesky Operator Optimization  
   GitHub: https://github.com/E1pco/hlstrack2025_46387

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Contributors

We thank all developers who contributed to this project!

For questions or suggestions, feel free to submit an issue or pull request.

---

## 🙏 Acknowledgments

Special thanks to:

- AMD/Xilinx for providing development tools and technical support
- PYNQ community for the open-source framework
- All authors and contributors of referenced projects

---

<div align="center">
  <p>⭐ If this project helps you, please give us a star!</p>
</div>
