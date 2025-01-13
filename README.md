# PicoAPRS-Firmware
## 📋 Prerequisites
| Tool | Description | Required Version |
|------|-------------|------------------|
| Hardware | NUCLEO-L432KC |        |
| `VSCode` | Text Editor | Latest |
| `STM32CubeMX` | MCU Configuration Tool | 6.9.2+ |
| `STM32CubeCLT` | Command Line Tools | Latest |
| `STM32 MCU Finder` | Board/MCU Discovery | Latest |
| `CMake` | Build System | 3.24+ |
| `Ninja` | Build System Generator | Latest |
| `arm-none-eabi-gcc` | ARM Toolchain | Latest |
| `arm-none-eabi-g++` | ARM C++ Compiler | Latest |
| `arm-none-eabi-objcopy` | Binary Utilities | Latest |

Note: The ARM toolchain (`arm-none-eabi-*`) appears as dependencies in your toolchain file generate by the STM32 VS Code Extension and is essential for cross-compilation.


## 🚀 Getting Started

### Installation

1. Install VSCode from [official website](https://code.visualstudio.com/)

2. Install STM32 prerequisites:
   - STM32CubeMX
   - STM32CubeCLT
   - STM32 MCU Finder

3. Install VSCode STM32 Extension Pack:
   - Open VSCode Extensions (Ctrl+Shift+X)
   - Search for "STM32"
   - Install official STM32 extension pack

### ⚙️ Environment Configuration

1. Open Command Palette (Crtl+Shift+P)
2. Type `Preferences: Open User Settings (JSON)
3. Add these paths based on your installation locations:
For Windows:
```json
{
    "STM32VSCodeExtension.projectCreator.executablePath": "C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeMX\\STM32CubeMX.exe",
    "STM32VSCodeExtension.productFinder.executablePath": "C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeMCUFinder\\STM32CubeMCUFinder.exe",
    "STM32VSCodeExtension.cubeClt.Path": "C:\\ST\\STM32CubeCLT"
}
```

For Linux/MacOS:
```json
{
    "STM32VSCodeExtension.projectCreator.executablePath": "/opt/st/stm32cubemx/STM32CubeMX",
    "STM32VSCodeExtension.productFinder.executablePath": "/opt/st/stm32cubemcufinder/STM32CubeMCUFinder",
    "STM32VSCodeExtension.cubeClt.Path": "/opt/st/stm32cubeclt"
}
```
**Note: Actual paths may vary based on installation choices, these are just examples**

### 📥 Project Setup

```bash
# Clone repository
git clone https://github.com/coffee-and-telesense/PicoAPRS-Firmware.git

# Open in VSCode
code PicoAPRS-Firmware
```

### 🔧 Project Import

1. Open VSCode Command Palette (Ctrl+Shift+P)
2. Type `STM32 Project Manager`
3. Under the project manager view of the stm32 extnesion, choose "Import CMake Project"
4. **[IMPORTANT]** Ensure that .vscode configuration generation completes (this generates important workspace settings for the stm32 extension)

## 📁 Directory Structure

```
project-root/
├── CMakeFiles/              # CMake build system files
├── Drivers/                 # STM32 HAL and CMSIS drivers
│   ├── CMSIS/              # Core system files
│   └── STM32L4xx_HAL_Driver/
├── Inc/                     # Header files
├── Src/                     # Source files
├── Sensors/                 # Libs/Drivers for Sensors
├── build/                   # Build output directory
├── cmake/                   # CMake configuration files
├── docs/                    # Documentation
├── mcu-config.ioc          # STM32CubeMX configuration
├── CMakeLists.txt          # Main CMake configuration
├── startup_stm32l432xx.s   # Startup assembly
└── stm32l432kcux_flash.ld  # Linker script
```

## ⚠️ Important Notes
- Review STM32 extension documentation before starting
- TODO: Add More 