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

## Project Structure
```bash
.
├── CubeMX/                 # Global configuration
├── libs/                    # Core source code
│   └── Sensors/            # Sensor drivers
├── targets/                # Target-specific code
├── tests/                  # Development and testing
└── docs/                   # Documentation
```

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
This step is important as it configures your vscode workspace to work with the stm32 tools. See the documenation on vscode's tasks and settings json files. 

1. Open VSCode Command Palette (Ctrl+Shift+P)
2. Type `STM32 Project Manager`
3. Under the project manager view of the stm32 extnesion, choose "Import CMake Project"
4. **[IMPORTANT]** Ensure that .vscode configuration generation completes (this generates important workspace settings for the stm32 extension)

### Next steps
- Review our [contributing guide](docs/contributing.md) 
- Learn the general stm32 HAL functions
- Make your own small projects and play around with flashing your target board; blinking the user led is always a good place to start


