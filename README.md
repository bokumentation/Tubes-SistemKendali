<div align="center">

# ESP-IDF Starter Template

[![ESP32](https://img.shields.io/github/actions/workflow/status/bokumentation/esp-idf-starter/build.yml?job=build-esp32&branch=main&label=ESP32)](https://github.com/bokumentation/esp-idf-starter/actions/workflows/build.yml)
[![ESP32-C3](https://img.shields.io/github/actions/workflow/status/bokumentation/esp-idf-starter/build.yml?job=build-esp32c3&branch=main&label=ESP32-C3)](https://github.com/bokumentation/esp-idf-starter/actions/workflows/build.yml)
[![ESP32-S3](https://img.shields.io/github/actions/workflow/status/bokumentation/esp-idf-starter/build.yml?job=build-esp32s3&branch=main&label=ESP32-S3)](https://github.com/bokumentation/esp-idf-starter/actions/workflows/build.yml)
[![Debug](https://img.shields.io/github/actions/workflow/status/bokumentation/esp-idf-starter/build.yml?job=build-esp32-debug&branch=main&label=Debug)](https://github.com/bokumentation/esp-idf-starter/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/actions/workflow/status/bokumentation/esp-idf-starter/build.yml?job=build-esp32-release&branch=main&label=Release)](https://github.com/bokumentation/esp-idf-starter/actions/workflows/build.yml)

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-latest-cyan?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
[![License: Unlicense](https://img.shields.io/badge/license-Unlicense-blue.svg)](http://unlicense.org/)

My personal ESP-IDF project template with multi-target support, OTA partitions, debug/release build profiles, CI, and dev container support.

</div>

## Features

- Multi-chip support: ESP32, ESP32-C3, ESP32-S3
- Example component with math utilities (add, multiply, map) and LED blink
- Pre-configured debug and release build profiles
- Multi-flash-size partition tables (4MB, 8MB, 16MB)
- GitHub Actions CI for automated builds across all targets and profiles
- VS Code and Zed task definitions for common operations
- Clangd configuration for accurate C/C++ code navigation
- Pre-commit hooks for automatic code formatting
- Docker build support for reproducible environments

## Quick Start

```bash
git clone https://github.com/bokumentation/esp-idf-starter.git
cd esp-idf-starter
idf.py set-target esp32
idf.py build
idf.py flash
idf.py monitor
```

## Build Profiles

Build profiles are located at the project root:

| Profile | Usage | Log Level | Optimizations | Assertions | Best For |
|---------|-------|-----------|---------------|------------|----------|
| `sdkconfig.defaults.debug` | `idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.debug build` | Verbose | Debug (-Og) | Enabled | Development, debugging |
| `sdkconfig.defaults.release` | `idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.release build` | Errors only | Size (-Os) | Disabled | Production builds |

## Supported Targets

| Chip       | Config File                      |
|------------|----------------------------------|
| ESP32      | `sdkconfig.defaults.esp32`      |
| ESP32-C3   | `sdkconfig.defaults.esp32c3`    |
| ESP32-S3   | `sdkconfig.defaults.esp32s3`    |

## Directory Layout

```
├── main/                     # Application entry point
│   ├── main.cpp              # Main firmware logic
│   └── CMakeLists.txt        # Component registration
├── components/               # Reusable components
│   └── components_example/   # Example library component
├── extras/                   # Partition tables and configs
├── tools/                    # CI, Docker, clangd configs
├── .github/workflows/        # GitHub Actions CI
├── .vscode/                  # VS Code tasks
├── .zed/                     # Zed tasks
├── sdkconfig.defaults        # Base SDK configuration
├── sdkconfig.defaults.debug  # Debug build profile
├── sdkconfig.defaults.release# Release build profile
├── sdkconfig.defaults.esp32  # ESP32 target defaults
├── sdkconfig.defaults.esp32c3# ESP32-C3 target defaults
├── sdkconfig.defaults.esp32s3# ESP32-S3 target defaults
├── .clang-format             # C/C++ formatter rules
├── .cmake-format.yaml        # CMake formatter rules
├── .editorconfig             # Cross-editor settings
├── .gitignore
├── CMakeLists.txt
├── LICENSE
├── CHANGELOG.md
└── README.md
```

## Requirements

- [ESP-IDF](https://github.com/espressif/esp-idf) v5.x or later
- Python 3.8+
- CMake 3.16+
- Ninja build system

## Docker Build

You can build using the official ESP-IDF Docker image without installing the toolchain locally:

```bash
docker run --rm -v $(pwd):/project -w /project espressif/idf:latest idf.py set-target esp32
docker run --rm -v $(pwd):/project -w /project espressif/idf:latest idf.py build
```

## CI/CD

This project uses GitHub Actions for continuous integration. The workflow automatically builds firmware for all supported targets and profiles:

- **ESP32**, **ESP32-C3**, **ESP32-S3** — standard builds
- **Debug** — ESP32 with verbose logging and debug optimizations
- **Release** — ESP32 with minimal logging and size optimizations

Build artifacts are uploaded and available for download from the Actions tab.

## License

This is free and unencumbered software released into the public domain. See [LICENSE](LICENSE) for details.
