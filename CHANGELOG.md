# Changelog

## [0.1.2] - 2026-05-25

### Added
- Doxygen documentation for all component API functions
- SPDX license headers in source files
- `extras/README.md` with usage instructions for partition tables and build profiles

### Changed
- Updated `sdkconfig.defaults.debug` usage comment to reflect extras path
- Updated `sdkconfig.defaults.release` usage comment to reflect extras path

## [0.1.1] - 2026-05-25

### Added
- Enhanced Zed tasks with `save`, `hide`, and `allow_concurrent_runs` options

### Changed
- Moved VS Code tasks to root `.vscode/tasks.json`
- Moved devcontainer config to root `.devcontainer/`
- Expanded README with project features and getting started guide
- Improved code style consistency across source files
- Updated `.gitignore` patterns

### Removed
- Duplicate `.gitignore` patterns

## [0.1.0] - 2026-05-25

### Added
- Initial ESP-IDF project template with ESP32, ESP32-C3, and ESP32-S3 support
- `components_example` component with math operations (add, multiply, map) and LED blink
- GitHub Actions CI workflow for automated ESP32 builds
- VS Code tasks for build, flash, monitor, menuconfig, and erase actions
- Zed tasks for build, flash, monitor, menuconfig, and erase actions
- Pre-commit hooks configuration for code quality
- Clangd configuration for C/C++ language server support
- Debug and release SDK config presets
- Multi-flash-size partition table presets (4MB, 8MB, 16MB)
- Docker-based build support
