# Contributing

Thank you for considering contributing to this project. Your help is appreciated.

## How to Report a Bug

Open a [GitHub Issue](https://github.com/bokumentation/esp-idf-starter/issues) and include:

- A clear title and description
- Steps to reproduce the issue
- ESP-IDF version and target chip
- Any relevant logs or error output

## How to Submit Changes

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-change`)
3. Make your changes
4. Run the formatter (`pre-commit run --all-files` or `clang-format -i **/*.cpp **/*.h`)
5. Commit with a descriptive message
6. Push to your fork and open a Pull Request

## Code Style

- **C/C++**: Follow `.clang-format` rules (4-space indent, 120 column limit)
- **CMake**: Follow `.cmake-format.yaml` rules
- **General**: Follow `.editorconfig` settings (LF line endings, UTF-8)
- No trailing whitespace, final newline at end of file

## Commit Messages

Use present tense, imperative style:

```
Add LED blink example for ESP32-C3
Fix partition table offset calculation
Update SDK config defaults for ESP32-S3
```

## Pre-commit Hooks

The project includes a pre-commit configuration. To enable it:

```bash
pip install pre-commit
pre-commit install
```

This will automatically check formatting and linting on every commit.
