# Extras

Additional configurations for the ESP-IDF Starter project.

## Partition Tables

Copy the desired partition table to your project root, then build with the custom partition table option:

```bash
cp extras/partitions-4mb.csv .
idf.py -DEXTRA_PARTITION_TABLE_CSV=partitions-4mb.csv build
```

| File | Flash Size | Recommended For |
|------|-----------|-----------------|
| `partitions-4mb.csv` | 4 MB | ESP32, ESP32-C3 |
| `partitions-8mb.csv` | 8 MB | ESP32-S3 |
| `partitions-16mb.csv` | 16 MB | ESP32-S3 (large projects) |

## Build Profiles

Build profiles are located at the project root. Use them directly without copying:

```bash
idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.debug build
```

| Profile | Log Level | Optimizations | Assertions | Best For |
|---------|-----------|---------------|------------|----------|
| `sdkconfig.defaults.debug` | Verbose | Debug (-Og) | Enabled | Development, debugging |
| `sdkconfig.defaults.release` | Errors only | Size (-Os) | Disabled | Production builds |
