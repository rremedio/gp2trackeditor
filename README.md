# gp2trackeditor

![image](https://user-images.githubusercontent.com/61913443/77226824-21c21500-6b73-11ea-8453-a9d065919314.png)

## Building

### Prerequisites

- Visual Studio 2017 or later with the **Desktop development with C++** workload (includes MFC)
- CMake 3.15 or later

### Build with Visual Studio Generator

```bash
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A Win32
cmake --build . --config Debug
```

Adjust the `-G` generator string to match your Visual Studio version:

| Visual Studio Version | Generator String             |
|-----------------------|------------------------------|
| 2017                  | `Visual Studio 15 2017`      |
| 2019                  | `Visual Studio 16 2019`      |
| 2022                  | `Visual Studio 17 2022`      |

### Build with Ninja

Use a **Developer Command Prompt** or **vcvarsall.bat** to set up the MSVC environment first:

```bash
mkdir build
cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build .
```

For a release build (either generator):

```bash
# Visual Studio generator (multi-config)
cmake --build . --config Release

# Ninja (single-config — reconfigure with Release)
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

> **Note:** The project must be built as **Win32** (32-bit). MFC static linking and the MultiByte character set are configured automatically by CMake.
