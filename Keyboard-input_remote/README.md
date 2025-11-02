# KeySender

ESP32 serial keyboard sender for VI-ROSE.

## Build Instructions

### Prerequisites
- CMake 3.15+
- C++17 compiler (GCC/Clang/MSVC)
- SDL2

### Windows (MinGW)
```bash
# Install SDL2 via MSYS2
pacman -S mingw-w64-x86_64-SDL2

# Build
cmake -B build -G "MinGW Makefiles"
cmake --build build --config Release
```

### Linux
```bash
# Install SDL2
sudo apt install libsdl2-dev  # Debian/Ubuntu
sudo dnf install SDL2-devel   # Fedora

# Build
cmake -B build
cmake --build build --config Release
```

## Usage
```bash
./KeySender  # or KeySender.exe on Windows
```