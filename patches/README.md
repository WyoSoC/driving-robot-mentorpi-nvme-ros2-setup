# Source Patches

These patches fix build failures in the Hiwonder-vendored `third_party/` packages
when compiling on Ubuntu 25.04 (GCC 14, CMake 3.31, Linux 6.14, Boost 1.87).

All patches apply to files inside `ros2_ws/src/third_party/`.

## Applying

```bash
cd ~/ros2_ws/src/third_party

# async_web_server_cpp — Boost forward-declaration fix
patch -p1 -d async_web_server_cpp < /path/to/0001-async_web_server_cpp-boost-array-include.patch

# ldlidar_stl_ros2 — Linux kernel header fix
patch -p1 -d ldlidar_stl_ros2 < /path/to/0002-ldlidar-linux-types-for-termios.patch

# ldlidar_stl_ros2 — missing pthread include
patch -p1 -d ldlidar_stl_ros2 < /path/to/0003-ldlidar-pthread-include.patch

# oradar_lidar — deprecated CMake policies
patch -p1 -d oradar_lidar < /path/to/0004-oradar-cmake-old-policies.patch

# oradar_lidar — CMP0026 LOCATION property removal
patch -p1 -d oradar_lidar < /path/to/0005-oradar-cmake-cmp0026-location.patch
```

## Patch details

| File | Root cause |
|---|---|
| `0001` — `async_web_server_cpp/include/.../http_connection.hpp` | Newer Boost provides only a forward declaration of `boost::array` via `<boost/lexical_cast.hpp>`. Added `#include <boost/array.hpp>` to force the full definition. |
| `0002` — `ldlidar_stl_ros2/.../serial_interface_linux.h` | Ubuntu 25.04 kernel headers: `<linux/termios.h>` now transitively includes `<linux/sched/types.h>` which needs `__u32`/`__u64` from `<linux/types.h>`. The include is inside a C++ namespace so normal guards don't rescue it. Added `#include <linux/types.h>` before the namespace block. |
| `0003` — `ldlidar_stl_ros2/.../log_module.cpp` | `pthread_mutex_init` and friends are used but `<pthread.h>` was never included (only commented out in the header). GCC 14 no longer provides implicit declarations. |
| `0004` — `oradar_lidar/sdk/CMakeLists.txt` | CMake 3.28+ makes it a hard error to set CMP0053, CMP0037, or CMP0043 to `OLD`. Changed all three to `NEW`. |
| `0005` — `oradar_lidar/sdk/cmake/install_package.cmake` | CMake 3.28+ also rejects `cmake_policy(SET CMP0026 OLD)`. CMP0026 guards reading `LOCATION` from a build target. Replaced `get_target_property(... LOCATION)` + `install(FILES)` with `install(TARGETS)`, and removed the unused `_lib` variable. |
