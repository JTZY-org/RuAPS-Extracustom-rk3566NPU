# Extracustom-rk3566NPU 开发日志

> 记录时间：2026-07-08  
> 目标板：RK3566 / KPOKY 1.0 (Yocto/Poky) / PREEMPT_RT / aarch64  
> 主机：x86_64 Ubuntu，16核32线程

---

## 一、Python 集成 (CPython Embedding)

### 背景
将 Python 脚本引入 C++ 主程序，实现用户层逻辑可热更新。

### 实现方案
- 在 `src/python/PythonEngine.hpp` / `src/python/PythonEngine.cpp` 中封装了 `PythonEngine` 类，管理 CPython 运行时生命周期。
- Python 调用入口为 `user_app.py`，定义两个生命周期函数：
  - `init(width, height, pixfmt)` — 由 C++ 的 `UserAppInit` 触发
  - `exchange(frame_bytes, width, height, pixfmt, telemetry)` — 由 C++ 的 `UserAppExChange` 每帧触发
- 修改了 `UserApp.cpp`，在 `UserAppInit` 和 `UserAppExChange` 中实例化并调用 `g_pythonEngine`。

### CMake 修改
- `CMakeLists.txt` 添加了 Yocto SDK 目标 sysroot 的 Python 3.13 头文件和库路径，避免链接到主机 x86_64 Python 库（交叉编译陷阱）。
- 将 `src/python/PythonEngine.cpp` 加入 `UserApp` 库编译目标。
- 将 `user_app.py` 加入 CPack 的 `/etc/rknn` 安装路径。

---

## 二、APM 模块（C++ → Python 绑定）

### 背景
将 `Public/PLGUserDefine.hpp` 中定义的飞控接口全部暴露给 Python。

### 实现
在 `PythonEngine.cpp` 中通过 `PyImport_AppendInittab` 动态注册内置模块 `apm`：

| Python 函数 | C++ 对应 |
|---|---|
| `apm.arm()` | `APMControllerARM()` |
| `apm.disarm()` | `APMControllerDISARM()` |
| `apm.position(x,y,z)` | `APMControllerPosition()` |
| `apm.speed(vx,vy,vz)` | `APMControllerSpeed()` |
| `apm.servo(ch,val)` | `APMControllerServo()` |
| `apm.broadcast(data_dict)` | `pushBoradcastData()` |

`apm` 模块在 C++ 程序启动时注册，Python 脚本可直接 `import apm` 使用。

---

## 三、NumPy + OpenCV 部署到目标板

### 背景
目标板（Yocto KPOKY 1.0）极其精简，无 pip，无 numpy，无 OpenCV。

### 解决过程（踩过的坑）

#### 1. 使用 opkg 安装失败 → 依赖地狱
尝试将 Yocto 编译的 `.ipk` 文件推送到板端用 `opkg install` 安装，但遭遇多级依赖缺失：
- `python3-opencv` → 需要 `tbb` → 需要 `libhwloc15` → 需要 `libpciaccess0`
- 还需要 `gstreamer`、`libgtk-3`、`libcairo` 等桌面图形库

每次 `opkg` 只报告一层依赖缺失，多次循环无法解决。

#### 2. 精简 OpenCV：禁用 GUI 和流媒体

在 `/home/user/build/poky/build/conf/local.conf` 添加：
```bitbake
PACKAGECONFIG:pn-opencv = "python3 eigen jpeg png"
```
去除 GTK3、GStreamer、Cairo 等所有 GUI 相关依赖，重新编译 OpenCV（Headless 版本）。

#### 3. 扩容 rootfs 分区

开发板 `/dev/mmcblk0p4` 只有 306MB（已用 100%），通过 `resize2fs` 在线扩容到 **13.5GB**：
```bash
resize2fs /dev/mmcblk0p4
```

#### 4. 物理文件拷贝方案（绕过 opkg）

从主机 Yocto 编译好的 rootfs 目录直接物理拷贝所有 `.so` 和 Python 包到开发板：
```
/home/user/build/poky/build/tmp/work/rk3566_acropi-poky-linux/core-image-base/1.0/rootfs/
```

逐步补充的缺失库：
- `libwebp.so.7`, `libwebpmux.so.3`, `libwebpdemux.so.2`
- `libsharpyuv.so.0`
- `libunwind.so.8`
- 整个 `cv2` Python 包目录（旧版含 GStreamer 引用，需完整替换）

#### 5. 最终验证通过
```
numpy: 2.2.3  cv2: 4.11.0 - ALL OK!
```

---

## 四、Yocto 编译配置优化

### 线程数修正
原配置 `BB_NUMBER_THREADS = "8"`，主机实为 16核32线程，修正为：
```bitbake
BB_NUMBER_THREADS = "32"
PARALLEL_MAKE = "-j 32"
```

---

## 五、当前状态（2026-07-08）

| 组件 | 状态 |
|---|---|
| C++ `PythonEngine` 封装 | ✅ 完成 |
| `apm` Python 模块绑定 | ✅ 完成 |
| `user_app.py` 模板 | ✅ 完成 |
| CMake 交叉编译配置 | ✅ 完成 |
| 目标板 `numpy 2.2.3` | ✅ 已安装 |
| 目标板 `cv2 4.11.0 Headless` | ✅ 已安装 |
| 目标板 rootfs 扩容 (13.5GB) | ✅ 完成 |
| Yocto 32线程编译配置 | ✅ 完成 |

---

## 六、注意事项

- `apm` 模块**不能**在裸 Python 解释器中 `import`，必须由主程序 `libUserApp.so` 加载后才有效。
- OpenCV 是 **Headless** 版本，不支持 `cv2.imshow()`、GStreamer 摄像头读取等功能。如需图像显示，直接使用 V4L2 + RGA 硬件路径。
- 每次修改 `user_app.py` 后，执行 `./deploy.sh` 即可热更新到板端 `/etc/rknn/user_app.py`，无需重新编译 C++。
