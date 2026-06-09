# 配置服务器 (Config Server)

这是一个简化的配置服务器框架，只包含socket模块，用于系统配置管理。

## 目录结构

```
httpcopy/
├── include/              # 头文件目录
│   ├── network/         # 网络模块头文件
│   │   ├── errorNum.hpp
│   │   ├── frame.hpp
│   │   ├── moduleObj.hpp
│   │   ├── netWorkBase.hpp
│   │   └── tcpUdp.hpp
│   ├── socket.h         # Socket头文件
│   ├── data_type.h      # 数据类型定义
│   └── httplib.h        # HTTP库头文件
├── src/                  # 源代码目录
│   └── network/         # 网络模块实现
│       ├── networkBase.cpp
│       ├── socket.cpp
│       └── tcpUdp.cpp
├── main.cpp             # 主程序入口
├── peizhi.html          # 配置页面
├── CMakeLists.txt       # CMake构建文件
└── README.md            # 说明文档
```

## 功能特性

- ✅ 简化的HTTP服务器，只包含socket模块
- ✅ 配置页面 (peizhi.html) 绑定到根路径
- ✅ UDP Socket初始化和管理
- ✅ 配置保存API接口
- ✅ SLAM地图显示和路径打点功能
- ✅ 多地图切换功能（支持跨楼层导航）
- ✅ 换图点自动触发机制
- ✅ 简洁的架构设计，易于扩展

## 编译和运行

### 编译

```bash
mkdir build
cd build
cmake ..
make
```

### 运行

```bash
# 确保peizhi.html文件在同一目录下
./configserver
```

服务器将在 `0.0.0.0:8080` 上启动，可以通过浏览器访问 `http://localhost:8080` 查看配置页面。

## API接口

### 配置相关

### GET /
返回配置页面 (peizhi.html)

### GET /peizhi.html
直接访问配置页面

### POST /api/save_config
保存配置信息

请求体格式 (JSON):
```json
{
  "serverIp": "192.168.31.70",
  "httpPort": "8080",
  "udpPort": "8111",
  "socketType": "udp",
  "visionPort": "8870",
  "robotPort": "8880",
  "meterPort": "8889",
  "tempPort": "9010"
}
```

### 地图相关

### POST /api/upload_map
上传地图文件（SLAM推送）

支持格式：
- PGM格式：`Content-Type: image/pgm` 或 `image/x-portable-graymap`
- PNG格式：`Content-Type: image/png`

请求示例：
```bash
curl -X POST http://localhost:8080/api/upload_map \
  -H "Content-Type: image/png" \
  --data-binary @slam_map.png
```

### GET /api/map_image
获取当前地图图像（前端拉取）

优先返回PNG格式，其次PGM格式

### 路径打点相关

### GET /api/get_position
获取当前SLAM位置

响应：
```json
{
  "x": 5.123,
  "y": 3.456,
  "yaw": 1.234
}
```

### POST /api/add_point
添加路径点

请求：
```json
{
  "x": 5.123,
  "y": 3.456,
  "yaw": 1.234,
  "keypointflag": 0
}
```

### DELETE /api/undo_point
撤销最后一个点

### DELETE /api/clear_points
清空所有路径点

### GET /api/get_points
获取所有路径点列表

### GET /api/download_xml
下载当前地图的路径XML文件

### 地图切换相关

### POST /api/switch_map
切换到新地图

请求：
```json
{
  "map_name": "二楼"
}
```

### GET /api/get_current_map
获取当前地图信息

响应：
```json
{
  "map_id": 1,
  "xml_file": "map_1_waypoints.xml"
}
```

### POST /api/load_map
加载指定地图

请求：
```json
{
  "map_id": 2
}
```

## 配置说明

- **服务器IP**: HTTP服务器绑定的IP地址
- **HTTP端口**: HTTP服务器监听端口（默认8080）
- **UDP端口**: UDP Socket端口（默认8111）
- **Socket类型**: 支持UDP和TCP
- **设备端口**: 各种设备的通信端口

## 依赖

- C++17 或更高版本
- pthread (线程支持)
- cpp-httplib (单文件头库，已包含)

## 地图文件格式

系统支持两种地图图像格式：

### PGM格式（Portable Gray Map）
- 常用于ROS SLAM系统
- 灰度图像，0=障碍物，255=自由空间
- 文件示例：`slam_map.pgm`

### PNG格式（推荐）
- 标准图像格式，浏览器原生支持
- 文件更小（支持压缩）
- 文件示例：`slam_map.png`

详细格式说明请参考：[MAP_FORMAT.md](MAP_FORMAT.md)

### 地图格式转换工具

```bash
# 生成测试地图
python3 map_converter.py generate test_map.png --width 600 --height 400

# PGM转PNG
python3 map_converter.py pgm2png input.pgm output.png

# 上传地图
python3 map_converter.py upload slam_map.png --server http://localhost:8080
```

## 地图切换功能

### 概述
支持机器狗在不同楼层之间移动时自动切换地图和路径规划。详细使用说明请参考 [MAP_SWITCH_GUIDE.md](MAP_SWITCH_GUIDE.md)

### 快速开始

1. **在一楼建图打点**
   - 正常打点，最后一个点选择"换图点"（keypointflag=5）

2. **切换到二楼**
   - 点击"切换到新地图"按钮
   - 输入地图名称（如"二楼"）
   - 系统自动保存一楼路径到 `map_1_waypoints.xml`

3. **在二楼建图打点**
   - 继续打点，建立二楼路径
   - 路径保存到 `map_2_waypoints.xml`

4. **配置换图点**
   - 编辑 `map_switcher.cpp` 配置换图点坐标
   - 机器狗运行时自动检测并切换地图

### 点类型说明
- 0: 普通点
- 1: 充电点
- 2: 待梯点
- 3: 乘梯点
- 4: 出梯点
- 5: 换图点（触发地图切换）

### 测试
```bash
# 运行地图切换测试脚本
python3 test_map_switch.py
```

## 注意事项

1. 确保 `peizhi.html` 文件与可执行文件在同一目录下
2. 需要root权限才能绑定小于1024的端口
3. UDP Socket在初始化时绑定到指定端口
4. 地图切换时会自动保存当前路径，请确保有写入权限
5. 换图点坐标需要根据实际SLAM坐标系配置
