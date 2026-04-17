# Electronic Race Vision

基于 OpenCV 的赛车视觉巡线系统，适用于嵌入式 Linux 平台（如 Rockchip RK3588）。

## 功能

- 摄像头实时采集，水平扫描条带提取赛道中心线
- EMA 平滑滤波，输出航向角通过串口发送给底盘控制器
- 可选 Rockchip RGA 硬件加速（自动检测 librga）
- HTTP MJPEG 推流，支持远程实时预览
- GStreamer UDP 推流（可选硬件编码）
- 支持图片/视频文件离线调试

## 依赖

- CMake >= 3.10
- OpenCV >= 4.x
- （可选）librga — Rockchip RGA 加速

## 编译

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

禁用 RGA：

```bash
cmake -DENABLE_RGA=OFF ..
```

## 用法

```
./build/car [image.jpg|video.mp4] [/dev/ttyS*] [选项]

选项:
  --headless              无界面模式
  --no-rga                禁用 RGA 加速
  --camera=N              摄像头索引（默认 0）
  --camera-device=/dev/videoX  指定摄像头设备
  --gst-udp=HOST:PORT     GStreamer UDP 推流目标
  --gst-sw                使用软件编码（默认硬件）
  --http-mjpeg=PORT       启动 HTTP MJPEG 服务
  --http-quality=1..100   MJPEG 质量（默认 70）
  --http-scale=0.1..1.0   推流缩放比例
  --http-fps=N            推流帧率限制
```

快速启动（使用 `/dev/video0`，HTTP 预览端口 8080，串口 `/dev/ttyS2`）：

```bash
./start.bash
```

## 参数调整

主要参数集中在 `car.cpp` 顶部的配置区：

| 参数 | 说明 |
|------|------|
| `NUM_STRIPS` | 水平扫描条带数 |
| `SEARCH_WIDTH` | 质心搜索半宽（px） |
| `EMA_ALPHA` | EMA 平滑系数 |
| `LOOKAHEAD_RATIO` | 前瞻控制点位置比例 |
| `ENABLE_UNDISTORT` | 是否启用畸变校正 |

## 串口协议

航向角以 `0.01°` 为单位编码后发送，范围 ±90°。
