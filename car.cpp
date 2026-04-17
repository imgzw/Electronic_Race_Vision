#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>


#if defined(HAVE_RGA)
#if __has_include(<im2d.hpp>)
#include <im2d.hpp>
#include <im2d_type.h>
#elif __has_include(<im2d.h>)
#include <im2d.h>
#include <im2d_type.h>
#elif __has_include(<rga/im2d.hpp>)
#include <rga/im2d.hpp>
#include <rga/im2d_type.h>
#elif __has_include(<rga/im2d.h>)
#include <rga/im2d.h>
#include <rga/im2d_type.h>
#else
#undef HAVE_RGA
#endif
#endif

using namespace std;
using namespace cv;

// 运行模式 0-9，由蓝牙端直接设置，透传到串口
static atomic<uint8_t> g_mode{0};

// ============ 参数配置 ============
// 依据摄像头规格书，选用 1280x720 满血宽幅视野，获取极佳的前瞻可视面积
const int FRAME_W = 1280;
const int FRAME_H = 720;
const int NUM_STRIPS = 12;    // 水平扫描条带数 (恢复稳定版)
const int SEARCH_WIDTH = 150; // 对应 1280 画幅放大的质心搜索半宽
const int MAX_SEARCH_WIDTH = 270;
const double EMA_ALPHA = 0.3;   // EMA平滑系数 
const int MIN_VALID_POINTS = 3; // 恢复丢线阈值
const double ROI_TOP_RATIO = 0.1; // 稍微让开极点，防止视野无用畸变
const double ROI_BOTTOM_RATIO = 0.95;
const double LOOKAHEAD_RATIO = 0.4;  // 既然是纯俯视，前瞻控制点应该设在画面中上方（例如 0.4）来主导拐弯
const double OUTLIER_REJECT_PX = 45.0; // 宽幅下飞点容忍度扩大至 45px
const int LOST_CONFIRM_FRAMES = 3;
const double SERIAL_HEADING_SCALE = 100.0; // 串口载荷单位: 0.01 deg
const double SERIAL_HEADING_LIMIT_DEG = 90.0;
const int HUD_PANEL_W = 320;
const int HUD_PANEL_H = 265;
const int BINARY_PREVIEW_W = 280;
const int BINARY_PREVIEW_H = 140;

// 相机畸变校正参数 (目前提供一组普通广角镜头的经验值，您可根据实际标定结果修改)
// 性能警告：800x600的3通道全彩色去畸变十分消耗CPU(耗时10ms+)，若非精准测距强烈建议关闭以换取高帧率
const bool ENABLE_UNDISTORT = false;
const double CAM_FX = 550.0; // 焦距 X
const double CAM_FY = 550.0; // 焦距 Y
const double CAM_CX = 400.0; // 光心 X
const double CAM_CY = 300.0; // 光心 Y
const double CAM_K1 = -0.35; // 桶形畸变系数 (负值校正向外凸起的畸变)
const double CAM_K2 = 0.10;
const double CAM_P1 = 0.0;
const double CAM_P2 = 0.0;
const double CAM_K3 = 0.0;

struct RuntimeOptions {
  string input_path;
  string serial_port;
  int camera_index = 0;
  string camera_device;
  string gst_udp_host;
  int gst_udp_port = 0;
  bool gst_use_hw = true;
  int http_mjpeg_port = 0;
  int http_mjpeg_quality = 70;
  double http_mjpeg_scale = 1.0;
  double http_mjpeg_fps = 0.0;
  bool headless = false;
  bool disable_rga = false;
};

void printUsage(const char *prog) {
  cout << "用法: " << prog
       << " [image.jpg|video.mp4] [/dev/ttyS*] [--headless] [--no-rga] "
          "[--camera=N] [--camera-device=/dev/videoX]"
       << endl;
  cout << "             [--gst-udp=HOST:PORT] [--gst-sw] [--http-mjpeg=PORT]"
       << endl;
  cout << "             [--http-quality=1..100] [--http-scale=0.1..1.0] "
          "[--http-fps=N]"
       << endl;
  cout << "  不传输入源时默认打开摄像头" << endl;
}

RuntimeOptions parseArgs(int argc, char **argv) {
  RuntimeOptions options;

  for (int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if (arg == "--headless") {
      options.headless = true;
    } else if (arg == "--no-rga") {
      options.disable_rga = true;
    } else if (arg == "--gst-sw") {
      options.gst_use_hw = false;
    } else if (arg.rfind("--camera=", 0) == 0) {
      options.camera_index = stoi(arg.substr(9));
    } else if (arg.rfind("--camera-device=", 0) == 0) {
      options.camera_device = arg.substr(16);
    } else if (arg.rfind("--gst-udp=", 0) == 0) {
      string target = arg.substr(10);
      size_t colon = target.rfind(':');
      if (colon != string::npos) {
        options.gst_udp_host = target.substr(0, colon);
        options.gst_udp_port = stoi(target.substr(colon + 1));
      }
    } else if (arg.rfind("--http-mjpeg=", 0) == 0) {
      options.http_mjpeg_port = stoi(arg.substr(13));
    } else if (arg.rfind("--http-quality=", 0) == 0) {
      options.http_mjpeg_quality = max(1, min(100, stoi(arg.substr(15))));
    } else if (arg.rfind("--http-scale=", 0) == 0) {
      options.http_mjpeg_scale = max(0.1, min(1.0, stod(arg.substr(13))));
    } else if (arg.rfind("--http-fps=", 0) == 0) {
      options.http_mjpeg_fps = max(0.0, stod(arg.substr(11)));
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      exit(0);
    } else if (arg.rfind("/dev/", 0) == 0) {
      options.serial_port = arg;
    } else if (options.input_path.empty()) {
      options.input_path = arg;
    } else {
      cerr << "警告: 忽略未知参数 " << arg << endl;
    }
  }

  return options;
}

#if defined(HAVE_RGA)
static bool resizeWithRga(const Mat &src, Mat &dst) {
  if (src.empty())
    return false;
  if (src.type() != CV_8UC3 && src.type() != CV_8UC1)
    return false;

  int src_format =
      (src.type() == CV_8UC3) ? RK_FORMAT_BGR_888 : RK_FORMAT_YCbCr_400;
  int dst_format =
      (dst.type() == CV_8UC3) ? RK_FORMAT_BGR_888 : RK_FORMAT_YCbCr_400;

  rga_buffer_t src_buf =
      wrapbuffer_virtualaddr(src.data, src.cols, src.rows, src_format);
  rga_buffer_t dst_buf =
      wrapbuffer_virtualaddr(dst.data, dst.cols, dst.rows, dst_format);

  IM_STATUS status = imresize(src_buf, dst_buf);
  if (status != IM_STATUS_SUCCESS) {
    cerr << "RGA resize 失败，回退到 OpenCV。status="
         << static_cast<int>(status) << endl;
    return false;
  }
  return true;
}
#endif

static bool resizeFrameWithAccel(const Mat &src, Mat &dst, bool enable_rga) {
  if (src.empty())
    return false;

  if (src.cols == FRAME_W && src.rows == FRAME_H) {
    dst = src.clone();
    return false;
  }

  dst.create(FRAME_H, FRAME_W, src.type());

#if defined(HAVE_RGA)
  if (enable_rga && resizeWithRga(src, dst)) {
    return true;
  }
#else
  (void)enable_rga;
#endif

  resize(src, dst, Size(FRAME_W, FRAME_H), 0, 0, cv::INTER_LINEAR);
  return false;
}

static void drawLabel(Mat &frame, const string &text, Point org, Scalar color,
                      double scale = 0.58, int thickness = 2) {
  putText(frame, text, org, FONT_HERSHEY_SIMPLEX, scale, color, thickness,
          LINE_AA);
}

static void drawDashedVerticalLine(Mat &frame, int x, int y0, int y1,
                                   Scalar color) {
  for (int y = y0; y < y1; y += 10) {
    line(frame, Point(x, y), Point(x, min(y + 5, y1)), color, 1, LINE_AA);
  }
}

static string buildGstUdpPipeline(const RuntimeOptions &options) {
  if (options.gst_udp_host.empty() || options.gst_udp_port <= 0)
    return "";

  string encoder = options.gst_use_hw
                       ? "mpph264enc bps=2000000 rc-mode=cbr gop=30"
                       : "x264enc tune=zerolatency speed-preset=ultrafast "
                         "bitrate=2000 key-int-max=30";

  return "appsrc ! queue ! videoconvert ! video/x-raw,format=I420 "
         "! " +
         encoder +
         " ! h264parse ! rtph264pay pt=96 config-interval=1 "
         "! udpsink host=" +
         options.gst_udp_host + " port=" + to_string(options.gst_udp_port) +
         " sync=false async=false";
}

class MjpegServer {
public:
  bool start(int port, int jpeg_quality, double scale, double fps_limit) {
    if (port <= 0)
      return false;
    jpeg_quality_ = max(1, min(100, jpeg_quality));
    scale_ = max(0.1, min(1.0, scale));
    fps_limit_ = max(0.0, fps_limit);
    last_send_ts_ = chrono::steady_clock::now();

    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0)
      return false;

    int yes = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    fcntl(listen_fd_, F_SETFL, fcntl(listen_fd_, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) <
        0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    if (::listen(listen_fd_, 1) < 0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    port_ = port;
    return true;
  }

  void pollAccept() {
    if (listen_fd_ < 0 || client_fd_ >= 0)
      return;

    sockaddr_in client_addr{};
    socklen_t len = sizeof(client_addr);
    int fd =
        ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &len);
    if (fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return;
      return;
    }

    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    client_fd_ = fd;

    static const char header[] =
        "HTTP/1.0 200 OK\r\n"
        "Server: car_cpp\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    if (!sendAll(client_fd_, reinterpret_cast<const uint8_t *>(header),
                 sizeof(header) - 1)) {
      closeClient();
    }
  }

  void sendFrame(const Mat &frame) {
    if (client_fd_ < 0 || frame.empty())
      return;

    auto now = chrono::steady_clock::now();
    if (fps_limit_ > 0.0) {
      double min_interval_ms = 1000.0 / fps_limit_;
      double elapsed_ms =
          chrono::duration_cast<chrono::duration<double, milli>>(now -
                                                                 last_send_ts_)
              .count();
      if (elapsed_ms < min_interval_ms)
        return;
    }

    Mat encoded_frame;
    if (scale_ < 0.999) {
      resize(frame, encoded_frame, Size(), scale_, scale_, cv::INTER_AREA);
    } else {
      encoded_frame = frame;
    }

    vector<uchar> jpeg;
    vector<int> params = {IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!imencode(".jpg", encoded_frame, jpeg, params))
      return;

    string part_header =
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
        to_string(jpeg.size()) + "\r\n\r\n";
    static const char footer[] = "\r\n";

    if (!sendAll(client_fd_,
                 reinterpret_cast<const uint8_t *>(part_header.data()),
                 part_header.size()) ||
        !sendAll(client_fd_, jpeg.data(), jpeg.size()) ||
        !sendAll(client_fd_, reinterpret_cast<const uint8_t *>(footer),
                 sizeof(footer) - 1)) {
      closeClient();
      return;
    }
    last_send_ts_ = now;
  }

  bool isStarted() const { return listen_fd_ >= 0; }
  int port() const { return port_; }

  ~MjpegServer() {
    closeClient();
    if (listen_fd_ >= 0)
      close(listen_fd_);
  }

private:
  bool sendAll(int fd, const uint8_t *data, size_t size) {
    size_t sent = 0;
    while (sent < size) {
      ssize_t n = ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
      if (n > 0) {
        sent += static_cast<size_t>(n);
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        this_thread::sleep_for(chrono::milliseconds(1));
        continue;
      }
      return false;
    }
    return true;
  }

  void closeClient() {
    if (client_fd_ >= 0) {
      close(client_fd_);
      client_fd_ = -1;
    }
  }

  int listen_fd_ = -1;
  int client_fd_ = -1;
  int port_ = 0;
  int jpeg_quality_ = 70;
  double scale_ = 1.0;
  double fps_limit_ = 0.0;
  chrono::steady_clock::time_point last_send_ts_{};
};

struct StreamStats {
  atomic<double> send_ms{0.0};
  atomic<double> fps{0.0};
  atomic<uint64_t> dropped_frames{0};
};

class RemoteStreamWorker {
public:
  bool start(const RuntimeOptions &options) {
    options_ = options;
    if (options_.gst_udp_host.empty() && options_.http_mjpeg_port <= 0)
      return false;
    running_ = true;
    worker_ = thread(&RemoteStreamWorker::run, this);
    return true;
  }

  void submitFrame(const Mat &frame) {
    if (!running_ || frame.empty())
      return;
    lock_guard<mutex> lock(mutex_);
    if (!latest_frame_.empty())
      stats_.dropped_frames.fetch_add(1, memory_order_relaxed);
    frame.copyTo(latest_frame_);
    has_new_frame_ = true;
    cv_.notify_one();
  }

  void stop() {
    if (!running_)
      return;
    {
      lock_guard<mutex> lock(mutex_);
      running_ = false;
      cv_.notify_one();
    }
    if (worker_.joinable())
      worker_.join();
  }

  const StreamStats &stats() const { return stats_; }

  ~RemoteStreamWorker() { stop(); }

private:
  void run() {
    VideoWriter writer;
    MjpegServer mjpeg;
    bool have_output = false;

    if (!options_.gst_udp_host.empty() && options_.gst_udp_port > 0) {
      string pipeline = buildGstUdpPipeline(options_);
      writer.open(pipeline, CAP_GSTREAMER, 0, 30.0, Size(FRAME_W, FRAME_H),
                  true);
      have_output = have_output || writer.isOpened();
    }

    if (options_.http_mjpeg_port > 0) {
      have_output =
          mjpeg.start(options_.http_mjpeg_port, options_.http_mjpeg_quality,
                      options_.http_mjpeg_scale, options_.http_mjpeg_fps) ||
          have_output;
    }

    auto last_sent = chrono::steady_clock::now();
    double fps_ema = 0.0;
    while (true) {
      Mat frame;
      {
        unique_lock<mutex> lock(mutex_);
        cv_.wait_for(lock, chrono::milliseconds(5),
                     [&] { return has_new_frame_ || !running_; });
        if (!running_ && !has_new_frame_)
          break;
        if (has_new_frame_) {
          latest_frame_.copyTo(frame);
          has_new_frame_ = false;
        }
      }

      mjpeg.pollAccept();
      if (!have_output || frame.empty())
        continue;

      auto send_begin = chrono::steady_clock::now();
      if (writer.isOpened())
        writer.write(frame);
      if (mjpeg.isStarted())
        mjpeg.sendFrame(frame);
      auto send_end = chrono::steady_clock::now();

      double send_ms = chrono::duration_cast<chrono::duration<double, milli>>(
                           send_end - send_begin)
                           .count();
      stats_.send_ms.store(send_ms, memory_order_relaxed);

      double dt =
          chrono::duration_cast<chrono::duration<double>>(send_end - last_sent)
              .count();
      if (dt > 1e-6) {
        double inst = 1.0 / dt;
        fps_ema = (fps_ema <= 0.0) ? inst : (0.85 * fps_ema + 0.15 * inst);
        stats_.fps.store(fps_ema, memory_order_relaxed);
      }
      last_sent = send_end;
    }
  }

  RuntimeOptions options_;
  thread worker_;
  mutable mutex mutex_;
  condition_variable cv_;
  Mat latest_frame_;
  bool has_new_frame_ = false;
  bool running_ = false;
  StreamStats stats_;
};

// ============ 输出结构体 ============
struct TrackResult {
  double lateral_error;      // 横向偏差(像素)，正值=线在右侧
  double heading_deg;        // 航向角(度)，正值=线向右弯
  double curvature;          // 曲率(1/像素)，正值=右弯
  int valid_points;          // 有效质心数
  bool line_lost;            // 是否丢线
  bool cross_detected;       // 是否检测到十字
  double cross_score;        // 十字检测强度
  vector<Point2d> centroids; // 各条带质心(ROI坐标系)
  vector<double> weights;    // 各条带权重
  Vec3d poly;                // 多项式系数 [a, b, c]: x = a*y^2 + b*y + c
};

struct TrackingState {
  int lost_streak = 0;
  bool have_valid_track = false;
  TrackResult last_valid_result;
};

struct PerfStats {
  chrono::steady_clock::time_point last_frame_ts = chrono::steady_clock::now();
  double fps = 0.0;
  double loop_ms = 0.0;
  double control_ms = 0.0;
  double capture_ms = 0.0;
  double preprocess_ms = 0.0;
  double track_ms = 0.0;
  double serial_ms = 0.0;
  bool initialized = false;
};


// ============ 矩阵扫描与数学拟合库 ==========
static double evalPolyX(const Vec3d &poly, double y) {
  return poly[0] * y * y + poly[1] * y + poly[2];
}

static double computePointWeight(int strip_idx_from_bottom, int total_strips,
                                 double fill_ratio,
                                 double normalized_distance) {
  // 底部条带(近处)权重高、顶部条带(远处)权重低：近处识别最可靠，远处易受十字/弯道干扰
  double nearness = 1.0 - static_cast<double>(strip_idx_from_bottom) /
                             max(1, total_strips - 1);
  double proximity_weight = 1.0 + 0.9 * nearness;
  double compactness = 0.7 + 0.6 * min(1.0, fill_ratio * 3.0);
  double continuity = 1.5 - 0.7 * min(1.0, normalized_distance);
  return max(0.2, proximity_weight * compactness * continuity);
}

vector<Point2d> scanCentroids(const Mat &binary, int num_strips, int search_w, int max_search_w,
                              vector<double> *point_weights) {
  vector<Point2d> centroids;
  if (point_weights)
    point_weights->clear();

  int strip_h = binary.rows / num_strips;
  double prev_cx = binary.cols / 2.0; // 首条带从图像中心开始搜索
  double prev_dx = 0.0;
  bool have_prev = false;
  const double MAX_DX_CHANGE = search_w * 0.6; // 限制单条带间最大偏移

  for (int i = num_strips - 1; i >= 0; i--) {
    int y_start = i * strip_h;
    int y_end = (i == num_strips - 1) ? binary.rows : (i + 1) * strip_h;
    int h = y_end - y_start;
    double predicted_cx = have_prev ? prev_cx + prev_dx : prev_cx;

    // 计算搜索窗口的 x 范围
    int x_left = max(0, cvRound(predicted_cx - search_w));
    int x_right = min(binary.cols, cvRound(predicted_cx + search_w));
    int w = x_right - x_left;
    if (w <= 0 || h <= 0)
      continue;

    // 截取 2D 搜索区域: 这是最稳固的质量矩阵，完全免疫微小断点和局部厚边
    Rect window(x_left, y_start, w, h);
    Mat strip = binary(window);

    if (countNonZero(strip) < 10 && search_w < max_search_w) {
      int expanded_half = min(max_search_w, search_w + search_w / 2);
      x_left = max(0, cvRound(predicted_cx - expanded_half));
      x_right = min(binary.cols, cvRound(predicted_cx + expanded_half));
      w = x_right - x_left;
      if (w > 0) {
        window = Rect(x_left, y_start, w, h);
        strip = binary(window);
      }
    }

    Moments m = moments(strip, true);
    if (m.m00 < 10) {
      // 第三阶段：以上一次成功质心为锚点的全宽搜索
      double fallback_cx = have_prev ? prev_cx : binary.cols / 2.0;
      x_left = max(0, cvRound(fallback_cx - max_search_w));
      x_right = min(binary.cols, cvRound(fallback_cx + max_search_w));
      w = x_right - x_left;
      window = Rect(x_left, y_start, w, h);
      strip = binary(window);
      m = moments(strip, true);
      if (m.m00 < 10)
        continue;
    }

    double cx = x_left + m.m10 / m.m00;
    double cy = y_start + m.m01 / m.m00;
    double fill_ratio = m.m00 / max(1, w * h);

    // 过滤噪声：填充率过高(>0.5)说明不是线而是大片噪声
    if (fill_ratio > 0.5)
      continue;

    double normalized_distance = min(
        1.0, abs(cx - predicted_cx) / max(1.0, static_cast<double>(search_w)));
    int strip_idx_from_bottom = num_strips - 1 - i;

    centroids.push_back(Point2d(cx, cy));
    if (point_weights) {
      point_weights->push_back(computePointWeight(
          strip_idx_from_bottom, num_strips, fill_ratio, normalized_distance));
    }

    if (have_prev) {
      double raw_dx = cx - prev_cx;
      prev_dx = max(-MAX_DX_CHANGE, min(MAX_DX_CHANGE, raw_dx));
    }
    prev_cx = cx;
    have_prev = true;
  }
  return centroids;
}

// ============ 十字检测：独立全条带填充率扫描 ============
// 与 scanCentroids 解耦：对所有 num_strips 条带无条件计算填充率，不存在间隙
static vector<double> computeAllStripFills(const Mat &binary, int num_strips) {
  vector<double> fills(num_strips, 0.0);
  int strip_h = binary.rows / num_strips;

  for (int i = 0; i < num_strips; ++i) {
    int y_start = i * strip_h;
    int y_end = (i == num_strips - 1) ? binary.rows : (i + 1) * strip_h;
    int h = y_end - y_start;
    if (h <= 0) continue;

    Mat full_strip = binary(Rect(0, y_start, binary.cols, h));
    Mat row_sums;
    cv::reduce(full_strip, row_sums, 1, REDUCE_SUM, CV_32S);

    // top-3 行均值（比单行 max 更稳定）
    vector<int> row_vals;
    row_vals.reserve(row_sums.rows);
    for (int r = 0; r < row_sums.rows; ++r)
      row_vals.push_back(row_sums.at<int>(r, 0));
    sort(row_vals.rbegin(), row_vals.rend());
    int top_k = min(3, (int)row_vals.size());
    double top_sum = 0.0;
    for (int k = 0; k < top_k; ++k)
      top_sum += row_vals[k];
    fills[i] = top_sum / (top_k * 255.0 * binary.cols);
  }
  return fills;
}

// ============ 十字检测：自适应阈值 + 硬结构约束 ============
static pair<bool, double>
detectCrossFromFills(const vector<double> &all_fills, int img_width) {
  int n = (int)all_fills.size();
  if (n < 4)
    return {false, 0.0};

  vector<double> sorted_fills = all_fills;
  sort(sorted_fills.begin(), sorted_fills.end());
  double q1_fill = sorted_fills[n / 4];
  double median_fill = sorted_fills[n / 2];
  double baseline = min(q1_fill, median_fill);

  // wide 门槛 = baseline * 4 倍，且不低于 0.20
  // 弯道浅角度 fill ~15-18% → 被 0.20 硬门槛拦截
  // 真十字横臂 fill 通常 30%+ → 轻松通过
  double wide_thresh = max(0.20, baseline * 4.0);
  wide_thresh = min(wide_thresh, 0.55);

  double narrow_thresh = max(0.06, baseline * 1.5);
  narrow_thresh = min(narrow_thresh, wide_thresh * 0.5);

  // === 扫描连续宽行区域 ===
  int wide_streak = 0;
  int max_streak = 0;
  int zone_start = -1;
  int best_zone_start = -1, best_zone_end = -1;
  double score_sum = 0.0;
  double best_score_sum = 0.0;
  double max_fill_in_wide = 0.0;

  for (int i = 0; i < n; ++i) {
    if (all_fills[i] >= wide_thresh) {
      if (wide_streak == 0) zone_start = i;
      ++wide_streak;
      score_sum += all_fills[i];
      max_fill_in_wide = max(max_fill_in_wide, all_fills[i]);
      if (wide_streak > max_streak) {
        max_streak = wide_streak;
        best_score_sum = score_sum;
        best_zone_start = zone_start;
        best_zone_end = i;
      }
    } else {
      wide_streak = 0;
      score_sum = 0.0;
    }
  }

  // ★ 硬门槛1：至少连续 2 条带为宽
  if (max_streak < 2)
    return {false, 0.0};

  // ★ 硬门槛2：宽区峰值填充率必须 >= 0.20
  if (max_fill_in_wide < 0.20)
    return {false, 0.0};

  // ★ 硬门槛3：对比度 >= 3.0 倍
  double contrast = (baseline > 0.001) ? (max_fill_in_wide / baseline) : 100.0;
  if (contrast < 3.0)
    return {false, 0.0};

  // === 验证宽区两侧窄线（十字结构确认） ===
  int narrow_before = 0, narrow_after = 0;

  for (int i = 0; i < best_zone_start; ++i) {
    if (all_fills[i] < narrow_thresh)
      ++narrow_before;
  }
  for (int i = best_zone_end + 1; i < n; ++i) {
    if (all_fills[i] < narrow_thresh)
      ++narrow_after;
  }

  // ★ 硬门槛4：两侧都必须有窄线条带（核心反弯道策略）
  // 弯道: fill 从近端到远端单调递增 → 只有近端(before)有窄线 → 被拦截
  // 十字: 横臂区域上下都是正常窄线 → 两侧都有窄线 → 通过
  // 例外: 横臂刚进入画面顶部时 after==0，但 before 足够多也允许通过
  bool top_edge_case = (best_zone_start <= 1 && narrow_after == 0 && narrow_before >= 2);
  if (narrow_before == 0 || (narrow_after == 0 && !top_edge_case))
    return {false, 0.0};

  // === 综合评分 ===
  double avg_wide_fill = best_score_sum / max(1, max_streak);
  double contrast_ratio = (baseline > 0.001) ? (avg_wide_fill / baseline) : 50.0;
  double contrast_norm = min(1.0, contrast_ratio / 8.0);
  double streak_norm = min(1.0, max_streak / 3.0);

  double score = streak_norm * 0.30 + contrast_norm * 0.70;
  score = min(1.0, max(0.20, score));

  bool conservative = (max_streak >= 2 && score > 0.45);

  return {conservative, score};
}

// ============ 二次多项式拟合 ============
// 拟合 x = a*y^2 + b*y + c，返回 [a, b, c]
// 用 y 做自变量是因为巡线场景中线接近竖直，避免斜率无穷大
Vec3d fitPoly2Weighted(const vector<Point2d> &pts,
                       const vector<double> &weights) {
  int n = (int)pts.size();
  if (n < 3)
    return Vec3d(0, 0, 0);

  // 构建 Vandermonde 矩阵: [y^2, y, 1]
  Mat A(n, 3, CV_64F);
  Mat B(n, 1, CV_64F);
  for (int i = 0; i < n; i++) {
    double y = pts[i].y;
    double w = i < (int)weights.size() ? max(0.0, weights[i]) : 1.0;
    A.at<double>(i, 0) = w * y * y;
    A.at<double>(i, 1) = w * y;
    A.at<double>(i, 2) = w * 1.0;
    B.at<double>(i, 0) = w * pts[i].x;
  }

  Mat coeffs;
  solve(A, B, coeffs, DECOMP_SVD);
  return Vec3d(coeffs.at<double>(0), coeffs.at<double>(1),
               coeffs.at<double>(2));
}

static void rejectOutliers(const vector<Point2d> &pts,
                           const vector<double> &weights,
                           vector<Point2d> &filtered_pts,
                           vector<double> &filtered_weights) {
  filtered_pts = pts;
  filtered_weights = weights;
  if (pts.size() < 4)
    return;

  Vec3d initial_poly = fitPoly2Weighted(pts, weights);
  filtered_pts.clear();
  filtered_weights.clear();
  for (size_t i = 0; i < pts.size(); ++i) {
    double residual = abs(pts[i].x - evalPolyX(initial_poly, pts[i].y));
    if (residual <= OUTLIER_REJECT_PX) {
      filtered_pts.push_back(pts[i]);
      filtered_weights.push_back(i < weights.size() ? weights[i] : 1.0);
    }
  }

  if (filtered_pts.size() < 3) {
    filtered_pts = pts;
    filtered_weights = weights;
  }
}


TrackResult processFrame(const Mat &roi_binary, const Rect &roi_rect) {
  TrackResult result;
  result.line_lost = true;
  result.lateral_error = 0;
  result.heading_deg = 0;
  result.curvature = 0;
  result.valid_points = 0;
  result.poly = Vec3d(0, 0, 0);
  result.cross_detected = false;
  result.cross_score = 0;

  // 1. 独立扫描全条带填充率 + 质心提取（两者解耦）
  int coord_scale = max(1, roi_rect.width / max(1, roi_binary.cols));
  int adj_search_w = SEARCH_WIDTH / coord_scale;
  int adj_max_search_w = MAX_SEARCH_WIDTH / coord_scale;

  vector<double> all_strip_fills = computeAllStripFills(roi_binary, NUM_STRIPS);

  result.centroids = scanCentroids(roi_binary, NUM_STRIPS, adj_search_w,
                                   adj_max_search_w, &result.weights);

  for (auto &pt : result.centroids) {
    pt.x *= coord_scale;
    pt.y *= coord_scale;
  }

  auto cross_info = detectCrossFromFills(all_strip_fills, roi_binary.cols);
  result.cross_detected = cross_info.first;
  result.cross_score = cross_info.second;
  result.valid_points = (int)result.centroids.size();

  if (result.valid_points < MIN_VALID_POINTS)
    return result;

  // 2. 残差剔除与十字缝合（轨迹盲延）+ 加权二次多项式拟合
  vector<Point2d> pre_pts;
  vector<double> pre_w;
  
  if (result.cross_detected) {
    vector<double> sorted_af = all_strip_fills;
    sort(sorted_af.begin(), sorted_af.end());
    double fill_baseline = sorted_af[sorted_af.size() / 4];
    double flare_thresh = max(0.12, fill_baseline * 3.0);
    double transition_thresh = max(0.06, fill_baseline * 1.8);

    for (size_t k = 0; k < result.centroids.size(); ++k) {
      int strip_idx = min(NUM_STRIPS - 1,
                          (int)(result.centroids[k].y / max(1, roi_rect.height / NUM_STRIPS)));
      double this_fill = all_strip_fills[strip_idx];

      if (this_fill < flare_thresh) {
        pre_pts.push_back(result.centroids[k]);
        double w = result.weights[k];
        if (this_fill >= transition_thresh) {
          double suppress = 1.0 - (this_fill - transition_thresh) /
                                   (flare_thresh - transition_thresh);
          w *= max(0.2, suppress);
        }
        pre_w.push_back(w);
      }
    }
    if (pre_pts.size() < 3 && result.centroids.size() >= 3) {
      pre_pts.assign(result.centroids.begin(), result.centroids.begin() + 3);
      pre_w.assign(result.weights.begin(), result.weights.begin() + 3);
    }
  } else {
    pre_pts = result.centroids;
    pre_w = result.weights;
  }

  vector<Point2d> filtered_pts;
  vector<double> filtered_weights;
  rejectOutliers(pre_pts, pre_w, filtered_pts, filtered_weights);
  result.centroids = filtered_pts;
  result.weights = filtered_weights;
  result.valid_points = (int)result.centroids.size();
  if (result.valid_points < MIN_VALID_POINTS)
    return result;

  result.poly = fitPoly2Weighted(result.centroids, result.weights);
  double a = result.poly[0], b = result.poly[1], c = result.poly[2];

  // 3. 计算控制量 (预瞄点而不是 ROI 最底部)
  double y_lookahead = roi_rect.height * LOOKAHEAD_RATIO;
  double x_lookahead = evalPolyX(result.poly, y_lookahead);
  result.lateral_error = x_lookahead - roi_rect.width / 2.0;

  // 航向角: 融合切线方向与基于Pure Pursuit的水平位移牵引
  double dxdy = 2.0 * a * y_lookahead + b;
  // 注意取反：车前进方向是 -y（向画面上方），沿线前进的切向量是 (-dxdy, -1)
  double tangent_angle = atan2(-dxdy, 1.0) * 180.0 / CV_PI;

  // lateral_error>0 → 线在右侧 → 需要右转(正角度)，方向与切线角一致
  double dx_forward = result.lateral_error;
  double dy_forward = roi_rect.height - y_lookahead;
  double pure_pursuit_angle = atan2(dx_forward, dy_forward) * 180.0 / CV_PI;

  // 自适应融合：偏移越大 → 纠偏(Pure Pursuit)权重越高
  // 偏移<50px时切线主导(平滑跟线)，>150px时纠偏主导(强力拉回)
  double offset_abs = fabs(result.lateral_error);
  double pp_weight = min(0.85, max(0.4, (offset_abs - 50.0) / 150.0 + 0.4));
  double tangent_weight = 1.0 - pp_weight;
  result.heading_deg = tangent_angle * tangent_weight + pure_pursuit_angle * pp_weight;

  // 曲率: d2x/dy2 = 2a
  result.curvature = 2.0 * a;

  result.line_lost = false;
  return result;
}

// ============ 调试可视化 ============
void drawDebug(Mat &frame, const Rect &roi_rect, const TrackResult &result,
               double smooth_lateral, double smooth_heading, double smooth_curv,
               const Mat &binary, bool used_rga, bool track_hold,
               const PerfStats &perf_stats, const StreamStats *stream_stats,
               uint8_t cross_count) {
  // ROI 边框 (直接画在frame上)
  rectangle(frame, roi_rect, Scalar(255, 180, 40), 2, LINE_AA);

  // HUD背景: 只克隆HUD小区域做半透明混合，避免全帧clone (节省2.5MB拷贝)
  Rect hud_rect(12, 12, HUD_PANEL_W, HUD_PANEL_H);
  Mat hud_roi = frame(hud_rect);
  Mat hud_overlay;
  hud_roi.copyTo(hud_overlay);
  rectangle(hud_overlay, Rect(0, 0, HUD_PANEL_W, HUD_PANEL_H), Scalar(18, 18, 18), FILLED);
  addWeighted(hud_overlay, 0.45, hud_roi, 0.55, 0.0, hud_roi);
  rectangle(frame, hud_rect, Scalar(80, 80, 80), 1, LINE_AA);

  // 画图像中心参考线 (白色虚线)
  int cx = frame.cols / 2;
  drawDashedVerticalLine(frame, cx, roi_rect.y, roi_rect.y + roi_rect.height,
                         Scalar(220, 220, 220));

  if (result.line_lost) {
    drawLabel(frame, "TRACK VISION", Point(28, 40), Scalar(100, 200, 255), 0.7, 2);
    cv::line(frame, Point(28, 50), Point(28 + HUD_PANEL_W - 36, 50),
             Scalar(80, 140, 200), 1, LINE_AA);
    drawLabel(frame, "LINE LOST", Point(28, 80), Scalar(0, 80, 255), 0.7, 2);
    drawLabel(frame, format("Cross   #%d", cross_count),
             Point(28, 110), Scalar(0, 200, 255), 0.55, 2);
    return;
  }

  // 画质心点 (双圈光晕)
  for (const auto &pt : result.centroids) {
    Point p(cvRound(pt.x), cvRound(pt.y) + roi_rect.y);
    circle(frame, p, 6, Scalar(0, 180, 255), 1, LINE_AA);
    circle(frame, p, 3, Scalar(0, 245, 255), FILLED, LINE_AA);
  }

  // 画拟合曲线 (绿→青渐变)
  for (int y = 0; y < roi_rect.height - 1; y += 2) {
    double x1 = evalPolyX(result.poly, y);
    double x2 = evalPolyX(result.poly, y + 2);
    Point p1(cvRound(x1), y + roi_rect.y);
    Point p2(cvRound(x2), y + 2 + roi_rect.y);
    if (p1.x >= 0 && p1.x < frame.cols && p2.x >= 0 && p2.x < frame.cols) {
      double t = (double)y / roi_rect.height;
      Scalar color(60 + cvRound(80 * t), 255 - cvRound(60 * t), 120);
      line(frame, p1, p2, color, 2, LINE_AA);
    }
  }

  // === 左侧 HUD 数据面板 ===
  int hud_x = 28, hud_y = 40;
  const double font_s = 0.55;   // 统一字号
  const int th_bold = 2;
  const int th_thin = 1;

  // 标题
  drawLabel(frame, "TRACK VISION", Point(hud_x, hud_y),
            Scalar(100, 200, 255), 0.7, 2);
  hud_y += 10;
  cv::line(frame, Point(hud_x, hud_y),
           Point(hud_x + HUD_PANEL_W - 36, hud_y),
           Scalar(80, 140, 200), 1, LINE_AA);
  hud_y += 22;

  // 核心指标
  drawLabel(frame, format("Offset  %+7.1f px", smooth_lateral),
            Point(hud_x, hud_y), Scalar(230, 230, 230), font_s, th_bold);
  hud_y += 24;

  drawLabel(frame, format("Heading %+7.1f deg", smooth_heading),
            Point(hud_x, hud_y), Scalar(230, 230, 230), font_s, th_bold);
  hud_y += 24;

  drawLabel(frame, format("Curve  %+8.5f", smooth_curv),
            Point(hud_x, hud_y), Scalar(230, 230, 230), font_s, th_bold);
  hud_y += 16;
  cv::line(frame, Point(hud_x, hud_y),
           Point(hud_x + HUD_PANEL_W - 36, hud_y),
           Scalar(50, 50, 50), 1, LINE_AA);
  hud_y += 18;

  // 性能指标
  drawLabel(frame, format("FPS     %7.1f", perf_stats.fps),
            Point(hud_x, hud_y), Scalar(150, 255, 150), font_s, th_thin);
  hud_y += 22;

  drawLabel(frame, format("Latency %7.2f ms", perf_stats.control_ms),
            Point(hud_x, hud_y), Scalar(150, 255, 150), font_s, th_thin);
  hud_y += 16;
  cv::line(frame, Point(hud_x, hud_y),
           Point(hud_x + HUD_PANEL_W - 36, hud_y),
           Scalar(50, 50, 50), 1, LINE_AA);
  hud_y += 20;

  // 行驶方向 (基于航向角而非曲率，避免曲率过小导致直行误报)
  const char *state_text = "STRAIGHT";
  Scalar state_color(150, 255, 150);
  if (fabs(smooth_heading) > 3.0) {
    if (smooth_heading > 0) {
      state_text = ">> RIGHT";
      state_color = Scalar(80, 180, 255);
    } else {
      state_text = "<< LEFT";
      state_color = Scalar(255, 180, 80);
    }
  }
  circle(frame, Point(hud_x + 4, hud_y - 5), 5, state_color, FILLED, LINE_AA);
  drawLabel(frame, state_text, Point(hud_x + 16, hud_y),
            state_color, font_s, th_bold);
  hud_y += 24;

  // 十字计数
  Scalar cross_color = cross_count > 0 ? Scalar(0, 220, 255) : Scalar(140, 140, 140);
  drawLabel(frame, format("Cross   #%d", cross_count),
            Point(hud_x, hud_y), cross_color, font_s, th_bold);

  // === 预瞻点标记 ===
  int lookahead_y = roi_rect.y + cvRound(roi_rect.height * LOOKAHEAD_RATIO);
  int lookahead_x =
      cvRound(evalPolyX(result.poly, roi_rect.height * LOOKAHEAD_RATIO));
  circle(frame, Point(lookahead_x, lookahead_y), 8, Scalar(255, 170, 0), 2,
         LINE_AA);
  circle(frame, Point(lookahead_x, lookahead_y), 3, Scalar(255, 170, 0), FILLED,
         LINE_AA);
  line(frame, Point(cx, lookahead_y), Point(lookahead_x, lookahead_y),
       Scalar(255, 170, 0), 1, LINE_AA);

  // === 右上角状态徽章 ===
  int status_x = frame.cols - 175;
  int status_y = 28;
  if (used_rga) {
    drawLabel(frame, "RGA", Point(status_x, status_y), Scalar(80, 255, 120),
              0.65, 2);
    status_y += 26;
  }
  if (result.cross_detected) {
    drawLabel(frame, format("CROSS %.2f", result.cross_score),
              Point(status_x, status_y), Scalar(0, 190, 255), 0.65, 2);
    status_y += 26;
  }
  if (track_hold) {
    drawLabel(frame, "TRACK HOLD", Point(status_x, status_y),
              Scalar(255, 255, 0), 0.65, 2);
  }

  // === 右下角: 二值化预览 ===
  if (!binary.empty()) {
    Mat binary_bgr;
    cvtColor(binary, binary_bgr, COLOR_GRAY2BGR);
    Mat preview;
    resize(binary_bgr, preview, Size(BINARY_PREVIEW_W, BINARY_PREVIEW_H), 0, 0,
           cv::INTER_NEAREST);
    int preview_x = frame.cols - BINARY_PREVIEW_W - 14;
    int preview_y = frame.rows - BINARY_PREVIEW_H - 14;
    Rect preview_rect(preview_x, preview_y, BINARY_PREVIEW_W, BINARY_PREVIEW_H);
    preview.copyTo(frame(preview_rect));
    rectangle(frame, preview_rect, Scalar(200, 200, 200), 1, LINE_AA);
    drawLabel(frame, "ROI MASK", Point(preview_x + 6, preview_y - 8),
              Scalar(200, 200, 200), 0.45, 1);
  }

  // === 右下角: 推流状态 (在二值化预览上方，不重叠) ===
  if (stream_stats) {
    int metrics_x = frame.cols - BINARY_PREVIEW_W - 14;
    int metrics_y = frame.rows - BINARY_PREVIEW_H - 14 - 70;
    drawLabel(
        frame,
        format("STREAM  %.1f fps", stream_stats->fps.load(memory_order_relaxed)),
        Point(metrics_x, metrics_y), Scalar(180, 180, 180), 0.45, 1);
    drawLabel(frame,
              format("TX      %.1f ms",
                     stream_stats->send_ms.load(memory_order_relaxed)),
              Point(metrics_x, metrics_y + 18), Scalar(180, 180, 180), 0.45, 1);
    drawLabel(frame,
              format("DROP    %llu", static_cast<unsigned long long>(
                                         stream_stats->dropped_frames.load(
                                             memory_order_relaxed))),
              Point(metrics_x, metrics_y + 36), Scalar(180, 180, 180), 0.45, 1);
  }
}

// ============ 串口通信 ============
// 打开串口，配置 115200 8N1，返回 fd（失败返回 -1）
int serialOpen(const char *port) {
  int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;

  struct termios tty;
  tcgetattr(fd, &tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit
  tty.c_cflag &= ~(PARENB | CSTOPB);          // 无校验, 1停止位
  tty.c_cflag |= CLOCAL | CREAD;              // 启用接收
  tty.c_iflag = 0;                            // 无输入处理
  tty.c_oflag = 0;                            // 无输出处理
  tty.c_lflag = 0;                            // 原始模式

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

// 发送 6 字节帧: [0xAA] [mode] [angle_H] [angle_L] [status] [cross_count]
// status: 0=正常, 1=丢线, 2=十字路口
// cross_count: 累计检测到的十字路口个数 (0-255 循环)
// mode: 由蓝牙端设置的运行模式 (原 0x55 位置)
void serialSend(int fd, int16_t heading_cdeg, bool line_lost,
                bool cross_detected, double lateral_px, uint8_t cross_count) {
  if (fd < 0)
    return;

  uint8_t err_h = (uint8_t)((uint16_t)heading_cdeg >> 8);
  uint8_t err_l = (uint8_t)((uint16_t)heading_cdeg & 0xFF);
  uint8_t status = 0;
  if (line_lost)
    status = 1;
  else if (cross_detected)
    status = 2;

  uint8_t mode = g_mode.load();
  uint8_t buf[6] = {0xAA, mode, err_h, err_l, status, cross_count};
  ssize_t n = write(fd, buf, 6);
  (void)n;
  printf("[SERIAL] heading=%+.2f deg  lateral=%+.1f px  status=%d  cross=#%d  mode=%d  "
         "raw=[AA %02X %02X %02X %02X %02X]\n",
         heading_cdeg / 100.0, lateral_px, status, cross_count, mode,
         mode, err_h, err_l, status, cross_count);
}

// ============ main ============
int main(int argc, char **argv) {
  RuntimeOptions options = parseArgs(argc, argv);
  VideoCapture cap;
  Mat static_image;
  bool use_static_image = false;
  int serial_fd = -1;
  bool enable_rga = !options.disable_rga;
  RemoteStreamWorker remote_stream;

#if defined(PLATFORM_LINUX)
  // 模式监听线程：轮询 /tmp/car_mode，由 bt_agent.py 写入
  thread bt_thread([]() {
    uint8_t last = 255;
    while (true) {
      FILE *f = fopen("/tmp/car_mode", "r");
      if (f) {
        int v = fgetc(f);
        fclose(f);
        if (v >= '0' && v <= '9') {
          uint8_t m = v - '0';
          if (m != last) { g_mode.store(m); last = m; printf("[MODE] -> %d\n", m); }
        }
      }
      this_thread::sleep_for(chrono::milliseconds(100));
    }
  });
  bt_thread.detach();
#endif


  if (!options.input_path.empty()) {
    static_image = imread(options.input_path);
    if (!static_image.empty()) {
      use_static_image = true;
      cout << "已加载图片: " << options.input_path << endl;
    } else {
      cap.open(options.input_path);
      if (!cap.isOpened()) {
        cerr << "错误: 无法打开文件 " << options.input_path << endl;
        return -1;
      }
      cout << "已打开视频: " << options.input_path << endl;
    }
  } else {
    bool opened = false;
#if defined(PLATFORM_LINUX)
    if (!options.camera_device.empty()) {
      cout << "正在打开摄像头设备 " << options.camera_device << "..." << endl;
      opened = cap.open(options.camera_device, CAP_V4L2);
      if (!opened) {
        cerr << "警告: V4L2 打开失败，尝试自动后端: " << options.camera_device
             << endl;
        opened = cap.open(options.camera_device, CAP_ANY);
      }
    } else {
      cout << "正在打开摄像头 " << options.camera_index << "..." << endl;
      opened = cap.open(options.camera_index, CAP_V4L2);
      if (!opened) {
        cerr << "警告: V4L2 打开失败，尝试自动后端 camera index "
             << options.camera_index << endl;
        opened = cap.open(options.camera_index, CAP_ANY);
      }
    }
#else
    cout << "正在打开摄像头 " << options.camera_index << "..." << endl;
    opened = cap.open(options.camera_index);
#endif
    if (!opened || !cap.isOpened()) {
      cerr << "错误: 无法打开摄像头！" << endl;
      cerr << "提示: 先检查 /dev/video* 是否存在，或尝试 --camera=1 / "
              "--camera-device=/dev/video11"
           << endl;
      printUsage(argv[0]);
      return -1;
    }
    // 解锁硬件高帧率的两道防线：强制请求高帧率以及压缩流格式（防止USB带宽由于无损YUYV堵塞）
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_W);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_H);
    cap.set(CAP_PROP_FPS, 200); // 主动向 V4L2 底层索要高帧率
  }

  if (!options.serial_port.empty()) {
    serial_fd = serialOpen(options.serial_port.c_str());
    if (serial_fd < 0) {
      cerr << "警告: 无法打开串口 " << options.serial_port << "，将仅可视化运行"
           << endl;
    } else {
      cout << "已打开串口: " << options.serial_port << " (115200 8N1)" << endl;
    }
  }

  cout << "系统已启动";
  if (!options.headless)
    cout << "，按 ESC 退出";
  cout << endl;
#if defined(HAVE_RGA)
  cout << "RGA: " << (enable_rga ? "已启用(可回退)" : "已禁用") << endl;
#else
  cout << "RGA: 当前构建未链接 librga，使用 OpenCV 路径" << endl;
#endif
  cout << "UART payload: heading_deg x 100 (0.01 deg/int16)" << endl;

  if ((options.gst_udp_port > 0 && !options.gst_udp_host.empty()) ||
      options.http_mjpeg_port > 0) {
    if (remote_stream.start(options)) {
      if (!options.gst_udp_host.empty() && options.gst_udp_port > 0) {
        cout << "GStreamer UDP 推流已开启: " << options.gst_udp_host << ":"
             << options.gst_udp_port
             << (options.gst_use_hw ? " (硬编优先)" : " (软编)") << endl;
      }
      if (options.http_mjpeg_port > 0) {
        cout << "HTTP MJPEG 已开启: http://<板子IP>:" << options.http_mjpeg_port
             << "/"
             << " quality=" << options.http_mjpeg_quality
             << " scale=" << options.http_mjpeg_scale;
        if (options.http_mjpeg_fps > 0.0)
          cout << " fps=" << options.http_mjpeg_fps;
        cout << endl;
      }
    } else {
      cerr << "警告: 远程流初始化失败，将跳过 Web 监视/推流" << endl;
    }
  }

  double smooth_lateral = 0, smooth_heading = 0, smooth_curv = 0;
  bool first_frame = true;
  bool prev_frame_was_lost = false;
  TrackingState tracking_state;
  PerfStats perf_stats;
  bool cross_latched = false;
  int cross_latch_countdown = 0;
  uint8_t cross_count = 0;          // 十字路口累计计数 (0-255 循环)
  const int CROSS_LATCH_HOLD = 15;  // 锁存保持帧数：缩短至15帧(~340ms)，避免近距离十字被吞
  // 十字时序滤波：非对称 EMA + 连续帧确认
  double cross_confidence_ema = 0.0;
  int cross_detect_streak = 0;
  const double CROSS_EMA_RISE = 0.40;       // 上升EMA系数（~2帧建立信号，高速下十字仅停留2-3帧）
  const double CROSS_EMA_FALL = 0.55;       // 下降EMA系数（快，无证据时迅速衰减）
  const double CROSS_CONFIRM_THRESH = 0.25; // 确认阈值
  const int CROSS_CONFIRM_FRAMES = 2;       // 连续2帧超过阈值即确认

  Mat frame, resized_frame;
  // 降采样(0.5x)后形态学核：原图 15→7, 5→3，保持等效物理尺寸
  Mat close_kernel = getStructuringElement(MORPH_RECT, Size(7, 7)); // 闭运算: 填补反光空洞
  Mat open_kernel = getStructuringElement(MORPH_RECT, Size(3, 3));  // 开运算: 去除环境散粒噪点

  Mat map1, map2;
  if (ENABLE_UNDISTORT) {
    Mat cameraMatrix = (Mat_<double>(3, 3) << CAM_FX, 0, CAM_CX,
                                              0, CAM_FY, CAM_CY,
                                              0,      0,      1);
    Mat distCoeffs = (Mat_<double>(1, 5) << CAM_K1, CAM_K2, CAM_P1, CAM_P2, CAM_K3);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix,
                            Size(FRAME_W, FRAME_H), CV_16SC2, map1, map2);
  }

  while (true) {
    auto frame_begin = chrono::steady_clock::now();
    if (use_static_image) {
      frame = static_image.clone();
    } else {
      cap >> frame;
      if (frame.empty()) {
        cout << "视频流结束" << endl;
        break;
      }
    }
    auto capture_end = chrono::steady_clock::now();

    bool used_rga = resizeFrameWithAccel(frame, resized_frame, enable_rga);

    Mat work_frame;
    if (ENABLE_UNDISTORT) {
      remap(resized_frame, work_frame, map1, map2, cv::INTER_LINEAR);
    } else {
      work_frame = resized_frame; // 浅拷贝
    }

    int roi_y = cvRound(FRAME_H * ROI_TOP_RATIO);
    int roi_bottom = cvRound(FRAME_H * ROI_BOTTOM_RATIO);
    Rect roi_rect(0, roi_y, FRAME_W, roi_bottom - roi_y);
    Mat roi = work_frame(roi_rect);

    // 性能优化：先降采样BGR再cvtColor，节省3/4色彩转换开销
    Mat roi_small;
    resize(roi, roi_small, Size(), 0.5, 0.5, cv::INTER_LINEAR);
    Mat gray_small;
    cvtColor(roi_small, gray_small, COLOR_BGR2GRAY);

    GaussianBlur(gray_small, gray_small, Size(7, 7), 0);
    Mat binary_small;
    // 块大小 121: 防止十字横臂中心产生空洞
    // 旧值 61 在宽横臂中心看到的全是黑像素→局部均值≈像素值→阈值化失败→空洞
    // 121 能看到横臂两侧的白色背景→正确阈值化→实心填充
    adaptiveThreshold(gray_small, binary_small, 255, ADAPTIVE_THRESH_MEAN_C,
                      THRESH_BINARY_INV, 121, 12);
    
    morphologyEx(binary_small, binary_small, MORPH_CLOSE, close_kernel);
    morphologyEx(binary_small, binary_small, MORPH_OPEN, open_kernel);

    // 性能优化：跳过上采样，直接在半分辨率binary上扫描
    // processFrame内部自动处理坐标缩放
    auto preprocess_end = chrono::steady_clock::now();

    TrackResult result = processFrame(binary_small, roi_rect);
    TrackResult output_result = result;

    if (result.line_lost) {
      tracking_state.lost_streak++;
      if (tracking_state.have_valid_track &&
          tracking_state.lost_streak < LOST_CONFIRM_FRAMES) {
        output_result = tracking_state.last_valid_result;
        output_result.line_lost = false;
        output_result.cross_detected = result.cross_detected;
        output_result.cross_score = result.cross_score;
      }
    } else {
      tracking_state.lost_streak = 0;
      tracking_state.have_valid_track = true;
      tracking_state.last_valid_result = result;
    }

    bool confirmed_lost =
        result.line_lost && tracking_state.lost_streak >= LOST_CONFIRM_FRAMES;
    output_result.line_lost = confirmed_lost;
    // === 十字时序滤波：非对称 EMA + 连续帧确认 ===
    // 上升慢（防误检）、下降快（假信号迅速消退，不占用 latch 窗口）
    double raw_cross_score = result.cross_score;
    if (confirmed_lost) raw_cross_score = 0.0;
    double ema_alpha = (raw_cross_score > cross_confidence_ema)
                       ? CROSS_EMA_RISE   // 上升：慢，0.20
                       : CROSS_EMA_FALL;  // 下降：快，0.50
    cross_confidence_ema = cross_confidence_ema * (1.0 - ema_alpha)
                         + raw_cross_score * ema_alpha;

    // 连续帧确认：平滑后的置信度必须连续 N 帧超过阈值才确认十字
    bool cross_frame_pass = (cross_confidence_ema > CROSS_CONFIRM_THRESH);
    if (cross_frame_pass) {
      ++cross_detect_streak;
    } else {
      cross_detect_streak = 0;
    }

    bool cross_confirmed = cross_frame_pass && (cross_detect_streak >= CROSS_CONFIRM_FRAMES);

    // 更新输出结果以反映时序滤波后的状态（HUD显示和串口均使用此值）
    output_result.cross_detected = cross_confirmed;
    output_result.cross_score = cross_confidence_ema;

    if (cross_confirmed) {
      if (!cross_latched) {
        ++cross_count; // 上升沿：新十字确认，计数+1
      }
      cross_latched = true;
      cross_latch_countdown = CROSS_LATCH_HOLD;
    } else {
      // 信号消失：EMA 已跌破阈值则立即释放 latch，无需等待固定帧数
      // 防止近距离十字在 hold 期间到来时被吞掉
      if (cross_confidence_ema < CROSS_CONFIRM_THRESH) {
        cross_latched = false;
        cross_latch_countdown = 0;
      } else if (cross_latch_countdown > 0) {
        if (--cross_latch_countdown == 0) {
          cross_latched = false;
        }
      }
    }
    auto track_end = chrono::steady_clock::now();

    if (!output_result.line_lost) {
      bool just_recovered = prev_frame_was_lost;
      if (first_frame || just_recovered) {
        smooth_lateral = output_result.lateral_error;
        smooth_heading = output_result.heading_deg;
        smooth_curv = output_result.curvature;
        first_frame = false;
      } else {
        smooth_lateral += EMA_ALPHA * (output_result.lateral_error - smooth_lateral);
        smooth_heading += EMA_ALPHA * (output_result.heading_deg - smooth_heading);
        smooth_curv += EMA_ALPHA * (output_result.curvature - smooth_curv);
      }
    }
    prev_frame_was_lost = output_result.line_lost;

    // 下位机只能闭环 Yaw，因此我们直接发送图像计算出的车道线方向角 (Heading)
    // （注意：如果仅闭环方向角，车体一旦平移偏离中心线将无法自动纠正，除非下位机有其他侧偏补救机制）
    double clamped_heading = max(-SERIAL_HEADING_LIMIT_DEG,
                                 min(SERIAL_HEADING_LIMIT_DEG, smooth_heading));
    int16_t send_err =
        static_cast<int16_t>(llround(clamped_heading * SERIAL_HEADING_SCALE));
    auto serial_begin = chrono::steady_clock::now();
    serialSend(serial_fd, send_err, output_result.line_lost, cross_latched,
               smooth_lateral, cross_count);
    auto serial_end = chrono::steady_clock::now();

    perf_stats.capture_ms =
        chrono::duration_cast<chrono::duration<double, milli>>(capture_end -
                                                               frame_begin)
            .count();
    perf_stats.preprocess_ms =
        chrono::duration_cast<chrono::duration<double, milli>>(preprocess_end -
                                                               capture_end)
            .count();
    perf_stats.track_ms =
        chrono::duration_cast<chrono::duration<double, milli>>(track_end -
                                                               preprocess_end)
            .count();
    perf_stats.serial_ms =
        chrono::duration_cast<chrono::duration<double, milli>>(serial_end -
                                                               serial_begin)
            .count();
    perf_stats.control_ms =
        chrono::duration_cast<chrono::duration<double, milli>>(serial_end -
                                                               frame_begin)
            .count();

    auto frame_end = chrono::steady_clock::now();
    perf_stats.loop_ms = 
        chrono::duration_cast<chrono::duration<double, milli>>(frame_end - 
                                                               frame_begin)
            .count();

    if (!perf_stats.initialized) {
      perf_stats.last_frame_ts = frame_end;
      perf_stats.fps = 0.0;
      perf_stats.initialized = true;
    } else {
      double dt = chrono::duration_cast<chrono::duration<double>>(
                      frame_end - perf_stats.last_frame_ts)
                      .count();
      if (dt > 1e-6) {
        double inst_fps = 1.0 / dt;
        perf_stats.fps = (perf_stats.fps <= 0.0)
                             ? inst_fps
                             : (0.85 * perf_stats.fps + 0.15 * inst_fps);
      }
      perf_stats.last_frame_ts = frame_end;
    }

    bool need_visual_overlay =
        !options.headless || options.http_mjpeg_port > 0 ||
        (options.gst_udp_port > 0 && !options.gst_udp_host.empty());
    if (need_visual_overlay) {
      drawDebug(work_frame, roi_rect, output_result, smooth_lateral,
                smooth_heading, smooth_curv, binary_small, used_rga,
                !output_result.line_lost && tracking_state.lost_streak > 0,
                perf_stats, &remote_stream.stats(), cross_count);
    }

    if (!options.headless) {
      imshow("Line Tracking", work_frame);
      if (waitKey(1) == 27)
        break;
    } else {
      if (use_static_image)
        break;
      // 摄像头采集cap >> frame本身阻塞，无需人为增加过度睡眠带来的延迟累积
      this_thread::sleep_for(chrono::milliseconds(1));
    }

    remote_stream.submitFrame(work_frame);
  }

  if (serial_fd >= 0)
    close(serial_fd);
  remote_stream.stop();
  cap.release();
  destroyAllWindows();
  return 0;
}
