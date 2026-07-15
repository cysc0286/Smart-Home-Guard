#include <algorithm>
#include <array>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <cstdio>
#include <string>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>

#include "include/gpio_alarm_controller.hpp"

#include "include/coco_config.hpp"
#include "include/coco_detector.hpp"
#include "include/debounce_tracker.hpp"
#include "include/uart_control_channel.hpp"
#include "include/utils.hpp"
#include "include/common.hpp"

using namespace std;

bool g_exit_flag = false;
std::mutex g_mtx;

bool check_exit_flag();

struct SnapshotBuffer {
  std::vector<unsigned char> pgm_bytes;
  int width = 0;
  int height = 0;
  bool ready = false;
  std::mutex mutex;
};

struct RawFrameBuffer {
  std::vector<unsigned char> uyvy;
  std::array<int, 2> crop_shape = {0, 0};
  bool pending = false;
  std::mutex mutex;
  std::condition_variable cv;
};

enum class EnvPolicy {
  kNormal,
  kLowLight,
  kBright
};

struct EnvPolicyState {
  EnvPolicy policy = EnvPolicy::kNormal;
  int avg_luma = 0;
  int dark_votes = 0;
  int bright_votes = 0;
  float conf_threshold = coco_config::kConfThreshold;
  int alarm_hold_ms = coco_config::kAlarmHoldMs;
};

enum class RoiLoadState {
  kPaused,
  kEvery10,
  kEvery5,
  kEvery2
};

struct RoiLoadController {
  RoiLoadState state = RoiLoadState::kEvery5;
  int overload_votes = 0;
  int recovery_votes = 0;
  int warmup_windows = 0;
  int priority_holdoff_windows = 0;
};

struct RuntimeTestControl {
  bool load_enabled = false;
  bool camera_fail_enabled = false;
};

struct AcceptanceStats {
  int frames = 0;
  int detection_frames = 0;
  int alarm_frames = 0;
  int detections = 0;
  int alarm_detections = 0;
  int camera_recoveries = 0;
  int data_recoveries = 0;
  int camera_recovery_cycles = 0;
  int camera_init_attempts = 0;
  int camera_init_failures = 0;
  int camera_validation_failures = 0;
  int infer_failures = 0;
  int resource_warnings = 0;
  int roi_runs = 0;
  int roi_failures = 0;
  int roi_skipped = 0;
  int roi_cache_updates = 0;
  int roi_cache_hits = 0;
  int roi_cache_drops = 0;
  int roi_deduplicated = 0;
  int roi_load_transitions = 0;
  int roi_load_paused_frames = 0;
  int perf_samples = 0;
  int roi_perf_samples = 0;
  int base_deadline_misses = 0;
  int total_deadline_misses = 0;
  long long capture_us_total = 0;
  long long full_infer_us_total = 0;
  long long roi_work_us_total = 0;
  long long osd_us_total = 0;
  long long base_loop_us_total = 0;
  long long total_loop_us_total = 0;
  bool summary_printed = false;
  std::vector<long long> latency_ms;
};

struct PerfWindow {
  std::vector<long long> capture_us;
  std::vector<long long> full_infer_us;
  std::vector<long long> roi_work_us;
  std::vector<long long> osd_us;
  std::vector<long long> base_loop_us;
  std::vector<long long> total_loop_us;
  int base_deadline_misses = 0;
  int total_deadline_misses = 0;

  void Reserve(size_t count) {
    capture_us.reserve(count);
    full_infer_us.reserve(count);
    roi_work_us.reserve(count);
    osd_us.reserve(count);
    base_loop_us.reserve(count);
    total_loop_us.reserve(count);
  }

  void Clear() {
    capture_us.clear();
    full_infer_us.clear();
    roi_work_us.clear();
    osd_us.clear();
    base_loop_us.clear();
    total_loop_us.clear();
    base_deadline_misses = 0;
    total_deadline_misses = 0;
  }
};

const char* BoolText(bool value) {
  return value ? "ON" : "OFF";
}

const char* EnvPolicyName(EnvPolicy policy) {
  switch (policy) {
    case EnvPolicy::kLowLight: return "LOW_LIGHT";
    case EnvPolicy::kBright: return "BRIGHT";
    case EnvPolicy::kNormal:
    default: return "NORMAL";
  }
}

const char* RoiLoadStateName(RoiLoadState state) {
  switch (state) {
    case RoiLoadState::kPaused: return "PAUSED";
    case RoiLoadState::kEvery10: return "EVERY10";
    case RoiLoadState::kEvery2: return "EVERY2";
    case RoiLoadState::kEvery5:
    default: return "EVERY5";
  }
}

int RoiLoadIntervalFrames(RoiLoadState state) {
  switch (state) {
    case RoiLoadState::kPaused: return 0;
    case RoiLoadState::kEvery10:
      return coco_config::kRoiRecoveryIntervalFrames;
    case RoiLoadState::kEvery2:
      return coco_config::kRoiPriorityIntervalFrames;
    case RoiLoadState::kEvery5:
    default:
      return coco_config::kRoiIntervalFrames;
  }
}

bool FileReadable(const std::string& path) {
  return access(path.c_str(), R_OK) == 0;
}

struct SensorTensorInfo {
  void* data = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint8_t format = 0;
  size_t memory_size = 0;
};

bool InspectSensorTensor(const ssne_tensor_t& tensor,
                         const std::array<int, 2>& expected_shape,
                         SensorTensorInfo* info,
                         std::string* reason) {
  SensorTensorInfo current;
  current.data = get_data(tensor);
  current.width = get_width(tensor);
  current.height = get_height(tensor);
  current.format = get_data_format(tensor);
  current.memory_size = get_mem_size(tensor);
  const size_t expected_bytes = static_cast<size_t>(expected_shape[0]) *
                                static_cast<size_t>(expected_shape[1]) * 2u;

  if (info) *info = current;
  if (current.data == nullptr) {
    if (reason) *reason = "null data";
    return false;
  }
  if (current.width != static_cast<uint32_t>(expected_shape[0]) ||
      current.height != static_cast<uint32_t>(expected_shape[1])) {
    if (reason) {
      *reason = "shape=" + std::to_string(current.width) + "x" +
                std::to_string(current.height) + " expected=" +
                std::to_string(expected_shape[0]) + "x" +
                std::to_string(expected_shape[1]);
    }
    return false;
  }
  if (current.format != SSNE_YUV422_16) {
    if (reason) {
      *reason = "format=" + std::to_string(static_cast<int>(current.format)) +
                " expected=" + std::to_string(static_cast<int>(SSNE_YUV422_16));
    }
    return false;
  }
  if (current.memory_size < expected_bytes) {
    if (reason) {
      *reason = "bytes=" + std::to_string(current.memory_size) +
                " expected_at_least=" + std::to_string(expected_bytes);
    }
    return false;
  }
  if (reason) reason->clear();
  return true;
}

int EstimateFpsScore(float ratio) {
  const float clipped = std::max(0.0f, std::min(ratio, 1.0f));
  return static_cast<int>(std::floor(10.0f * clipped));
}

float ToFramePeriods(long long latency_ms, float sensor_fps) {
  const float frame_period_ms = 1000.0f / sensor_fps;
  return static_cast<float>(latency_ms) / frame_period_ms;
}

int EstimateLatencyScore(long long p95_ms, float sensor_fps) {
  const float periods = ToFramePeriods(p95_ms, sensor_fps);
  if (periods > 11.0f) return 0;
  return std::max(0, static_cast<int>(std::floor(11.0f - periods)));
}

long long PercentileMs(std::vector<long long> samples, int percentile) {
  if (samples.empty()) return 0;
  std::sort(samples.begin(), samples.end());
  const size_t idx = std::min(
      samples.size() - 1,
      static_cast<size_t>(samples.size() * static_cast<size_t>(percentile) / 100u));
  return samples[idx];
}

double PercentileUsAsMs(std::vector<long long> samples, int percentile) {
  if (samples.empty()) return 0.0;
  std::sort(samples.begin(), samples.end());
  const size_t idx = std::min(
      samples.size() - 1,
      static_cast<size_t>(samples.size() * static_cast<size_t>(percentile) / 100u));
  return static_cast<double>(samples[idx]) / 1000.0;
}

double AverageUsAsMs(long long total_us, int samples) {
  if (samples <= 0) return 0.0;
  return static_cast<double>(total_us) / (1000.0 * static_cast<double>(samples));
}

int SampleAverageLuma(const void* data_ptr, const std::array<int, 2>& crop_shape) {
  if (data_ptr == nullptr) return 0;
  const unsigned char* src = static_cast<const unsigned char*>(data_ptr);
  const int sample_count = 256;
  const int total_pixels = crop_shape[0] * crop_shape[1];
  long sum = 0;
  for (int i = 0; i < sample_count; ++i) {
    const int p = (i * total_pixels) / sample_count;
    sum += src[p * 2 + 1];
  }
  return static_cast<int>(sum / sample_count);
}

bool UpdateEnvPolicy(int avg_luma, EnvPolicyState* state) {
  const EnvPolicy previous = state->policy;
  state->avg_luma = avg_luma;

  if (avg_luma < coco_config::kEnvLowLightY) {
    state->dark_votes++;
    state->bright_votes = 0;
  } else if (avg_luma > coco_config::kEnvBrightY) {
    state->bright_votes++;
    state->dark_votes = 0;
  } else {
    state->dark_votes = 0;
    state->bright_votes = 0;
    state->policy = EnvPolicy::kNormal;
  }

  if (state->dark_votes >= coco_config::kEnvPolicyStableSamples) {
    state->policy = EnvPolicy::kLowLight;
  } else if (state->bright_votes >= coco_config::kEnvPolicyStableSamples) {
    state->policy = EnvPolicy::kBright;
  }

  if (state->policy == EnvPolicy::kLowLight) {
    state->conf_threshold = coco_config::kLowLightConfThreshold;
    state->alarm_hold_ms = coco_config::kLowLightAlarmHoldMs;
  } else if (state->policy == EnvPolicy::kBright) {
    state->conf_threshold = coco_config::kBrightConfThreshold;
    state->alarm_hold_ms = coco_config::kAlarmHoldMs;
  } else {
    state->conf_threshold = coco_config::kConfThreshold;
    state->alarm_hold_ms = coco_config::kAlarmHoldMs;
  }

  return previous != state->policy;
}

struct ZonePoint {
  int x;
  int y;

  ZonePoint() : x(0), y(0) {}
  ZonePoint(int point_x, int point_y) : x(point_x), y(point_y) {}
};

struct GuardZone {
  std::string shape = "none";
  std::vector<ZonePoint> points;
  std::vector<int> alarm_class_ids;
  bool active = false;

  GuardZone() : shape("none"), points(), alarm_class_ids(DefaultAlarmClassIds()), active(false) {}

  void SetRect(int x1, int y1, int x2, int y2) {
    shape = "rect";
    active = true;
    if (x1 > x2) std::swap(x1, x2);
    if (y1 > y2) std::swap(y1, y2);
    points.clear();
    points.push_back(ZonePoint(x1, y1));
    points.push_back(ZonePoint(x2, y2));
  }

  void SetPolygon(const std::vector<ZonePoint>& polygon_points) {
    shape = "polygon";
    active = polygon_points.size() >= 3;
    points = polygon_points;
  }

  int X1() const { return points.empty() ? 0 : points[0].x; }
  int Y1() const { return points.empty() ? 0 : points[0].y; }
  int X2() const { return points.size() < 2 ? 0 : points[1].x; }
  int Y2() const { return points.size() < 2 ? 0 : points[1].y; }

  std::string Describe() const {
    if (!active) {
      return "none";
    }
    if (shape == "polygon") {
      return "polygon points=" + std::to_string(points.size());
    }
    return "rect (" + std::to_string(X1()) + "," + std::to_string(Y1()) + ")-(" +
           std::to_string(X2()) + "," + std::to_string(Y2()) + ")";
  }

  static std::vector<int> DefaultAlarmClassIds() {
    std::vector<int> ids;
    ids.push_back(0);   // person
    ids.push_back(15);  // cat
    ids.push_back(16);  // dog
    return ids;
  }
};

bool SameGuardZone(const GuardZone& lhs, const GuardZone& rhs) {
  if (lhs.shape != rhs.shape || lhs.active != rhs.active ||
      lhs.alarm_class_ids != rhs.alarm_class_ids ||
      lhs.points.size() != rhs.points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.points.size(); ++i) {
    if (lhs.points[i].x != rhs.points[i].x ||
        lhs.points[i].y != rhs.points[i].y) {
      return false;
    }
  }
  return true;
}

enum class ArmMode {
  kHome,
  kAway,
  kSleep,
  kConfig,
};

const char* ArmModeName(ArmMode mode) {
  switch (mode) {
    case ArmMode::kAway: return "AWAY";
    case ArmMode::kSleep: return "SLEEP";
    case ArmMode::kConfig: return "CONFIG";
    case ArmMode::kHome:
    default: return "HOME";
  }
}

bool ParseArmMode(const std::string& text, ArmMode* mode) {
  if (mode == nullptr) return false;
  if (text == "HOME") {
    *mode = ArmMode::kHome;
    return true;
  }
  if (text == "AWAY") {
    *mode = ArmMode::kAway;
    return true;
  }
  if (text == "SLEEP") {
    *mode = ArmMode::kSleep;
    return true;
  }
  if (text == "CONFIG" || text == "OFF") {
    *mode = ArmMode::kConfig;
    return true;
  }
  return false;
}

bool ModeAllowsAlarmClass(int class_id, ArmMode mode) {
  // HOME protects family members and pets. AWAY/SLEEP focus on human entry.
  // CONFIG keeps detection/OSD running while all alarm outputs are disarmed.
  if (mode == ArmMode::kConfig) return false;
  if (mode == ArmMode::kHome) return true;
  return class_id == 0;
}

GpioAlarmMode GpioModeForArmMode(ArmMode mode) {
  switch (mode) {
    case ArmMode::kAway: return GpioAlarmMode::kAway;
    case ArmMode::kSleep: return GpioAlarmMode::kSleep;
    case ArmMode::kConfig:
    case ArmMode::kHome:
    default: return GpioAlarmMode::kHome;
  }
}

struct AlarmLifecycle {
  bool active = false;
  long long last_raw_active_ms = 0;
  long long started_ms = 0;
  int starts = 0;
  int ends = 0;
  CocoDetection last_detection;
  bool has_detection = false;

  bool Update(bool raw_active, long long now_ms, const CocoDetection* best_detection) {
    if (raw_active) {
      last_raw_active_ms = now_ms;
      if (best_detection != nullptr) {
        last_detection = *best_detection;
        has_detection = true;
      }
      if (!active) {
        active = true;
        started_ms = now_ms;
        ++starts;
        return true;
      }
      return false;
    }

    if (active && last_raw_active_ms > 0 &&
        now_ms - last_raw_active_ms > coco_config::kAlarmEventClearMs) {
      active = false;
      ++ends;
      return true;
    }
    return false;
  }

  void ResetActiveState() {
    active = false;
    last_raw_active_ms = 0;
    started_ms = 0;
    has_detection = false;
  }
};

const char* StatusBitmapName(ArmMode mode, bool zone_active,
                             bool alarm_active, bool degraded) {
  if (alarm_active) return coco_config::kStatusAlarmBitmapName;
  if (degraded) return coco_config::kStatusDegradedBitmapName;
  if (!zone_active) return coco_config::kStatusNoZoneBitmapName;
  switch (mode) {
    case ArmMode::kAway: return coco_config::kStatusAwayBitmapName;
    case ArmMode::kSleep: return coco_config::kStatusSleepBitmapName;
    case ArmMode::kConfig: return coco_config::kStatusConfigBitmapName;
    case ArmMode::kHome:
    default: return coco_config::kStatusHomeBitmapName;
  }
}

std::string WallClockText() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  char buffer[32] = {0};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_time);
  return std::string(buffer);
}

void AppendAlarmEvent(const char* event_name,
                      const CocoDetection* best_detection,
                      long long duration_ms) {
  const char* class_name =
      best_detection == nullptr ? "none" : best_detection->label.c_str();
  const float score = best_detection == nullptr ? 0.0f : best_detection->score;

  if (access(coco_config::kAlarmEventLogPath, F_OK) == 0) {
    const long file_size =
        static_cast<long>(std::ifstream(coco_config::kAlarmEventLogPath,
                                        std::ios::binary | std::ios::ate).tellg());
    if (file_size > coco_config::kAlarmEventMaxBytes) {
      std::ofstream reset(coco_config::kAlarmEventLogPath,
                          std::ios::binary | std::ios::trunc);
      reset << "# HALO alarm events\n";
    }
  }

  std::ofstream output(coco_config::kAlarmEventLogPath,
                       std::ios::out | std::ios::app);
  if (!output.is_open()) {
    fprintf(stderr, "[EVENT][WARN] Cannot open event log: %s\n",
            coco_config::kAlarmEventLogPath);
    return;
  }
  output << WallClockText() << " event=" << event_name
         << " class=" << class_name
         << " score=" << score
         << " duration_ms=" << duration_ms << "\n";
  output.close();
  printf("[EVENT] %s class=%s score=%.2f duration_ms=%lld\n",
         event_name, class_name, score, duration_ms);
}

void ApplyArmMode(ArmMode next_mode, ArmMode* arm_mode,
                  DebounceTracker* tracker, AlarmLifecycle* alarm_lifecycle,
                  GpioAlarmController* gpio_alarm, bool gpio_ready) {
  if (arm_mode == nullptr || tracker == nullptr || alarm_lifecycle == nullptr ||
      *arm_mode == next_mode) {
    return;
  }

  const long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now().time_since_epoch()).count();
  if (alarm_lifecycle->active) {
    const long long duration_ms = std::max(0LL, now_ms - alarm_lifecycle->started_ms);
    AppendAlarmEvent("END_MODE_CHANGE",
                     alarm_lifecycle->has_detection
                         ? &alarm_lifecycle->last_detection : nullptr,
                     duration_ms);
    ++alarm_lifecycle->ends;
  }

  tracker->Reset();
  alarm_lifecycle->ResetActiveState();
  if (gpio_ready && gpio_alarm != nullptr) gpio_alarm->Reset();
  *arm_mode = next_mode;
  printf("[MODE] %s policy_reset=1 confirm_ms=%d\n",
         ArmModeName(*arm_mode), coco_config::kAlarmConfirmMs);
}

class SnapshotHttpServer {
 public:
  SnapshotHttpServer(int port, SnapshotBuffer* snapshot)
      : port_(port), snapshot_(snapshot) {}

  void Run() {
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
      perror("[SNAPSHOT] socket");
      return;
    }

    int reuse = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      perror("[SNAPSHOT] bind");
      close(listen_fd);
      return;
    }

    if (listen(listen_fd, 4) < 0) {
      perror("[SNAPSHOT] listen");
      close(listen_fd);
      return;
    }

    printf("[SNAPSHOT] HTTP server listening on port %d\n", port_);

    while (!check_exit_flag()) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(listen_fd, &readfds);

      timeval timeout;
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;

      const int ready = select(listen_fd + 1, &readfds, nullptr, nullptr, &timeout);
      if (ready <= 0) {
        continue;
      }

      sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(listen_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
      if (client_fd < 0) {
        continue;
      }

      HandleClient(client_fd);
      close(client_fd);
    }

    close(listen_fd);
  }

 private:
  bool SendAll(int client_fd, const char* data, size_t len) const {
    size_t total = 0;
    while (total < len) {
      const int sent = send(client_fd, data + total, static_cast<int>(len - total), 0);
      if (sent <= 0) {
        return false;
      }
      total += static_cast<size_t>(sent);
    }
    return true;
  }

  void HandleClient(int client_fd) {
    char request[1024];
    const int received = recv(client_fd, request, sizeof(request) - 1, 0);
    if (received <= 0) {
      return;
    }
    request[received] = '\0';

    std::string path = ParsePath(request);
    if (path != coco_config::kSnapshotRoute && path != coco_config::kSnapshotAltRoute &&
        path != "/") {
      SendTextResponse(client_fd, "404 Not Found", "Not Found\n");
      return;
    }

    std::vector<unsigned char> snapshot_data;
    {
      std::lock_guard<std::mutex> lock(snapshot_->mutex);
      if (!snapshot_->ready || snapshot_->pgm_bytes.empty()) {
        SendTextResponse(client_fd, "503 Service Unavailable", "Snapshot not ready\n");
        return;
      }
      snapshot_data = snapshot_->pgm_bytes;
    }

    std::string header = "HTTP/1.1 200 OK\r\n";
    header += "Content-Type: image/x-portable-graymap\r\n";
    header += "Content-Length: " + std::to_string(snapshot_data.size()) + "\r\n";
    header += "Connection: close\r\n\r\n";

    if (!SendAll(client_fd, header.c_str(), header.size())) {
      return;
    }
    SendAll(client_fd,
            reinterpret_cast<const char*>(snapshot_data.data()),
            snapshot_data.size());
  }

  std::string ParsePath(const char* request) const {
    const char* line_end = std::strstr(request, "\r\n");
    std::string line = line_end == nullptr ? std::string(request) : std::string(request, line_end);
    const std::string prefix = "GET ";
    if (line.compare(0, prefix.size(), prefix) != 0) {
      return "";
    }

    const std::size_t path_start = prefix.size();
    const std::size_t path_end = line.find(' ', path_start);
    if (path_end == std::string::npos) {
      return "";
    }
    return line.substr(path_start, path_end - path_start);
  }

  void SendTextResponse(int client_fd, const char* status, const char* body) const {
    std::string payload = "HTTP/1.1 ";
    payload += status;
    payload += "\r\nContent-Type: text/plain\r\nContent-Length: ";
    payload += std::to_string(std::strlen(body));
    payload += "\r\nConnection: close\r\n\r\n";
    payload += body;
    SendAll(client_fd, payload.c_str(), payload.size());
  }

  int port_;
  SnapshotBuffer* snapshot_;
};

void keyboard_listener() {
  std::string input;
  std::cout << "Input q to quit..." << std::endl;
  while (true) {
    std::cin >> input;
    std::lock_guard<std::mutex> lock(g_mtx);
    if (input == "q" || input == "Q") {
      g_exit_flag = true;
      break;
    }
  }
}

bool check_exit_flag() {
  std::lock_guard<std::mutex> lock(g_mtx);
  return g_exit_flag;
}

// 裁剪坐标系(1440x1080) -> 原图坐标系(1920x1080)
void ConvertCropBoxesToOriginal(CocoDetectionResult* result) {
  for (auto& det : result->detections) {
    det.box_xyxy[0] += static_cast<float>(coco_config::kCropOffsetX);
    det.box_xyxy[2] += static_cast<float>(coco_config::kCropOffsetX);
  }
}

void ConvertCropBoxesToOriginal(std::vector<std::array<float, 4>>* boxes) {
  for (auto& box : *boxes) {
    box[0] += static_cast<float>(coco_config::kCropOffsetX);
    box[2] += static_cast<float>(coco_config::kCropOffsetX);
  }
}

struct ZoneRoiWindow {
  int x = 0;
  int y = 0;
  int side = 0;
  double area_ratio = 0.0;
  bool enabled = false;
};

struct RoiResultMetadata {
  std::uint64_t frame_id = 0;
  std::uint64_t pipeline_generation = 0;
  ZoneRoiWindow window;
  std::chrono::steady_clock::time_point timestamp =
      std::chrono::steady_clock::time_point();
  bool valid = false;
};

double PolygonAreaRatio(const GuardZone& zone, const std::array<int, 2>& crop_shape) {
  if (crop_shape[0] <= 0 || crop_shape[1] <= 0) {
    return 0.0;
  }
  if (zone.shape == "rect" && zone.points.size() >= 2) {
    const double width = std::abs(zone.points[1].x - zone.points[0].x);
    const double height = std::abs(zone.points[1].y - zone.points[0].y);
    return (width * height) /
           (static_cast<double>(crop_shape[0]) * crop_shape[1]);
  }
  if (zone.points.size() < 3) return 0.0;
  double area_twice = 0.0;
  for (std::size_t i = 0; i < zone.points.size(); ++i) {
    const ZonePoint& a = zone.points[i];
    const ZonePoint& b = zone.points[(i + 1) % zone.points.size()];
    area_twice += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
  }
  const double area = std::abs(area_twice) * 0.5;
  return area / (static_cast<double>(crop_shape[0]) * crop_shape[1]);
}

bool BuildZoneRoi(const GuardZone& zone,
                  const std::array<int, 2>& crop_shape,
                  ZoneRoiWindow* roi) {
  if (roi == nullptr) return false;
  *roi = ZoneRoiWindow();
  if (!zone.active || crop_shape[0] <= 0 || crop_shape[1] <= 0) return false;
  for (const ZonePoint& point : zone.points) {
    if (point.x < 0 || point.y < 0 ||
        point.x >= crop_shape[0] || point.y >= crop_shape[1]) {
      return false;
    }
  }

  const double area_ratio = PolygonAreaRatio(zone, crop_shape);
  roi->area_ratio = area_ratio;
  // A very large zone has little scale benefit and would add unnecessary load.
  if (area_ratio > 0.80) return false;

  int xmin = crop_shape[0] - 1;
  int ymin = crop_shape[1] - 1;
  int xmax = 0;
  int ymax = 0;
  if (zone.shape == "rect" && zone.points.size() >= 2) {
    xmin = std::min(zone.points[0].x, zone.points[1].x);
    xmax = std::max(zone.points[0].x, zone.points[1].x);
    ymin = std::min(zone.points[0].y, zone.points[1].y);
    ymax = std::max(zone.points[0].y, zone.points[1].y);
  } else if (zone.points.size() >= 3) {
    for (const ZonePoint& point : zone.points) {
      xmin = std::min(xmin, point.x);
      ymin = std::min(ymin, point.y);
      xmax = std::max(xmax, point.x);
      ymax = std::max(ymax, point.y);
    }
  } else {
    return false;
  }

  xmin = std::max(0, std::min(xmin, crop_shape[0] - 1));
  xmax = std::max(0, std::min(xmax, crop_shape[0] - 1));
  ymin = std::max(0, std::min(ymin, crop_shape[1] - 1));
  ymax = std::max(0, std::min(ymax, crop_shape[1] - 1));
  if (xmax <= xmin || ymax <= ymin) return false;

  const int box_width = xmax - xmin + 1;
  const int box_height = ymax - ymin + 1;
  const int box_max = std::max(box_width, box_height);
  const int margin = std::max(24, std::min(96, static_cast<int>(std::lround(box_max * 0.12))));
  const int alignment = coco_config::kRoiAlignment;
  int side = std::max(256, box_max + margin * 2);
  side = ((side + alignment - 1) / alignment) * alignment;
  const int max_side = std::min(crop_shape[0], crop_shape[1]);
  const int max_aligned_side = (max_side / alignment) * alignment;
  if (side > max_aligned_side) side = max_aligned_side;
  if (side < box_max) return false;

  const int center_x = (xmin + xmax) / 2;
  const int center_y = (ymin + ymax) / 2;
  int x = center_x - side / 2;
  int y = center_y - side / 2;
  x = std::max(0, std::min(x, crop_shape[0] - side));
  y = std::max(0, std::min(y, crop_shape[1] - side));
  // Packed YUV422 starts on a two-pixel chroma pair. The ROI dimensions are
  // additionally aligned for the board's offline AI preprocess pipeline.
  x -= x % 2;

  roi->x = x;
  roi->y = y;
  roi->side = side;
  roi->enabled = true;
  return true;
}

bool BuildZoneRoiTensor(const ssne_tensor_t& source,
                        const std::array<int, 2>& crop_shape,
                        const ZoneRoiWindow& roi,
                        ssne_tensor_t* output) {
  if (output == nullptr || !roi.enabled || roi.side <= 0 ||
      roi.side % coco_config::kRoiAlignment != 0 || roi.x % 2 != 0) {
    return false;
  }
  output->data = nullptr;
  if (get_data(source) == nullptr ||
      get_data_format(source) != SSNE_YUV422_16 ||
      static_cast<int>(get_width(source)) != crop_shape[0] ||
      static_cast<int>(get_height(source)) != crop_shape[1]) {
    return false;
  }

  ssne_tensor_t roi_tensor = create_tensor(
      static_cast<uint32_t>(roi.side), static_cast<uint32_t>(roi.side),
      SSNE_YUV422_16, SSNE_BUF_AI);
  const unsigned char* src = static_cast<const unsigned char*>(get_data(source));
  if (roi_tensor.data == nullptr || src == nullptr) {
    if (roi_tensor.data != nullptr) release_tensor(roi_tensor);
    return false;
  }

  const std::size_t row_bytes = static_cast<std::size_t>(roi.side) * 2u;
  const std::size_t source_stride = static_cast<std::size_t>(crop_shape[0]) * 2u;
  const std::size_t roi_bytes = row_bytes * static_cast<std::size_t>(roi.side);
  std::vector<unsigned char> packed_roi(roi_bytes);
  for (int row = 0; row < roi.side; ++row) {
    const unsigned char* src_row = src +
        static_cast<std::size_t>(roi.y + row) * source_stride +
        static_cast<std::size_t>(roi.x) * 2u;
    std::memcpy(packed_roi.data() + static_cast<std::size_t>(row) * row_bytes,
                src_row, row_bytes);
  }
  if (get_mem_size(roi_tensor) < roi_bytes ||
      load_tensor_buffer_ptr(roi_tensor, packed_roi.data(),
                             static_cast<int>(roi_bytes)) != 0) {
    release_tensor(roi_tensor);
    return false;
  }
  *output = roi_tensor;
  return true;
}

float DetectionIoU(const CocoDetection& a, const CocoDetection& b) {
  const float x1 = std::max(a.box_xyxy[0], b.box_xyxy[0]);
  const float y1 = std::max(a.box_xyxy[1], b.box_xyxy[1]);
  const float x2 = std::min(a.box_xyxy[2], b.box_xyxy[2]);
  const float y2 = std::min(a.box_xyxy[3], b.box_xyxy[3]);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float area_a = std::max(0.0f, a.box_xyxy[2] - a.box_xyxy[0]) *
                       std::max(0.0f, a.box_xyxy[3] - a.box_xyxy[1]);
  const float area_b = std::max(0.0f, b.box_xyxy[2] - b.box_xyxy[0]) *
                       std::max(0.0f, b.box_xyxy[3] - b.box_xyxy[1]);
  return inter / std::max(1e-6f, area_a + area_b - inter);
}

float DetectionOverlapOverSmaller(const CocoDetection& a,
                                  const CocoDetection& b) {
  const float x1 = std::max(a.box_xyxy[0], b.box_xyxy[0]);
  const float y1 = std::max(a.box_xyxy[1], b.box_xyxy[1]);
  const float x2 = std::min(a.box_xyxy[2], b.box_xyxy[2]);
  const float y2 = std::min(a.box_xyxy[3], b.box_xyxy[3]);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float area_a = std::max(0.0f, a.box_xyxy[2] - a.box_xyxy[0]) *
                       std::max(0.0f, a.box_xyxy[3] - a.box_xyxy[1]);
  const float area_b = std::max(0.0f, b.box_xyxy[2] - b.box_xyxy[0]) *
                       std::max(0.0f, b.box_xyxy[3] - b.box_xyxy[1]);
  return inter / std::max(1e-6f, std::min(area_a, area_b));
}

bool SameRoiWindow(const ZoneRoiWindow& a, const ZoneRoiWindow& b) {
  return a.enabled && b.enabled &&
         a.x == b.x && a.y == b.y && a.side == b.side;
}

int MergeCropDetections(CocoDetectionResult* full_result,
                        const CocoDetectionResult& additions,
                        bool prefer_higher_score) {
  if (full_result == nullptr) return 0;
  int deduplicated = 0;
  for (const CocoDetection& det : additions.detections) {
    bool duplicate = false;
    for (CocoDetection& existing : full_result->detections) {
      if (existing.class_id == det.class_id &&
          (DetectionIoU(existing, det) > coco_config::kNmsThreshold ||
           DetectionOverlapOverSmaller(existing, det) >=
               coco_config::kRoiContainmentThreshold)) {
        duplicate = true;
        ++deduplicated;
        if (prefer_higher_score && det.score > existing.score) existing = det;
        break;
      }
    }
    if (!duplicate) full_result->detections.push_back(det);
  }
  return deduplicated;
}

void MapRoiDetectionsToCrop(const CocoDetectionResult& roi_result,
                            const ZoneRoiWindow& roi,
                            const std::array<int, 2>& crop_shape,
                            CocoDetectionResult* mapped_result) {
  if (mapped_result == nullptr) return;
  mapped_result->Clear();
  if (!roi.enabled) return;
  for (const CocoDetection& source : roi_result.detections) {
    CocoDetection det = source;
    det.box_xyxy[0] += static_cast<float>(roi.x);
    det.box_xyxy[1] += static_cast<float>(roi.y);
    det.box_xyxy[2] += static_cast<float>(roi.x);
    det.box_xyxy[3] += static_cast<float>(roi.y);
    for (int axis = 0; axis < 2; ++axis) {
      const float limit = static_cast<float>(crop_shape[axis] - 1);
      det.box_xyxy[axis] = std::max(0.0f, std::min(det.box_xyxy[axis], limit));
      det.box_xyxy[axis + 2] = std::max(0.0f, std::min(det.box_xyxy[axis + 2], limit));
    }
    if (det.box_xyxy[2] <= det.box_xyxy[0] || det.box_xyxy[3] <= det.box_xyxy[1]) {
      continue;
    }
    mapped_result->detections.push_back(det);
  }
}

bool ExtractJsonInt(const std::string& json_line, const char* key, int* value) {
  const std::string token = std::string("\"") + key + "\"";
  const std::size_t key_pos = json_line.find(token);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::size_t colon_pos = json_line.find(':', key_pos + token.size());
  if (colon_pos == std::string::npos) {
    return false;
  }
  std::size_t number_pos = colon_pos + 1;
  while (number_pos < json_line.size() &&
         (json_line[number_pos] == ' ' || json_line[number_pos] == '\t')) {
    ++number_pos;
  }
  return std::sscanf(json_line.c_str() + number_pos, "%d", value) == 1;
}

std::vector<int> ExtractJsonIntArray(const std::string& text) {
  std::vector<int> values;
  for (std::size_t i = 0; i < text.size();) {
    if (text[i] != '-' && !std::isdigit(static_cast<unsigned char>(text[i]))) {
      ++i;
      continue;
    }
    char* end_ptr = nullptr;
    const long value = std::strtol(text.c_str() + i, &end_ptr, 10);
    if (end_ptr == text.c_str() + i) {
      ++i;
      continue;
    }
    values.push_back(static_cast<int>(value));
    i = static_cast<std::size_t>(end_ptr - text.c_str());
  }
  return values;
}

bool ExtractJsonArrayText(const std::string& json_line,
                          const char* key,
                          std::string* array_text) {
  const std::string token = std::string("\"") + key + "\"";
  const std::size_t key_pos = json_line.find(token);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::size_t colon_pos = json_line.find(':', key_pos + token.size());
  if (colon_pos == std::string::npos) {
    return false;
  }
  const std::size_t array_start = json_line.find('[', colon_pos);
  if (array_start == std::string::npos) {
    return false;
  }
  int depth = 0;
  for (std::size_t pos = array_start; pos < json_line.size(); ++pos) {
    if (json_line[pos] == '[') {
      ++depth;
    } else if (json_line[pos] == ']') {
      --depth;
      if (depth == 0) {
        *array_text = json_line.substr(array_start, pos - array_start + 1);
        return true;
      }
    }
  }
  return false;
}

int CocoClassIdByName(const std::string& name) {
  for (std::size_t i = 0; i < coco_config::kClassNames.size(); ++i) {
    if (name == coco_config::kClassNames[i]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::vector<int> ExtractJsonStringClassNames(const std::string& array_text) {
  std::vector<int> ids;
  std::size_t pos = 0;
  while (pos < array_text.size()) {
    const std::size_t start = array_text.find('"', pos);
    if (start == std::string::npos) {
      break;
    }
    const std::size_t end = array_text.find('"', start + 1);
    if (end == std::string::npos) {
      break;
    }
    const std::string name = array_text.substr(start + 1, end - start - 1);
    const int class_id = CocoClassIdByName(name);
    if (class_id >= 0 && std::find(ids.begin(), ids.end(), class_id) == ids.end()) {
      ids.push_back(class_id);
    }
    pos = end + 1;
  }
  return ids;
}

void ParseAlarmClassIds(const std::string& json_line, GuardZone* zone) {
  std::string array_text;
  if (!ExtractJsonArrayText(json_line, "alarm_class_ids", &array_text) &&
      !ExtractJsonArrayText(json_line, "alarm_classes", &array_text)) {
    return;
  }

  std::vector<int> ids = ExtractJsonStringClassNames(array_text);
  if (ids.empty()) {
    ids = ExtractJsonIntArray(array_text);
    ids.erase(std::remove_if(ids.begin(), ids.end(),
                             [](int id) { return id < 0 || id >= coco_config::kNumClasses; }),
              ids.end());
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  }
  if (!ids.empty()) {
    zone->alarm_class_ids = ids;
  }
}

bool ParsePolygonZoneJson(const std::string& json_line, GuardZone* zone) {
  const std::string token = "\"points\"";
  const std::size_t key_pos = json_line.find(token);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::size_t colon_pos = json_line.find(':', key_pos + token.size());
  if (colon_pos == std::string::npos) {
    return false;
  }
  const std::size_t array_start = json_line.find('[', colon_pos);
  const std::size_t array_end = json_line.rfind(']');
  if (array_start == std::string::npos || array_end == std::string::npos ||
      array_end <= array_start) {
    return false;
  }

  std::vector<int> values = ExtractJsonIntArray(json_line.substr(array_start, array_end - array_start + 1));
  if (values.size() < 6 || (values.size() % 2) != 0) {
    return false;
  }

  std::vector<ZonePoint> points;
  points.reserve(values.size() / 2);
  for (std::size_t i = 0; i + 1 < values.size(); i += 2) {
    points.push_back(ZonePoint(values[i], values[i + 1]));
  }
  GuardZone parsed;
  parsed.SetPolygon(points);
  if (!parsed.active) {
    return false;
  }
  ParseAlarmClassIds(json_line, &parsed);
  *zone = parsed;
  return true;
}

bool ParseRectZoneJson(const std::string& json_line, GuardZone* zone) {
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  if (!ExtractJsonInt(json_line, "x1", &x1) ||
      !ExtractJsonInt(json_line, "y1", &y1) ||
      !ExtractJsonInt(json_line, "x2", &x2) ||
      !ExtractJsonInt(json_line, "y2", &y2)) {
    return false;
  }
  GuardZone parsed;
  parsed.SetRect(x1, y1, x2, y2);
  ParseAlarmClassIds(json_line, &parsed);
  *zone = parsed;
  return true;
}

bool ParseZoneJson(const std::string& json_line, GuardZone* zone) {
  if (json_line.find("\"polygon\"") != std::string::npos) {
    return ParsePolygonZoneJson(json_line, zone);
  }
  return ParseRectZoneJson(json_line, zone);
}

bool SaveZoneToFile(const GuardZone& zone, const char* path) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }
  output << "{\n"
         << "  \"type\": \"zone_update\",\n";
  if (zone.shape == "polygon") {
    output << "  \"shape\": \"polygon\",\n"
           << "  \"points\": [";
    for (std::size_t i = 0; i < zone.points.size(); ++i) {
      if (i > 0) output << ", ";
      output << "[" << zone.points[i].x << ", " << zone.points[i].y << "]";
    }
    output << "],\n";
  } else {
    output << "  \"shape\": \"rect\",\n"
           << "  \"x1\": " << zone.X1() << ",\n"
           << "  \"y1\": " << zone.Y1() << ",\n"
           << "  \"x2\": " << zone.X2() << ",\n"
           << "  \"y2\": " << zone.Y2() << ",\n";
  }
  output << "  \"alarm_classes\": [";
  for (std::size_t i = 0; i < zone.alarm_class_ids.size(); ++i) {
    if (i > 0) output << ", ";
    const int class_id = zone.alarm_class_ids[i];
    if (class_id >= 0 && class_id < coco_config::kNumClasses) {
      output << "\"" << coco_config::kClassNames[class_id] << "\"";
    } else {
      output << class_id;
    }
  }
  output << "]\n";
  output << "}\n";
  output.close();
  return static_cast<bool>(output);
}

bool LoadZoneFromFile(const char* path, GuardZone* zone) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return false;
  }
  std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  return ParseZoneJson(content, zone);
}

bool IsPointInsidePolygon(float x, float y, const std::vector<ZonePoint>& points) {
  bool inside = false;
  const std::size_t count = points.size();
  for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
    const float xi = static_cast<float>(points[i].x);
    const float yi = static_cast<float>(points[i].y);
    const float xj = static_cast<float>(points[j].x);
    const float yj = static_cast<float>(points[j].y);
    const bool intersects = ((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / ((yj - yi) == 0.0f ? 0.0001f : (yj - yi)) + xi);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

bool IsDetectionInsideZone(const CocoDetection& det, const GuardZone& zone) {
  if (!zone.active) {
    return true;
  }
  const float center_x = (det.box_xyxy[0] + det.box_xyxy[2]) * 0.5f;
  const float center_y = (det.box_xyxy[1] + det.box_xyxy[3]) * 0.5f;
  if (zone.shape == "polygon") {
    return IsPointInsidePolygon(center_x, center_y, zone.points);
  }
  return center_x >= zone.X1() && center_x <= zone.X2() &&
         center_y >= zone.Y1() && center_y <= zone.Y2();
}

void FilterDetectionsByZone(CocoDetectionResult* result, const GuardZone& zone) {
  if (!zone.active) {
    return;
  }
  std::vector<CocoDetection> filtered;
  filtered.reserve(result->detections.size());
  for (const auto& det : result->detections) {
    if (IsDetectionInsideZone(det, zone)) {
      filtered.push_back(det);
    }
  }
  result->detections.swap(filtered);
}

// 划分正常/报警检测：如果 zone 激活且 det 在 zone 内 + 类别属于报警类 -> 报警；否则正常
void ClassifyDetections(const CocoDetectionResult& result,
                         const GuardZone& zone,
                         ArmMode mode,
                         std::vector<std::array<float, 4>>* normal_boxes,
                        std::vector<std::array<float, 4>>* alarm_boxes) {
  normal_boxes->clear();
  alarm_boxes->clear();
  const std::vector<int>& alarm_ids = zone.alarm_class_ids.empty()
      ? GuardZone::DefaultAlarmClassIds()
      : zone.alarm_class_ids;

  for (const auto& det : result.detections) {
    bool is_alarm_class = std::find(alarm_ids.begin(), alarm_ids.end(), det.class_id) !=
                          alarm_ids.end();
    bool inside = zone.active && IsDetectionInsideZone(det, zone);
    if (is_alarm_class && ModeAllowsAlarmClass(det.class_id, mode) && inside) {
      alarm_boxes->push_back(det.box_xyxy);
    } else {
      normal_boxes->push_back(det.box_xyxy);
    }
  }
}

// zone坐标存在裁剪空间(1440x1080)，OSD显示和判断均使用真实多边形。
// OSD位图提交失败时降级为外接矩形，避免危险区完全不可见。
void RefreshZoneOverlay(VISUALIZER* visualizer, const GuardZone& zone) {
  if (visualizer == nullptr) return;
  visualizer->ClearZoneOverlay();
  if (!zone.active) return;
  if (zone.shape == "polygon") {
    std::vector<std::array<int, 2>> pts;
    pts.reserve(zone.points.size());
    for (const auto& p : zone.points) {
      pts.push_back({p.x + coco_config::kCropOffsetX, p.y});
    }
    if (visualizer->DrawZonePolygon(pts)) {
      printf("[ZONE] judgement=polygon display=polygon-rle points=%zu\n", zone.points.size());
    } else {
      printf("[ZONE][WARN] polygon OSD failed; fallback display=bbox points=%zu\n",
             zone.points.size());
      visualizer->DrawZonePolygonBBox(pts);
    }
  } else {
    printf("[ZONE] judgement=rect display=rect\n");
    visualizer->DrawZoneRect(zone.X1() + coco_config::kCropOffsetX,
                             zone.Y1(),
                             zone.X2() + coco_config::kCropOffsetX,
                             zone.Y2());
  }
}

void FilterDetectionsByAlarmClasses(CocoDetectionResult* result,
                                    const GuardZone& zone) {
  const std::vector<int>& alarm_class_ids = zone.alarm_class_ids.empty()
      ? GuardZone::DefaultAlarmClassIds()
      : zone.alarm_class_ids;
  std::vector<CocoDetection> filtered;
  filtered.reserve(result->detections.size());
  for (const auto& det : result->detections) {
    if (std::find(alarm_class_ids.begin(), alarm_class_ids.end(), det.class_id) !=
            alarm_class_ids.end()) {
      filtered.push_back(det);
    }
  }
  result->detections.swap(filtered);
}

bool FindBestAlarmDetection(const CocoDetectionResult& result,
                            const GuardZone& zone,
                            ArmMode mode,
                            CocoDetection* best) {
  if (best == nullptr) return false;
  bool found = false;
  for (const auto& det : result.detections) {
    if (!zone.active || !ModeAllowsAlarmClass(det.class_id, mode) ||
        !IsDetectionInsideZone(det, zone)) continue;
    if (!found || det.score > best->score) {
      *best = det;
      found = true;
    }
  }
  return found;
}

std::vector<unsigned char> BuildPgmSnapshot(const ssne_tensor_t& img_sensor,
                                            const std::array<int, 2>& crop_shape) {
  const int width = crop_shape[0];
  const int height = crop_shape[1];
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));

  std::string header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  std::vector<unsigned char> pgm(header.begin(), header.end());
  pgm.reserve(header.size() + width * height);

  // UYVY格式：亮度Y在奇数字节
  for (int i = 0; i < width * height; ++i) {
    pgm.push_back(src[i * 2 + 1]);
  }
  return pgm;
}

// 后台线程：收到UYVY帧后编码为PGM写入SnapshotBuffer
void SnapshotEncodeWorker(RawFrameBuffer* raw_buf, SnapshotBuffer* snap_buf) {
  while (!check_exit_flag()) {
    std::vector<unsigned char> uyvy;
    std::array<int, 2> cs;
    {
      std::unique_lock<std::mutex> lock(raw_buf->mutex);
      raw_buf->cv.wait_for(lock, std::chrono::seconds(1),
          [raw_buf] { return raw_buf->pending || check_exit_flag(); });
      if (!raw_buf->pending) continue;
      uyvy = std::move(raw_buf->uyvy);
      cs = raw_buf->crop_shape;
      raw_buf->pending = false;
    }
    const int w = cs[0];
    const int h = cs[1];
    if (w <= 0 || h <= 0 || uyvy.size() < static_cast<size_t>(w * h * 2)) continue;
    std::string header = "P5\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n";
    std::vector<unsigned char> pgm(header.begin(), header.end());
    pgm.reserve(header.size() + w * h);
    for (int i = 0; i < w * h; ++i) {
      pgm.push_back(uyvy[i * 2 + 1]);
    }
    {
      std::lock_guard<std::mutex> lock(snap_buf->mutex);
      snap_buf->pgm_bytes = std::move(pgm);
      snap_buf->width  = w;
      snap_buf->height = h;
      snap_buf->ready  = true;
    }
  }
}

std::vector<unsigned char> BuildPreviewPgm(const ssne_tensor_t& img_sensor,
                                           const std::array<int, 2>& crop_shape,
                                           int preview_width,
                                           int preview_height) {
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));
  const int source_width = crop_shape[0];
  const int source_height = crop_shape[1];

  std::string header = "P5\n" + std::to_string(preview_width) + " " +
                       std::to_string(preview_height) + "\n255\n";
  std::vector<unsigned char> pgm(header.begin(), header.end());
  pgm.reserve(header.size() + preview_width * preview_height);

  for (int y = 0; y < preview_height; ++y) {
    const int src_y = y * source_height / preview_height;
    for (int x = 0; x < preview_width; ++x) {
      const int src_x = x * source_width / preview_width;
      const int pixel_index = src_y * source_width + src_x;
      pgm.push_back(src[pixel_index * 2 + 1]);
    }
  }
  return pgm;
}

static unsigned char ClampToByte(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return static_cast<unsigned char>(value);
}

static void YuvToRgb(int y, int u, int v,
                     unsigned char* r,
                     unsigned char* g,
                     unsigned char* b) {
  const int c = y - 16;
  const int d = u - 128;
  const int e = v - 128;
  *r = ClampToByte((298 * c + 409 * e + 128) >> 8);
  *g = ClampToByte((298 * c - 100 * d - 208 * e + 128) >> 8);
  *b = ClampToByte((298 * c + 516 * d + 128) >> 8);
}

std::vector<unsigned char> BuildPreviewPpm(const ssne_tensor_t& img_sensor,
                                           const std::array<int, 2>& crop_shape,
                                           int preview_width,
                                           int preview_height) {
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));
  const int source_width = crop_shape[0];
  const int source_height = crop_shape[1];

  std::string header = "P6\n" + std::to_string(preview_width) + " " +
                       std::to_string(preview_height) + "\n255\n";
  std::vector<unsigned char> ppm(header.begin(), header.end());
  ppm.reserve(header.size() + preview_width * preview_height * 3);

  for (int y = 0; y < preview_height; ++y) {
    const int src_y = y * source_height / preview_height;
    for (int x = 0; x < preview_width; ++x) {
      const int src_x = x * source_width / preview_width;
      const int pair_x = src_x & ~1;
      const int pair_index = src_y * source_width + pair_x;
      const unsigned char* pair = src + pair_index * 2;

      // UYVY: U0 Y0 V0 Y1,相邻像素共用UV
      const int u_value = pair[0];
      const int y_value = (src_x & 1) ? pair[3] : pair[1];
      const int v_value = pair[2];

      unsigned char r;
      unsigned char g;
      unsigned char b;
      YuvToRgb(y_value, u_value, v_value, &r, &g, &b);
      ppm.push_back(r);
      ppm.push_back(g);
      ppm.push_back(b);
    }
  }
  return ppm;
}

static uint32_t QoiHash(unsigned char r, unsigned char g, unsigned char b, unsigned char a) {
  return (r * 3u + g * 5u + b * 7u + a * 11u) % 64u;
}

static void AppendBe32(std::vector<unsigned char>* out, uint32_t value) {
  out->push_back(static_cast<unsigned char>((value >> 24) & 0xff));
  out->push_back(static_cast<unsigned char>((value >> 16) & 0xff));
  out->push_back(static_cast<unsigned char>((value >> 8) & 0xff));
  out->push_back(static_cast<unsigned char>(value & 0xff));
}

std::vector<unsigned char> BuildPreviewQoi(const ssne_tensor_t& img_sensor,
                                           const std::array<int, 2>& crop_shape,
                                           int preview_width,
                                           int preview_height) {
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));
  const int source_width = crop_shape[0];
  const int source_height = crop_shape[1];

  std::vector<unsigned char> qoi;
  qoi.reserve(14 + preview_width * preview_height * 3 / 2);
  qoi.push_back('q');
  qoi.push_back('o');
  qoi.push_back('i');
  qoi.push_back('f');
  AppendBe32(&qoi, static_cast<uint32_t>(preview_width));
  AppendBe32(&qoi, static_cast<uint32_t>(preview_height));
  qoi.push_back(3);
  qoi.push_back(0);

  unsigned char index[64][4] = {{0}};
  unsigned char prev_r = 0;
  unsigned char prev_g = 0;
  unsigned char prev_b = 0;
  const unsigned char prev_a = 255;
  int run = 0;

  for (int y = 0; y < preview_height; ++y) {
    const int src_y = y * source_height / preview_height;
    for (int x = 0; x < preview_width; ++x) {
      const int src_x = x * source_width / preview_width;
      const int pair_x = src_x & ~1;
      const int pair_index = src_y * source_width + pair_x;
      const unsigned char* pair = src + pair_index * 2;

      const int u_value = pair[0];
      const int y_value = (src_x & 1) ? pair[3] : pair[1];
      const int v_value = pair[2];

      unsigned char r;
      unsigned char g;
      unsigned char b;
      YuvToRgb(y_value, u_value, v_value, &r, &g, &b);

      if (r == prev_r && g == prev_g && b == prev_b) {
        ++run;
        if (run == 62) {
          qoi.push_back(static_cast<unsigned char>(0xc0 | (run - 1)));
          run = 0;
        }
        continue;
      }

      if (run > 0) {
        qoi.push_back(static_cast<unsigned char>(0xc0 | (run - 1)));
        run = 0;
      }

      const uint32_t index_pos = QoiHash(r, g, b, prev_a);
      if (index[index_pos][0] == r && index[index_pos][1] == g &&
          index[index_pos][2] == b && index[index_pos][3] == prev_a) {
        qoi.push_back(static_cast<unsigned char>(index_pos));
      } else {
        index[index_pos][0] = r;
        index[index_pos][1] = g;
        index[index_pos][2] = b;
        index[index_pos][3] = prev_a;

        const int dr = static_cast<int>(r) - static_cast<int>(prev_r);
        const int dg = static_cast<int>(g) - static_cast<int>(prev_g);
        const int db = static_cast<int>(b) - static_cast<int>(prev_b);
        const int dr_dg = dr - dg;
        const int db_dg = db - dg;

        if (dr >= -2 && dr <= 1 && dg >= -2 && dg <= 1 && db >= -2 && db <= 1) {
          qoi.push_back(static_cast<unsigned char>(
              0x40 | ((dr + 2) << 4) | ((dg + 2) << 2) | (db + 2)));
        } else if (dg >= -32 && dg <= 31 &&
                   dr_dg >= -8 && dr_dg <= 7 &&
                   db_dg >= -8 && db_dg <= 7) {
          qoi.push_back(static_cast<unsigned char>(0x80 | (dg + 32)));
          qoi.push_back(static_cast<unsigned char>(((dr_dg + 8) << 4) | (db_dg + 8)));
        } else {
          qoi.push_back(0xfe);
          qoi.push_back(r);
          qoi.push_back(g);
          qoi.push_back(b);
        }
      }

      prev_r = r;
      prev_g = g;
      prev_b = b;
    }
  }

  if (run > 0) {
    qoi.push_back(static_cast<unsigned char>(0xc0 | (run - 1)));
  }
  for (int i = 0; i < 7; ++i) qoi.push_back(0);
  qoi.push_back(1);
  return qoi;
}

void UpdateSnapshotBuffer(const ssne_tensor_t& img_sensor,
                          const std::array<int, 2>& crop_shape,
                          SnapshotBuffer* snapshot) {
  std::vector<unsigned char> pgm = BuildPgmSnapshot(img_sensor, crop_shape);
  std::lock_guard<std::mutex> lock(snapshot->mutex);
  snapshot->pgm_bytes.swap(pgm);
  snapshot->width = crop_shape[0];
  snapshot->height = crop_shape[1];
  snapshot->ready = true;
}

bool SaveSnapshotToFile(const std::vector<unsigned char>& pgm, const char* path) {
  const std::string temp_path = std::string(path) + ".tmp";
  std::ofstream output(temp_path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }
  output.write(reinterpret_cast<const char*>(pgm.data()), static_cast<std::streamsize>(pgm.size()));
  output.close();
  if (!output) {
    return false;
  }
  return std::rename(temp_path.c_str(), path) == 0;
}

bool ApplyZoneCommand(UartControlChannel* uart, const std::string& json,
                      GuardZone* zone, VISUALIZER* visualizer) {
  GuardZone parsed;
  if (!ParseZoneJson(json, &parsed)) {
    uart->SendTextLine("ERR ZONE");
    return false;
  }
  if (!SaveZoneToFile(parsed, coco_config::kZoneConfigPath)) {
    fprintf(stderr, "[ZONE] Failed to save zone to %s; using in-memory zone only\n",
            coco_config::kZoneConfigPath);
  }
  *zone = parsed;
  printf("[ZONE] Updated: %s\n", zone->Describe().c_str());
  RefreshZoneOverlay(visualizer, *zone);
  uart->SendTextLine("OK ZONE");
  return true;
}

bool SendSerialSnapshot(UartControlChannel* uart,
                        IMAGEPROCESSOR* processor,
                        const std::array<int, 2>& crop_shape) {
  ssne_tensor_t img_sensor;
  if (!processor->GetImage(&img_sensor)) {
    uart->SendTextLine("ERR SNAPSHOT");
    return false;
  }
  SensorTensorInfo tensor_info;
  std::string invalid_reason;
  if (!InspectSensorTensor(img_sensor, crop_shape, &tensor_info, &invalid_reason)) {
    fprintf(stderr, "[SNAPSHOT][WARN] Invalid sensor tensor: %s\n",
            invalid_reason.c_str());
    uart->SendTextLine("ERR SNAPSHOT");
    return false;
  }
  std::vector<unsigned char> preview = BuildPreviewQoi(
      img_sensor, crop_shape, coco_config::kSerialPreviewWidth, coco_config::kSerialPreviewHeight);
  const std::string ppm_header = "P6\n" +
                                 std::to_string(coco_config::kSerialPreviewWidth) + " " +
                                 std::to_string(coco_config::kSerialPreviewHeight) + "\n255\n";
  const size_t ppm_payload_size =
      ppm_header.size() +
      static_cast<size_t>(coco_config::kSerialPreviewWidth) *
      static_cast<size_t>(coco_config::kSerialPreviewHeight) * 3u;
  if (preview.size() >= ppm_payload_size) {
    preview = BuildPreviewPpm(
        img_sensor, crop_shape, coco_config::kSerialPreviewWidth, coco_config::kSerialPreviewHeight);
  }
  std::string header = "SNAPSHOT " +
                       std::to_string(coco_config::kSerialPreviewWidth) + " " +
                       std::to_string(coco_config::kSerialPreviewHeight) + " " +
                       std::to_string(crop_shape[0]) + " " +
                       std::to_string(crop_shape[1]) + " " +
                       std::to_string(preview.size()) + "\n";

  // The UART API is backed by a 32-byte TX FIFO.  On the final short write,
  // part of the FIFO can remain pending until another write is submitted.
  // Keep these transport-only bytes outside the payload size advertised in
  // the header: they flush the complete QOI tail onto the wire, while the PC
  // reads exactly preview.size() bytes and then drains this padding.
  const std::array<uint8_t, 64> transport_flush = {{0}};
  return uart->SendBytes(reinterpret_cast<const uint8_t*>(header.data()), header.size()) &&
         uart->SendBytes(preview.data(), preview.size()) &&
         uart->SendBytes(transport_flush.data(), transport_flush.size());
}

bool RunSerialSetup(UartControlChannel* uart,
                    IMAGEPROCESSOR* processor,
                    const std::array<int, 2>& crop_shape,
                    GuardZone* zone,
                    VISUALIZER* visualizer,
                    ArmMode* arm_mode,
                    DebounceTracker* tracker,
                    AlarmLifecycle* alarm_lifecycle,
                    GpioAlarmController* gpio_alarm,
                    bool gpio_ready) {
  if (LoadZoneFromFile(coco_config::kZoneConfigPath, zone)) {
    printf("[SETUP] Loaded existing zone: %s\n", zone->Describe().c_str());
  } else {
    printf("[SETUP] No existing zone config found\n");
  }
  printf("[SETUP] Waiting serial commands: SNAPSHOT | ZONE <json> | MODE HOME/AWAY/SLEEP/CONFIG | START\n");

  std::string line;
  while (!check_exit_flag()) {
    if (!uart->ReceiveLine(&line, 200)) {
      continue;
    }
    if (line == "SNAPSHOT") {
      if (!SendSerialSnapshot(uart, processor, crop_shape)) {
        fprintf(stderr, "[SETUP] Failed to send snapshot over UART\n");
        return false;
      }
    } else if (line.rfind("ZONE ", 0) == 0) {
      ApplyZoneCommand(uart, line.substr(5), zone, visualizer);
    } else if (line.rfind("MODE ", 0) == 0) {
      ArmMode parsed_mode;
      if (ParseArmMode(line.substr(5), &parsed_mode)) {
        ApplyArmMode(parsed_mode, arm_mode, tracker, alarm_lifecycle,
                     gpio_alarm, gpio_ready);
        uart->SendTextLine(std::string("OK MODE ") + ArmModeName(*arm_mode));
      } else {
        uart->SendTextLine("ERR MODE");
      }
    } else if (line == "START") {
      uart->SendTextLine("OK START");
      return true;
    } else if (line == "QUIT") {
      return false;
    } else if (!line.empty()) {
      uart->SendTextLine("ERR CMD");
    }
  }
  return false;
}

void PollRuntimeSerial(UartControlChannel* uart,
                       IMAGEPROCESSOR* processor,
                       const std::array<int, 2>& crop_shape,
                       GuardZone* zone,
                       VISUALIZER* visualizer,
                       ArmMode* arm_mode,
                       DebounceTracker* tracker,
                       AlarmLifecycle* alarm_lifecycle,
                       GpioAlarmController* gpio_alarm,
                       bool gpio_ready,
                       RuntimeTestControl* test_control) {
  if (uart == nullptr || !uart->IsOpen()) {
    return;
  }
  std::string line;
  int handled = 0;
  while (handled < 4 && uart->ReceiveLine(&line, 0)) {
    ++handled;
    if (line == "SNAPSHOT") {
      if (test_control != nullptr && test_control->camera_fail_enabled) {
        uart->SendTextLine("ERR SNAPSHOT CAMERA TEST");
      } else if (!SendSerialSnapshot(uart, processor, crop_shape)) {
        fprintf(stderr, "[UART] Failed to send runtime snapshot\n");
      }
    } else if (line.rfind("ZONE ", 0) == 0) {
      ApplyZoneCommand(uart, line.substr(5), zone, visualizer);
    } else if (line.rfind("MODE ", 0) == 0) {
      ArmMode parsed_mode;
      if (ParseArmMode(line.substr(5), &parsed_mode)) {
        ApplyArmMode(parsed_mode, arm_mode, tracker, alarm_lifecycle,
                     gpio_alarm, gpio_ready);
        uart->SendTextLine(std::string("OK MODE ") + ArmModeName(*arm_mode));
      } else {
        uart->SendTextLine("ERR MODE");
      }
    } else if (line.rfind("TEST ", 0) == 0 && test_control != nullptr) {
      const std::string command = line.substr(5);
      if (command == "LOAD ON") {
        test_control->load_enabled = true;
        uart->SendTextLine("OK TEST LOAD ON");
        printf("[TEST] synthetic base load enabled delay=%dms\n",
               coco_config::kTestLoadDelayMs);
      } else if (command == "LOAD OFF") {
        test_control->load_enabled = false;
        uart->SendTextLine("OK TEST LOAD OFF");
        printf("[TEST] synthetic base load disabled\n");
      } else if (command == "CAM FAIL ON") {
        test_control->camera_fail_enabled = true;
        uart->SendTextLine("OK TEST CAM FAIL ON");
        printf("[TEST] synthetic camera failure enabled\n");
      } else if (command == "CAM FAIL OFF") {
        test_control->camera_fail_enabled = false;
        uart->SendTextLine("OK TEST CAM FAIL OFF");
        printf("[TEST] synthetic camera failure disabled\n");
      } else if (command == "STATUS") {
        uart->SendTextLine(
            std::string("OK TEST STATUS LOAD=") +
            (test_control->load_enabled ? "ON" : "OFF") +
            " CAM_FAIL=" +
            (test_control->camera_fail_enabled ? "ON" : "OFF"));
      } else {
        uart->SendTextLine("ERR TEST");
      }
    } else if (line == "START") {
      uart->SendTextLine("OK START");
    } else if (line == "QUIT") {
      std::lock_guard<std::mutex> lock(g_mtx);
      g_exit_flag = true;
    } else if (!line.empty()) {
      uart->SendTextLine("ERR CMD");
    }
  }
}

int main() {
  std::array<int, 2> img_shape  = coco_config::kImageShape;
  std::array<int, 2> crop_shape = coco_config::kCropShape;
  std::array<int, 2> det_shape  = coco_config::kDetShape;
  std::string        model_path = coco_config::kModelPath;

  if (ssne_initial()) {
    fprintf(stderr, "[INIT] SSNE initialization failed!\n");
    return -1;
  }
  printf("[INIT] SSNE initialized\n");

  IMAGEPROCESSOR processor;
  const bool camera_initial_opened = processor.Initialize(&img_shape);
  if (!camera_initial_opened) {
    fprintf(stderr, "[CAM][ALARM] Initial pipeline open failed; recovery mode enabled\n");
  }

  COCO_DETECTOR detector;
  AcceptanceStats accept_stats;
  accept_stats.camera_init_attempts = 1;
  if (!camera_initial_opened) {
    accept_stats.camera_init_failures = 1;
    accept_stats.camera_recovery_cycles = 1;
  }
  bool detector_ready = false;
  if (!FileReadable(model_path)) {
    ++accept_stats.resource_warnings;
    fprintf(stderr, "[RESOURCE][ALARM] model missing: %s; detection disabled\n",
            model_path.c_str());
  } else if (!detector.Initialize(model_path, &crop_shape, &det_shape)) {
    ++accept_stats.resource_warnings;
    fprintf(stderr, "[RESOURCE][ALARM] Detector init failed; detection disabled\n");
  } else {
    detector_ready = true;
    printf("[INIT] Detector loaded: %s\n", model_path.c_str());
  }

  VISUALIZER visualizer;
  const std::string color_lut_path = "/app_demo/app_assets/colorLUT.sscl";
  if (!FileReadable(color_lut_path)) {
    ++accept_stats.resource_warnings;
    fprintf(stderr, "[RESOURCE][WARN] color LUT missing: %s; OSD will try default path\n",
            color_lut_path.c_str());
  }
  // 切换到 colorLUT.sscl（21 RGB 条目）以支持白/红/黄三色 OSD 显示
  visualizer.Initialize(img_shape, "colorLUT.sscl");

  const char* const status_bitmap_names[] = {
      coco_config::kStatusHomeBitmapName,
      coco_config::kStatusAwayBitmapName,
      coco_config::kStatusSleepBitmapName,
      coco_config::kStatusConfigBitmapName,
      coco_config::kStatusNoZoneBitmapName,
      coco_config::kStatusAlarmBitmapName,
      coco_config::kStatusDegradedBitmapName,
  };
  bool status_bitmaps_ready = true;
  for (size_t i = 0; i < sizeof(status_bitmap_names) / sizeof(status_bitmap_names[0]); ++i) {
    const std::string path = "/app_demo/app_assets/" + std::string(status_bitmap_names[i]);
    if (!FileReadable(path)) {
      status_bitmaps_ready = false;
      ++accept_stats.resource_warnings;
      fprintf(stderr, "[RESOURCE][WARN] status bitmap missing: %s\n", path.c_str());
    }
  }

  GpioAlarmController gpio_alarm;
  bool gpio_ready = gpio_alarm.Initialize();
  if (!gpio_ready) {
    ++accept_stats.resource_warnings;
    fprintf(stderr, "[RESOURCE][WARN] GPIO alarm disabled; detection/OSD will continue\n");
  } else {
    printf("[INIT] GPIO alarm ready\n");
  }

  usleep(200000);
  printf("[INIT] System ready -- input q to quit\n");

  ssne_tensor_t       img_sensor;
  CocoDetectionResult det_result;
  DebounceTracker     tracker;
  SnapshotBuffer      snapshot_buffer;
  RawFrameBuffer      raw_frame_buf;
  GuardZone           active_zone;
  ArmMode             arm_mode = ArmMode::kAway;
  AlarmLifecycle      alarm_lifecycle;
  UartControlChannel  uart_channel;

  if (LoadZoneFromFile(coco_config::kZoneConfigPath, &active_zone)) {
    printf("[ZONE] Loaded zone: %s\n", active_zone.Describe().c_str());
  } else {
    printf("[ZONE] No zone config found, detections will run without zone filtering\n");
  }

  bool uart_ready = false;
  if (coco_config::kEnableSerialSetup) {
    if (!uart_channel.Initialize(coco_config::kSerialBaudrate)) {
      ++accept_stats.resource_warnings;
      fprintf(stderr, "[RESOURCE][WARN] UART setup disabled; using existing/default zone\n");
    } else {
      uart_ready = true;
      if (!RunSerialSetup(&uart_channel, &processor, crop_shape, &active_zone,
                          &visualizer, &arm_mode, &tracker, &alarm_lifecycle,
                          &gpio_alarm, gpio_ready)) {
        if (check_exit_flag()) {
          uart_channel.Release();
          if (gpio_ready) gpio_alarm.Release();
          if (detector_ready) detector.Release();
          processor.Release();
          visualizer.Release();
          if (ssne_release()) {
            fprintf(stderr, "SSNE release failed!\n");
            return -1;
          }
          return 0;
        }
        ++accept_stats.resource_warnings;
        fprintf(stderr, "[RESOURCE][WARN] Serial setup incomplete; continuing run mode\n");
      }
    }
  }

  // 启动后立即把已加载的 zone 绘制为黄色框
  RefreshZoneOverlay(&visualizer, active_zone);

  const bool static_degraded = !detector_ready || !gpio_ready || !uart_ready;
  if (status_bitmaps_ready) {
    visualizer.ShowStatusCard(
        StatusBitmapName(arm_mode, active_zone.active, false,
                         static_degraded || !camera_initial_opened),
        coco_config::kStatusBitmapPosX, coco_config::kStatusBitmapPosY);
  }

  std::thread listener_thread(keyboard_listener);
  SnapshotHttpServer snapshot_server(coco_config::kSnapshotHttpPort, &snapshot_buffer);
  std::thread snapshot_thread(&SnapshotHttpServer::Run, &snapshot_server);
  std::thread snapshot_encode_thread(SnapshotEncodeWorker, &raw_frame_buf, &snapshot_buffer);

  auto last_log_time      = std::chrono::steady_clock::now();
  auto last_snapshot_time = std::chrono::steady_clock::now() -
                            std::chrono::milliseconds(coco_config::kRunSnapshotUpdateIntervalMs);
  auto fps_window_start   = std::chrono::steady_clock::now();
  auto last_brightness_log = std::chrono::steady_clock::now();
  auto acceptance_start    = std::chrono::steady_clock::now();
  constexpr int kDetLogIntervalMs    = coco_config::kDetectionSummaryLogMs;
  constexpr int kIdleLogIntervalMs   = 5000;
  constexpr int kFpsLogIntervalMs    = 1000;
  constexpr int kBrightnessLogMs     = coco_config::kEnvLogIntervalMs;
  constexpr int kLatencyReportEveryN = 60;   // 每 60 帧上报一次 P95 延迟
  constexpr float kSensorFps         = 45.0f;  // SC235HAI Task1: 1920x1080@45fps
  constexpr int kCamFailMax          = 5;
  constexpr int kInferFailMax        = 30;   // ~0.5s of consecutive inference failures
  constexpr int kDataFailMax         = 3;
  constexpr int kCameraValidFrames   = 3;
  constexpr int kCameraAttemptsCycle = 3;
  constexpr int kCameraRetryMs       = 200;
  constexpr int kCameraCooldownMs    = 1500;

  int fps_frame_count   = 0;
  int cam_fail_count    = 0;
  int infer_fail_count  = 0;
  int data_fail_count   = 0;
  EnvPolicyState env_state;
  RoiLoadController roi_load;
  RuntimeTestControl test_control;
  float last_app_fps = 0.0f;
  int no_detection_frames = 0;
  int roi_frame_index = 0;
  CocoDetectionResult cached_roi_result;
  RoiResultMetadata cached_roi_metadata;
  std::uint64_t inference_frame_id = 0;
  std::uint64_t roi_pipeline_generation = 1;
  GuardZone observed_roi_zone = active_zone;
  auto roi_retry_after = std::chrono::steady_clock::now();

  enum class CameraHealth {
    kRecovering,
    kValidating,
    kHealthy
  };
  enum class CameraFaultCause {
    kStartup,
    kCapture,
    kTensor
  };
  CameraHealth camera_health = camera_initial_opened
                                   ? CameraHealth::kValidating
                                   : CameraHealth::kRecovering;
  CameraFaultCause camera_fault_cause = CameraFaultCause::kStartup;
  bool camera_recovery_pending = !camera_initial_opened;
  int camera_valid_count = 0;
  int camera_attempts_in_cycle = 0;
  auto next_camera_attempt = std::chrono::steady_clock::now();

  // 端到端延迟样本环（帧捕获 -> OSD刷新），用于 P95 统计
  std::vector<long long> latency_samples;
  latency_samples.reserve(kLatencyReportEveryN);
  PerfWindow perf_window;
  perf_window.Reserve(kLatencyReportEveryN);
  bool selftest_camera_reported = false;

  auto invalidate_roi_cache = [&]() {
    cached_roi_result.Clear();
    cached_roi_metadata = RoiResultMetadata();
  };

  auto sync_roi_zone_generation = [&]() {
    if (SameGuardZone(observed_roi_zone, active_zone)) return;
    observed_roi_zone = active_zone;
    ++roi_pipeline_generation;
    invalidate_roi_cache();
    roi_frame_index = 0;
    roi_retry_after = std::chrono::steady_clock::now();
    tracker.Reset();
    roi_load.recovery_votes = 0;
    roi_load.priority_holdoff_windows = std::max(
        roi_load.priority_holdoff_windows,
        coco_config::kRoiZoneChangeHoldoffWindows);
    if (roi_load.state == RoiLoadState::kEvery2) {
      roi_load.state = RoiLoadState::kEvery5;
      ++accept_stats.roi_load_transitions;
      printf("[LOAD] roi=EVERY2->EVERY5 reason=zone-change holdoff=%d\n",
             roi_load.priority_holdoff_windows);
    }
    printf("[ROI][GEN] generation=%llu reason=zone-change cache=cleared\n",
           static_cast<unsigned long long>(roi_pipeline_generation));
  };

  auto clear_camera_dependent_state = [&](const char* reason) {
    const auto clear_now = std::chrono::steady_clock::now();
    const long long clear_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        clear_now.time_since_epoch()).count();
    if (alarm_lifecycle.active) {
      const long long duration_ms = std::max(0LL, clear_now_ms - alarm_lifecycle.started_ms);
      AppendAlarmEvent("END_CAMERA_FAULT",
                       alarm_lifecycle.has_detection
                           ? &alarm_lifecycle.last_detection : nullptr,
                       duration_ms);
      ++alarm_lifecycle.ends;
    }
    alarm_lifecycle.ResetActiveState();
    tracker.Reset();
    det_result.Clear();
    ++roi_pipeline_generation;
    invalidate_roi_cache();
    roi_retry_after = clear_now;
    roi_frame_index = 0;
    infer_fail_count = 0;
    cam_fail_count = 0;
    data_fail_count = 0;
    fps_frame_count = 0;
    last_app_fps = 0.0f;
    no_detection_frames = 0;
    roi_load = RoiLoadController();
    fps_window_start = clear_now;
    last_snapshot_time = clear_now -
                         std::chrono::milliseconds(coco_config::kRunSnapshotUpdateIntervalMs);
    latency_samples.clear();
    perf_window.Clear();
    if (gpio_ready) gpio_alarm.Reset();
    std::vector<std::array<float, 4>> empty_boxes;
    visualizer.DrawDetections(empty_boxes, empty_boxes);
    {
      std::lock_guard<std::mutex> lock(raw_frame_buf.mutex);
      raw_frame_buf.uyvy.clear();
      raw_frame_buf.crop_shape = {0, 0};
      raw_frame_buf.pending = false;
    }
    {
      std::lock_guard<std::mutex> lock(snapshot_buffer.mutex);
      snapshot_buffer.pgm_bytes.clear();
      snapshot_buffer.width = 0;
      snapshot_buffer.height = 0;
      snapshot_buffer.ready = false;
    }
    printf("[CAM][RESET] stale inference/alarm state cleared reason=%s\n", reason);
  };

  auto enter_camera_recovery = [&](CameraFaultCause cause, const char* reason) {
    if (camera_health == CameraHealth::kRecovering) return;
    clear_camera_dependent_state(reason);
    processor.Release();
    camera_health = CameraHealth::kRecovering;
    camera_fault_cause = cause;
    camera_recovery_pending = true;
    camera_valid_count = 0;
    camera_attempts_in_cycle = 0;
    next_camera_attempt = std::chrono::steady_clock::now();
    selftest_camera_reported = false;
    ++accept_stats.camera_recovery_cycles;
    if (status_bitmaps_ready) {
      visualizer.ShowStatusCard(
          coco_config::kStatusDegradedBitmapName,
          coco_config::kStatusBitmapPosX, coco_config::kStatusBitmapPosY);
    }
    fprintf(stderr, "[CAM][RECOVERY] entered reason=%s cycle=%d\n",
            reason, accept_stats.camera_recovery_cycles);
  };

  printf("[CHECK][BEGIN] duration=%dms mode=%s sensor_fps=%.0f det=%dx%d\n",
         coco_config::kAcceptanceDurationMs,
         coco_config::kEnableAcceptanceMode ? "ACCEPT" : "NORMAL",
         kSensorFps,
         det_shape[0],
         det_shape[1]);
  printf("[CHECK][FEATURE] detect=%s zone=%s osd=ON gpio=%s uart=%s snapshot=MEM%s env=ON exceptions=ON mode=%s events=ON selftest=ON roi_load=%s\n",
         BoolText(detector_ready),
         active_zone.active ? active_zone.shape.c_str() : "OFF",
         BoolText(gpio_ready),
         BoolText(uart_ready),
          coco_config::kSaveSnapshotFileInRun ? "+FILE" : "",
          ArmModeName(arm_mode),
          RoiLoadStateName(roi_load.state));
  printf("[SELFTEST] camera=PENDING model=%s gpio=%s uart=%s osd=%s zone=%s mode=%s health=%s\n",
         BoolText(detector_ready),
         BoolText(gpio_ready),
         BoolText(uart_ready),
         BoolText(status_bitmaps_ready),
         active_zone.active ? "READY" : "NOT_CONFIGURED",
         ArmModeName(arm_mode),
         detector_ready && gpio_ready && uart_ready && status_bitmaps_ready && active_zone.active
             ? "READY" : "DEGRADED");
  if (accept_stats.resource_warnings > 0) {
    printf("[CHECK][DEGRADE] resource_warnings=%d detector=%s gpio=%s uart=%s status_bitmaps=%s\n",
           accept_stats.resource_warnings,
           BoolText(detector_ready),
           BoolText(gpio_ready),
           BoolText(uart_ready),
           BoolText(status_bitmaps_ready));
  }

  while (!check_exit_flag()) {
    const auto loop_start = std::chrono::steady_clock::now();
    const auto now = loop_start;
    long long capture_us = 0;
    long long full_infer_us = 0;
    long long roi_work_us = 0;
    long long osd_us = 0;
    bool roi_work_sampled = false;

    // --- [异常类1] 摄像头异常处理 ---
    if (camera_health == CameraHealth::kRecovering) {
      // Keep the control plane responsive while the camera pipeline is down.
      // MODE/ZONE commands must not be coupled to successful frame capture.
      if (uart_ready) {
        PollRuntimeSerial(&uart_channel, &processor, crop_shape, &active_zone,
                          &visualizer, &arm_mode, &tracker, &alarm_lifecycle,
                          &gpio_alarm, gpio_ready, &test_control);
      }
      sync_roi_zone_generation();
      if (now < next_camera_attempt) {
        usleep(20000);
        continue;
      }

      ++camera_attempts_in_cycle;
      ++accept_stats.camera_init_attempts;
      printf("[CAM][RECOVERY] initialize attempt=%d/%d total=%d\n",
             camera_attempts_in_cycle,
             kCameraAttemptsCycle,
             accept_stats.camera_init_attempts);
      const bool pipeline_opened =
          !test_control.camera_fail_enabled && processor.Initialize(&img_shape);
      if (pipeline_opened) {
        camera_health = CameraHealth::kValidating;
        camera_valid_count = 0;
        cam_fail_count = 0;
        data_fail_count = 0;
        printf("[CAM][RECOVERY] pipeline opened; validating %d consecutive frames\n",
               kCameraValidFrames);
      } else {
        ++accept_stats.camera_init_failures;
        if (camera_attempts_in_cycle >= kCameraAttemptsCycle) {
          camera_attempts_in_cycle = 0;
          next_camera_attempt = now + std::chrono::milliseconds(kCameraCooldownMs);
          fprintf(stderr,
                  "[CAM][RECOVERY] initialize cycle failed; cooldown=%dms failures=%d\n",
                  kCameraCooldownMs,
                  accept_stats.camera_init_failures);
        } else {
          next_camera_attempt = now + std::chrono::milliseconds(kCameraRetryMs);
        }
      }
      continue;
    }

    const bool image_ready =
        !test_control.camera_fail_enabled && processor.GetImage(&img_sensor);
    if (!image_ready) {
      ++cam_fail_count;
      if (cam_fail_count == 1) {
        fprintf(stderr, "[CAM][ALARM] Camera frame acquisition failed\n");
      }
      if (camera_health == CameraHealth::kValidating) {
        ++accept_stats.camera_validation_failures;
        enter_camera_recovery(CameraFaultCause::kCapture,
                              "capture failed during validation");
      } else if (cam_fail_count >= kCamFailMax) {
        enter_camera_recovery(CameraFaultCause::kCapture,
                              "consecutive capture failures");
      }
      continue;
    }
    capture_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - loop_start).count();

    // --- [异常类2] 数据异常处理 ---
    // 校验张量数据指针、维度合法性
    {
      SensorTensorInfo tensor_info;
      std::string tensor_reason;
      if (!InspectSensorTensor(img_sensor, crop_shape, &tensor_info, &tensor_reason)) {
        ++data_fail_count;
        if (data_fail_count == 1) {
          fprintf(stderr, "[DATA][ALARM] Invalid sensor tensor: %s\n",
                  tensor_reason.c_str());
        }
        if (camera_health == CameraHealth::kValidating) {
          ++accept_stats.camera_validation_failures;
          enter_camera_recovery(CameraFaultCause::kTensor,
                                "invalid tensor during validation");
        } else if (data_fail_count >= kDataFailMax) {
          enter_camera_recovery(CameraFaultCause::kTensor,
                                "consecutive invalid sensor tensors");
        }
        continue;
      }

      cam_fail_count = 0;
      data_fail_count = 0;

      if (camera_health == CameraHealth::kValidating) {
        ++camera_valid_count;
        printf("[CAM][VALIDATE] frame=%d/%d shape=%ux%u format=%u bytes=%zu\n",
               camera_valid_count,
               kCameraValidFrames,
               tensor_info.width,
               tensor_info.height,
               static_cast<unsigned int>(tensor_info.format),
               tensor_info.memory_size);
        if (camera_valid_count < kCameraValidFrames) continue;

        camera_health = CameraHealth::kHealthy;
        camera_valid_count = 0;
        camera_attempts_in_cycle = 0;
        if (camera_recovery_pending) {
          if (camera_fault_cause == CameraFaultCause::kTensor) {
            ++accept_stats.data_recoveries;
          } else {
            ++accept_stats.camera_recoveries;
          }
          camera_recovery_pending = false;
        }
        fps_frame_count = 0;
        fps_window_start = now;
        last_snapshot_time = now -
                             std::chrono::milliseconds(coco_config::kRunSnapshotUpdateIntervalMs);
        if (status_bitmaps_ready) {
          visualizer.ShowStatusCard(
              StatusBitmapName(arm_mode, active_zone.active, false, static_degraded),
              coco_config::kStatusBitmapPosX, coco_config::kStatusBitmapPosY);
        }
        printf("[CAM][RECOVERY] healthy after %d consecutive valid frames\n",
               kCameraValidFrames);
      }

      if (!selftest_camera_reported) {
        printf("[SELFTEST] camera=OK model=%s gpio=%s uart=%s osd=%s zone=%s mode=%s health=%s\n",
               BoolText(detector_ready),
               BoolText(gpio_ready),
               BoolText(uart_ready),
               BoolText(status_bitmaps_ready),
               active_zone.active ? "READY" : "NOT_CONFIGURED",
               ArmModeName(arm_mode),
               detector_ready && gpio_ready && uart_ready && status_bitmaps_ready && active_zone.active
                   ? "READY" : "DEGRADED");
        selftest_camera_reported = true;
      }

      // 亮度统计 (鲁棒性): 采样 UYVY 中的 Y 通道
      const auto bright_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_brightness_log).count();
      if (bright_elapsed_ms >= kBrightnessLogMs) {
        const int avg_y = SampleAverageLuma(tensor_info.data, crop_shape);
        const bool policy_changed = UpdateEnvPolicy(avg_y, &env_state);
        printf("[ENV]  avg_luma=%d  policy=%s  conf=%.2f  hold=%dms%s\n",
               avg_y,
               EnvPolicyName(env_state.policy),
               env_state.conf_threshold,
               env_state.alarm_hold_ms,
               policy_changed ? "  changed=1" : "");
        last_brightness_log = now;
      }
    }

    if (uart_ready) {
      PollRuntimeSerial(&uart_channel, &processor, crop_shape, &active_zone,
                        &visualizer, &arm_mode, &tracker, &alarm_lifecycle,
                        &gpio_alarm, gpio_ready, &test_control);
    }
    sync_roi_zone_generation();

    if (test_control.load_enabled) {
      usleep(static_cast<useconds_t>(coco_config::kTestLoadDelayMs) * 1000U);
    }

    ++fps_frame_count;
    ++accept_stats.frames;
    ++inference_frame_id;
    const auto fps_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - fps_window_start).count();
    if (fps_elapsed_ms >= kFpsLogIntervalMs) {
      const float fps_app = fps_frame_count * 1000.0f / static_cast<float>(fps_elapsed_ms);
      const float ratio   = fps_app / kSensorFps;
      last_app_fps = fps_app;
      printf("[FPS]  app=%.1f  sensor=%.0f  R=%.2f  score_est=%d\n",
             fps_app, kSensorFps, ratio, EstimateFpsScore(ratio));
      fps_frame_count  = 0;
      fps_window_start = now;
    }

    const auto snapshot_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_snapshot_time).count();
    if (snapshot_elapsed_ms >= coco_config::kRunSnapshotUpdateIntervalMs) {
      void* data_ptr = get_data(img_sensor);
      if (data_ptr) {
        const size_t sz = static_cast<size_t>(crop_shape[0]) * crop_shape[1] * 2;
        std::lock_guard<std::mutex> lock(raw_frame_buf.mutex);
        raw_frame_buf.uyvy.assign(
            static_cast<const unsigned char*>(data_ptr),
            static_cast<const unsigned char*>(data_ptr) + sz);
        raw_frame_buf.crop_shape = crop_shape;
        raw_frame_buf.pending    = true;
        raw_frame_buf.cv.notify_one();
      }
      last_snapshot_time = now;
    }

    // --- [异常类3] 推理异常处理 ---
    if (detector_ready) {
      const auto full_infer_start = std::chrono::steady_clock::now();
      const bool full_infer_ok =
          detector.Predict(&img_sensor, &det_result, env_state.conf_threshold);
      full_infer_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - full_infer_start).count();
      if (!full_infer_ok) {
        ++infer_fail_count;
        ++accept_stats.infer_failures;
        fprintf(stderr, "[INFER][ALARM] Inference failed (%d consecutive)\n", infer_fail_count);
        if (infer_fail_count >= kInferFailMax) {
          fprintf(stderr, "[INFER][ALARM] Too many inference failures, skipping OSD this cycle\n");
        }
        visualizer.Draw({});
        continue;
      }
      infer_fail_count = 0;
    } else {
      det_result.Clear();
    }

    // Optional local enhancement: the existing danger zone is the only user
    // configuration. The full-frame result remains authoritative on failures.
    ++roi_frame_index;
    ZoneRoiWindow roi;
    const int roi_interval_frames = RoiLoadIntervalFrames(roi_load.state);
    const bool roi_load_enabled = roi_interval_frames > 0;
    const bool roi_enabled = detector_ready &&
                             BuildZoneRoi(active_zone, crop_shape, &roi);
    if (cached_roi_metadata.valid &&
        (!roi_load_enabled || !roi_enabled ||
         !SameRoiWindow(roi, cached_roi_metadata.window))) {
      ++accept_stats.roi_cache_drops;
      invalidate_roi_cache();
    }

    if (detector_ready && roi_load_enabled &&
        roi_frame_index % roi_interval_frames == 0) {
      if (roi_enabled && std::chrono::steady_clock::now() >= roi_retry_after) {
        const auto roi_work_start = std::chrono::steady_clock::now();
        roi_work_sampled = true;
        ssne_tensor_t roi_tensor;
        if (BuildZoneRoiTensor(img_sensor, crop_shape, roi, &roi_tensor)) {
          CocoDetectionResult roi_result;
          const std::array<int, 2> roi_shape = {roi.side, roi.side};
          ++accept_stats.roi_runs;
          if (detector.Predict(&roi_tensor, &roi_result,
                               env_state.conf_threshold, &roi_shape)) {
            CocoDetectionResult mapped_roi_result;
            MapRoiDetectionsToCrop(roi_result, roi, crop_shape,
                                   &mapped_roi_result);
            accept_stats.roi_deduplicated +=
                MergeCropDetections(&det_result, mapped_roi_result, true);
            // A successful empty ROI result must replace any older non-empty
            // cache; otherwise a disappeared person can remain as a ghost box.
            cached_roi_result = mapped_roi_result;
            cached_roi_metadata.frame_id = inference_frame_id;
            cached_roi_metadata.pipeline_generation = roi_pipeline_generation;
            cached_roi_metadata.window = roi;
            cached_roi_metadata.timestamp = std::chrono::steady_clock::now();
            cached_roi_metadata.valid = true;
            ++accept_stats.roi_cache_updates;
            roi_retry_after = std::chrono::steady_clock::now();
          } else {
            ++accept_stats.roi_failures;
            roi_retry_after = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(
                                  coco_config::kRoiFailureBackoffMs);
            fprintf(stderr, "[ROI][WARN] local inference failed; using full-frame result\n");
          }
          release_tensor(roi_tensor);
        } else {
          ++accept_stats.roi_failures;
          roi_retry_after = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(
                                coco_config::kRoiFailureBackoffMs);
          fprintf(stderr, "[ROI][WARN] local tensor creation failed; using full-frame result\n");
        }
        roi_work_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - roi_work_start).count();
      } else if (active_zone.active) {
        ++accept_stats.roi_skipped;
      }
    } else if (detector_ready && active_zone.active && !roi_load_enabled) {
      ++accept_stats.roi_load_paused_frames;
    }

    if (cached_roi_metadata.valid) {
      const auto cache_now = std::chrono::steady_clock::now();
      const auto roi_cache_age_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              cache_now - cached_roi_metadata.timestamp).count();
      const bool frame_order_valid =
          inference_frame_id >= cached_roi_metadata.frame_id;
      const int roi_result_max_frame_gap = roi_load_enabled
          ? std::min(coco_config::kRoiResultMaxFrameGap,
                     roi_interval_frames + 1)
          : 0;
      const bool frame_gap_valid = frame_order_valid &&
          inference_frame_id - cached_roi_metadata.frame_id <=
              static_cast<std::uint64_t>(roi_result_max_frame_gap);
      const bool cache_valid =
          roi_load_enabled && roi_enabled &&
          SameRoiWindow(roi, cached_roi_metadata.window) &&
          cached_roi_metadata.pipeline_generation == roi_pipeline_generation &&
          frame_gap_valid &&
          roi_cache_age_ms <= coco_config::kRoiResultCacheMs;
      if (cache_valid && cached_roi_metadata.frame_id < inference_frame_id) {
        // Fill only the frames between ROI runs. The fresh ROI result was
        // already merged above, so it must not be merged twice.
        accept_stats.roi_deduplicated +=
            MergeCropDetections(&det_result, cached_roi_result, false);
        ++accept_stats.roi_cache_hits;
      } else if (!cache_valid) {
        ++accept_stats.roi_cache_drops;
        invalidate_roi_cache();
      }
    }

    // 追踪和区域判断都在裁剪坐标系(1440x1080)下完成
    FilterDetectionsByAlarmClasses(&det_result, active_zone);

    tracker.Update(det_result);
    CocoDetectionResult stable_crop = tracker.ConfirmedDetections();

    // 判断完后坐标转回1920x1080给OSD显示
    std::vector<std::array<float, 4>> normal_boxes;
    std::vector<std::array<float, 4>> alarm_boxes;
    ClassifyDetections(stable_crop, active_zone, arm_mode, &normal_boxes, &alarm_boxes);
    ConvertCropBoxesToOriginal(&normal_boxes);
    ConvertCropBoxesToOriginal(&alarm_boxes);

    CocoDetectionResult stable_display = stable_crop;
    ConvertCropBoxesToOriginal(&stable_display);

    const bool has_object       = !stable_crop.detections.empty();
    if (has_object) {
      no_detection_frames = 0;
    } else if (no_detection_frames < coco_config::kRoiTargetLostFrames +
                                      kLatencyReportEveryN) {
      ++no_detection_frames;
    }
    const bool raw_alarm_active = !alarm_boxes.empty();
    CocoDetection best_alarm_detection;
    const bool has_best_alarm = FindBestAlarmDetection(
        stable_crop, active_zone, arm_mode, &best_alarm_detection);
    const bool lifecycle_changed = alarm_lifecycle.Update(
        raw_alarm_active, std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch()).count(),
        has_best_alarm ? &best_alarm_detection : nullptr);
    const bool is_alarm_active = alarm_lifecycle.active;
    if (lifecycle_changed && alarm_lifecycle.active) {
      AppendAlarmEvent("START", has_best_alarm ? &best_alarm_detection : nullptr, 0);
    } else if (lifecycle_changed && !alarm_lifecycle.active) {
      const long long duration_ms =
          std::max(0LL, std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch()).count() - alarm_lifecycle.started_ms);
      AppendAlarmEvent("END", alarm_lifecycle.has_detection
                                  ? &alarm_lifecycle.last_detection : nullptr,
                       duration_ms);
    }
    if (has_object) {
      ++accept_stats.detection_frames;
      accept_stats.detections += static_cast<int>(stable_crop.detections.size());
    }
    if (is_alarm_active) {
      ++accept_stats.alarm_frames;
      accept_stats.alarm_detections += static_cast<int>(alarm_boxes.size());
    }

    // GPIO 报警仅在 zone 内触发（更准确反映安防意图）
    if (gpio_ready) {
      gpio_alarm.Update(is_alarm_active, env_state.alarm_hold_ms,
                        GpioModeForArmMode(arm_mode));
    }

    // 状态卡按事件变化切换；相同位图命中缓存，不产生逐帧 OSD 写入。
    if (status_bitmaps_ready) {
      const auto status_osd_start = std::chrono::steady_clock::now();
      visualizer.ShowStatusCard(
          StatusBitmapName(arm_mode, active_zone.active,
                           is_alarm_active, static_degraded),
          coco_config::kStatusBitmapPosX, coco_config::kStatusBitmapPosY);
      osd_us += std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - status_osd_start).count();
    }

    const auto log_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_log_time).count();
    const int log_interval = has_object ? kDetLogIntervalMs : kIdleLogIntervalMs;
    if (log_elapsed_ms >= log_interval) {
      if (has_object) {
        if (coco_config::kVerboseDetectionLog) {
          for (const auto& det : stable_display.detections) {
            printf("[DET]  %-10s  conf=%.2f  [%.0f,%.0f,%.0f,%.0f]\n",
                   det.label.c_str(), det.score,
                   det.box_xyxy[0], det.box_xyxy[1], det.box_xyxy[2], det.box_xyxy[3]);
          }
        } else {
          float best_score = 0.0f;
          for (const auto& det : stable_display.detections) {
            best_score = std::max(best_score, det.score);
          }
          printf("[DET]  objects=%zu  alarm=%zu  best_conf=%.2f  policy=%s\n",
                 stable_display.detections.size(),
                 alarm_boxes.size(),
                 best_score,
                 EnvPolicyName(env_state.policy));
        }
        if (is_alarm_active) {
          printf("[ALARM] %zu object(s) inside danger zone\n", alarm_boxes.size());
        }
      } else {
        printf("[IDLE] no detection\n");
      }
      last_log_time = now;
    }

    const auto detection_osd_start = std::chrono::steady_clock::now();
    visualizer.DrawDetections(normal_boxes, alarm_boxes);
    osd_us += std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - detection_osd_start).count();

    // 端到端延迟统计 (帧捕获 -> 绘制完成)
    const auto loop_end = std::chrono::steady_clock::now();
    const long long total_loop_us = std::chrono::duration_cast<std::chrono::microseconds>(
        loop_end - loop_start).count();
    const long long base_loop_us = std::max(0LL, total_loop_us - roi_work_us);
    const long long deadline_us = static_cast<long long>(
        coco_config::kApplicationDeadlineMs * 1000.0f);
    const bool base_deadline_missed = base_loop_us > deadline_us;
    const bool total_deadline_missed = total_loop_us > deadline_us;
    const long long latency_ms = total_loop_us / 1000;
    latency_samples.push_back(latency_ms);
    accept_stats.latency_ms.push_back(latency_ms);
    perf_window.capture_us.push_back(capture_us);
    perf_window.full_infer_us.push_back(full_infer_us);
    if (roi_work_sampled) {
      perf_window.roi_work_us.push_back(roi_work_us);
      ++accept_stats.roi_perf_samples;
      accept_stats.roi_work_us_total += roi_work_us;
    }
    perf_window.osd_us.push_back(osd_us);
    perf_window.base_loop_us.push_back(base_loop_us);
    perf_window.total_loop_us.push_back(total_loop_us);
    if (base_deadline_missed) {
      ++perf_window.base_deadline_misses;
      ++accept_stats.base_deadline_misses;
    }
    if (total_deadline_missed) {
      ++perf_window.total_deadline_misses;
      ++accept_stats.total_deadline_misses;
    }
    ++accept_stats.perf_samples;
    accept_stats.capture_us_total += capture_us;
    accept_stats.full_infer_us_total += full_infer_us;
    accept_stats.osd_us_total += osd_us;
    accept_stats.base_loop_us_total += base_loop_us;
    accept_stats.total_loop_us_total += total_loop_us;
    if (static_cast<int>(latency_samples.size()) >= kLatencyReportEveryN) {
      std::vector<long long> sorted = latency_samples;
      std::sort(sorted.begin(), sorted.end());
      const long long p50 = sorted[sorted.size() * 50 / 100];
      const long long p95 = sorted[sorted.size() * 95 / 100];
      const long long p99 = sorted[sorted.size() * 99 / 100];
      printf("[LAT]  p50=%lldms  p95=%lldms  p95_T=%.1f  p99=%lldms  score_est=%d  (n=%zu)\n",
             p50,
             p95,
             ToFramePeriods(p95, kSensorFps),
             p99,
             EstimateLatencyScore(p95, kSensorFps),
             sorted.size());
      const float cap_p95 = PercentileUsAsMs(perf_window.capture_us, 95);
      const float full_p95 = PercentileUsAsMs(perf_window.full_infer_us, 95);
      const float roi_p95 = PercentileUsAsMs(perf_window.roi_work_us, 95);
      const float osd_p95 = PercentileUsAsMs(perf_window.osd_us, 95);
      const float base_p95 = PercentileUsAsMs(perf_window.base_loop_us, 95);
      const float total_p95 = PercentileUsAsMs(perf_window.total_loop_us, 95);
      const float perf_sample_count = static_cast<float>(
          std::max<size_t>(1, perf_window.total_loop_us.size()));
      const float base_miss_pct =
          100.0f * perf_window.base_deadline_misses / perf_sample_count;
      const float total_miss_pct =
          100.0f * perf_window.total_deadline_misses / perf_sample_count;
      const bool priority_roi = active_zone.active && detector_ready &&
          (env_state.policy != EnvPolicy::kNormal ||
           no_detection_frames >= coco_config::kRoiTargetLostFrames);
      const bool overloaded =
          full_p95 > coco_config::kRoiOverloadPathP95Ms ||
          base_p95 > coco_config::kRoiOverloadPathP95Ms ||
          (last_app_fps > 0.0f &&
           last_app_fps < coco_config::kRoiOverloadAppFps);
      const bool healthy =
          full_p95 < coco_config::kRoiRecoveryPathP95Ms &&
          base_p95 < coco_config::kRoiRecoveryPathP95Ms &&
          last_app_fps >= coco_config::kRoiRecoveryAppFps;
      const bool priority_healthy =
          priority_roi && healthy &&
          last_app_fps >= coco_config::kRoiPriorityMinAppFps;

      if (!active_zone.active || !detector_ready) {
        roi_load = RoiLoadController();
      } else if (roi_load.warmup_windows <
                 coco_config::kRoiLoadWarmupWindows) {
        ++roi_load.warmup_windows;
        roi_load.overload_votes = 0;
        roi_load.recovery_votes = 0;
      } else {
        if (overloaded) {
          ++roi_load.overload_votes;
          roi_load.recovery_votes = 0;
        } else if (healthy) {
          ++roi_load.recovery_votes;
          roi_load.overload_votes = 0;
        } else {
          roi_load.overload_votes = 0;
          roi_load.recovery_votes = 0;
        }

        const RoiLoadState previous_load_state = roi_load.state;
        const char* transition_reason = nullptr;
        if (roi_load.overload_votes >=
                coco_config::kRoiOverloadVoteWindows &&
            roi_load.state != RoiLoadState::kPaused) {
          roi_load.state = RoiLoadState::kPaused;
          transition_reason = "overload";
        } else if (roi_load.state == RoiLoadState::kPaused &&
                   roi_load.recovery_votes >=
                       coco_config::kRoiRecoveryVoteWindows) {
          roi_load.state = RoiLoadState::kEvery10;
          transition_reason = "recovery-step1";
          roi_load.priority_holdoff_windows = std::max(
              roi_load.priority_holdoff_windows,
              coco_config::kRoiPriorityHoldoffWindows);
        } else if (roi_load.state == RoiLoadState::kEvery10 &&
                   roi_load.recovery_votes >=
                       coco_config::kRoiRecoveryVoteWindows) {
          roi_load.state = RoiLoadState::kEvery5;
          transition_reason = "recovery-step2";
        } else if (roi_load.state == RoiLoadState::kEvery5 &&
                   priority_healthy &&
                   roi_load.priority_holdoff_windows == 0 &&
                   roi_load.recovery_votes >=
                       coco_config::kRoiRecoveryVoteWindows) {
          roi_load.state = RoiLoadState::kEvery2;
          transition_reason = "priority-step3";
        } else if (roi_load.state == RoiLoadState::kEvery2 &&
                   (!priority_healthy ||
                    roi_load.priority_holdoff_windows > 0)) {
          roi_load.state = RoiLoadState::kEvery5;
          transition_reason = "priority-budget-clear";
          roi_load.priority_holdoff_windows = std::max(
              roi_load.priority_holdoff_windows,
              coco_config::kRoiPriorityHoldoffWindows);
        }

        if (roi_load.state != previous_load_state) {
          ++accept_stats.roi_load_transitions;
          roi_load.overload_votes = 0;
          roi_load.recovery_votes = 0;
          invalidate_roi_cache();
          roi_frame_index = 0;
          printf("[LOAD] roi=%s->%s reason=%s full_p95=%.2fms base_p95=%.2fms app=%.1f priority=%d holdoff=%d\n",
                 RoiLoadStateName(previous_load_state),
                 RoiLoadStateName(roi_load.state),
                 transition_reason ? transition_reason : "policy",
                 full_p95,
                 base_p95,
                 last_app_fps,
                 priority_roi ? 1 : 0,
                 roi_load.priority_holdoff_windows);
        } else if (healthy && roi_load.priority_holdoff_windows > 0) {
          --roi_load.priority_holdoff_windows;
        }
      }

      printf("[PERF] n=%zu roi_n=%zu cap_p95=%.2fms full_p95=%.2fms roi_p95=%.2fms osd_p95=%.2fms base_p95=%.2fms total_p95=%.2fms deadline=%.1fms base_miss=%.1f%% total_miss=%.1f%% load=%s interval=%d app=%.1f votes=%d/%d priority=%d ready=%d holdoff=%d test=%d/%d\n",
             perf_window.total_loop_us.size(),
             perf_window.roi_work_us.size(),
             cap_p95,
             full_p95,
             roi_p95,
             osd_p95,
             base_p95,
             total_p95,
             coco_config::kApplicationDeadlineMs,
             base_miss_pct,
             total_miss_pct,
             RoiLoadStateName(roi_load.state),
             RoiLoadIntervalFrames(roi_load.state),
             last_app_fps,
             roi_load.overload_votes,
             roi_load.recovery_votes,
             priority_roi ? 1 : 0,
             priority_healthy ? 1 : 0,
             roi_load.priority_holdoff_windows,
             test_control.load_enabled ? 1 : 0,
             test_control.camera_fail_enabled ? 1 : 0);
      latency_samples.clear();
      perf_window.Clear();
    }

    if (coco_config::kEnableAcceptanceMode && !accept_stats.summary_printed) {
      const auto accept_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          loop_end - acceptance_start).count();
      if (accept_elapsed_ms >= coco_config::kAcceptanceDurationMs) {
        const float runtime_sec = static_cast<float>(accept_elapsed_ms) / 1000.0f;
        const float avg_fps = accept_stats.frames / std::max(0.001f, runtime_sec);
        const float ratio = avg_fps / kSensorFps;
        const long long p95 = PercentileMs(accept_stats.latency_ms, 95);
        const char* stable_state =
            (accept_stats.camera_recoveries == 0 &&
             accept_stats.data_recoveries == 0 &&
             accept_stats.camera_init_failures == 0 &&
             accept_stats.camera_validation_failures == 0 &&
             accept_stats.infer_failures == 0) ? "PASS" : "PASS_WITH_RECOVERY";
        printf("[CHECK][SUMMARY] runtime=%.1fs stable=%s frames=%d avg_app=%.1f R=%.2f fps_score_est=%d p95=%lldms p95_T=%.1f latency_score_est=%d\n",
               runtime_sec,
               stable_state,
               accept_stats.frames,
               avg_fps,
               ratio,
               EstimateFpsScore(ratio),
               p95,
               ToFramePeriods(p95, kSensorFps),
               EstimateLatencyScore(p95, kSensorFps));
        printf("[CHECK][COUNTS] det_frames=%d alarm_frames=%d detections=%d alarm_detections=%d alarm_starts=%d alarm_ends=%d cam_recoveries=%d data_recoveries=%d cam_cycles=%d init_attempts=%d init_failures=%d validation_failures=%d infer_failures=%d resource_warnings=%d roi_runs=%d roi_failures=%d roi_skipped=%d roi_cache_updates=%d roi_cache_hits=%d roi_cache_drops=%d roi_deduplicated=%d load_transitions=%d load_paused_frames=%d roi_load=%s roi_interval=%d env_policy=%s mode=%s\n",
               accept_stats.detection_frames,
               accept_stats.alarm_frames,
               accept_stats.detections,
               accept_stats.alarm_detections,
               alarm_lifecycle.starts,
               alarm_lifecycle.ends,
               accept_stats.camera_recoveries,
               accept_stats.data_recoveries,
               accept_stats.camera_recovery_cycles,
               accept_stats.camera_init_attempts,
               accept_stats.camera_init_failures,
               accept_stats.camera_validation_failures,
               accept_stats.infer_failures,
               accept_stats.resource_warnings,
               accept_stats.roi_runs,
               accept_stats.roi_failures,
               accept_stats.roi_skipped,
               accept_stats.roi_cache_updates,
                accept_stats.roi_cache_hits,
                accept_stats.roi_cache_drops,
                accept_stats.roi_deduplicated,
                accept_stats.roi_load_transitions,
                accept_stats.roi_load_paused_frames,
                RoiLoadStateName(roi_load.state),
                RoiLoadIntervalFrames(roi_load.state),
                EnvPolicyName(env_state.policy),
               ArmModeName(arm_mode));
        printf("[CHECK][PERF] samples=%d roi_samples=%d cap_avg=%.2fms full_avg=%.2fms roi_avg=%.2fms osd_avg=%.2fms base_avg=%.2fms total_avg=%.2fms deadline=%.1fms base_miss=%.1f%% total_miss=%.1f%%\n",
               accept_stats.perf_samples,
               accept_stats.roi_perf_samples,
               AverageUsAsMs(accept_stats.capture_us_total, accept_stats.perf_samples),
               AverageUsAsMs(accept_stats.full_infer_us_total, accept_stats.perf_samples),
               AverageUsAsMs(accept_stats.roi_work_us_total, accept_stats.roi_perf_samples),
               AverageUsAsMs(accept_stats.osd_us_total, accept_stats.perf_samples),
               AverageUsAsMs(accept_stats.base_loop_us_total, accept_stats.perf_samples),
               AverageUsAsMs(accept_stats.total_loop_us_total, accept_stats.perf_samples),
               coco_config::kApplicationDeadlineMs,
               100.0f * accept_stats.base_deadline_misses /
                   static_cast<float>(std::max(1, accept_stats.perf_samples)),
               100.0f * accept_stats.total_deadline_misses /
                   static_cast<float>(std::max(1, accept_stats.perf_samples)));
        accept_stats.summary_printed = true;
      }
    }
  }

  if (listener_thread.joinable()) {
    listener_thread.join();
  }
  raw_frame_buf.cv.notify_all();
  if (snapshot_encode_thread.joinable()) {
    snapshot_encode_thread.join();
  }
  if (snapshot_thread.joinable()) {
    snapshot_thread.join();
  }

  if (uart_ready) {
    uart_channel.Release();
  }
  if (gpio_ready) {
    gpio_alarm.Release();
  }
  if (detector_ready) {
    detector.Release();
  }
  processor.Release();
  visualizer.Release();

  if (ssne_release()) {
    fprintf(stderr, "SSNE release failed!\n");
    return -1;
  }

  return 0;
}
