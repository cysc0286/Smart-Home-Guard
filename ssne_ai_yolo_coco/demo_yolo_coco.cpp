#include <algorithm>
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>

#include "include/gpio_alarm_controller.hpp"

#include "uart_api.h"

#include "include/coco_config.hpp"
#include "include/coco_detector.hpp"
#include "include/debounce_tracker.hpp"
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

struct RectZone {
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  bool active = false;

  void Normalize() {
    if (x1 > x2) std::swap(x1, x2);
    if (y1 > y2) std::swap(y1, y2);
  }
};

class UartControlChannel {
 public:
  UartControlChannel() = default;

  bool Initialize(uint32_t baudrate) {
    handle_ = uart_init();
    if (handle_ == NULL) {
      fprintf(stderr, "[UART] uart_init failed\n");
      return false;
    }

    uart_set_baudrate(handle_, UART_TX0, baudrate);
    uart_set_baudrate(handle_, UART_RX0, baudrate);
    uart_set_parity(handle_, UART_TX0, UART_PARITY_NONE);
    uart_set_parity(handle_, UART_RX0, UART_PARITY_NONE);
    printf("[UART] Ready at %u baud\n", baudrate);
    return true;
  }

  void Release() {
    if (handle_ != NULL) {
      uart_close(handle_);
      handle_ = NULL;
    }
  }

  bool SendTextLine(const std::string& line) {
    std::string payload = line;
    payload.push_back('\n');
    return SendBytes(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  }

  bool SendBytes(const uint8_t* data, size_t len) {
    if (handle_ == NULL) {
      return false;
    }

    size_t offset = 0;
    while (offset < len) {
      const uint32_t chunk =
          static_cast<uint32_t>(std::min<size_t>(len - offset, static_cast<size_t>(32)));
      uart_send_data(handle_, UART_TX0, const_cast<uint8_t*>(data + offset), chunk);
      offset += chunk;
    }
    return true;
  }

  bool ReceiveLine(std::string* out_line, int timeout_ms) {
    if (handle_ == NULL) {
      return false;
    }

    std::string line;
    auto start = std::chrono::steady_clock::now();
    while (true) {
      uint8_t buffer[32] = {0};
      uint32_t actual_len = 0;
      uart_receive_data(handle_, UART_RX0, buffer, sizeof(buffer), &actual_len);
      if (actual_len > 0) {
        for (uint32_t i = 0; i < actual_len; ++i) {
          const char ch = static_cast<char>(buffer[i]);
          if (ch == '\r') {
            continue;
          }
          if (ch == '\n') {
            *out_line = line;
            return true;
          }
          line.push_back(ch);
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
      if (elapsed_ms >= timeout_ms) {
        return false;
      }
      usleep(2000);
    }
  }

 private:
  uart_handle_t handle_ = NULL;
};

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

// Restore crop-offset: coordinates from the 1440x1080 crop must be shifted
// back to the full 1920x1080 image space before display.
void ConvertCropBoxesToOriginal(CocoDetectionResult* result) {
  for (auto& det : result->detections) {
    det.box_xyxy[0] += static_cast<float>(coco_config::kCropOffsetX);
    det.box_xyxy[2] += static_cast<float>(coco_config::kCropOffsetX);
  }
}

bool ParseRectZoneJson(const std::string& json_line, RectZone* zone) {
  auto extract_int = [&](const char* key, int* value) -> bool {
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
  };

  RectZone parsed;
  if (!extract_int("x1", &parsed.x1) ||
      !extract_int("y1", &parsed.y1) ||
      !extract_int("x2", &parsed.x2) ||
      !extract_int("y2", &parsed.y2)) {
    return false;
  }
  parsed.Normalize();
  parsed.active = true;
  *zone = parsed;
  return true;
}

bool SaveZoneToFile(const RectZone& zone, const char* path) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }
  output << "{\n"
         << "  \"type\": \"zone_update\",\n"
         << "  \"shape\": \"rect\",\n"
         << "  \"x1\": " << zone.x1 << ",\n"
         << "  \"y1\": " << zone.y1 << ",\n"
         << "  \"x2\": " << zone.x2 << ",\n"
         << "  \"y2\": " << zone.y2 << "\n"
         << "}\n";
  output.close();
  return static_cast<bool>(output);
}

bool LoadZoneFromFile(const char* path, RectZone* zone) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return false;
  }
  std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  return ParseRectZoneJson(content, zone);
}

bool IsDetectionInsideZone(const CocoDetection& det, const RectZone& zone) {
  if (!zone.active) {
    return true;
  }
  const float center_x = (det.box_xyxy[0] + det.box_xyxy[2]) * 0.5f;
  const float center_y = (det.box_xyxy[1] + det.box_xyxy[3]) * 0.5f;
  return center_x >= zone.x1 && center_x <= zone.x2 &&
         center_y >= zone.y1 && center_y <= zone.y2;
}

void FilterDetectionsByZone(CocoDetectionResult* result, const RectZone& zone) {
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

std::vector<unsigned char> BuildPgmSnapshot(const ssne_tensor_t& img_sensor,
                                            const std::array<int, 2>& crop_shape) {
  const int width = crop_shape[0];
  const int height = crop_shape[1];
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));

  std::string header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  std::vector<unsigned char> pgm(header.begin(), header.end());
  pgm.reserve(header.size() + width * height);

  // Online pipeline outputs YUV422 16-bit packed data. Luma bytes are every even byte.
  for (int i = 0; i < width * height; ++i) {
    pgm.push_back(src[i * 2]);
  }
  return pgm;
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
      pgm.push_back(src[pixel_index * 2]);
    }
  }
  return pgm;
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

bool RunSerialSetup(UartControlChannel* uart,
                    IMAGEPROCESSOR* processor,
                    const std::array<int, 2>& crop_shape,
                    RectZone* zone) {
  if (LoadZoneFromFile(coco_config::kZoneConfigPath, zone)) {
    printf("[SETUP] Loaded existing zone: (%d,%d)-(%d,%d)\n",
           zone->x1, zone->y1, zone->x2, zone->y2);
  } else {
    printf("[SETUP] No existing zone config found\n");
  }
  printf("[SETUP] Waiting serial commands: SNAPSHOT | ZONE <json> | START\n");

  std::string line;
  while (!check_exit_flag()) {
    if (!uart->ReceiveLine(&line, 200)) {
      continue;
    }
    if (line == "SNAPSHOT") {
      ssne_tensor_t img_sensor;
      processor->GetImage(&img_sensor);
      std::vector<unsigned char> preview = BuildPreviewPgm(
          img_sensor, crop_shape, coco_config::kSerialPreviewWidth, coco_config::kSerialPreviewHeight);
      std::string header = "SNAPSHOT " +
                           std::to_string(coco_config::kSerialPreviewWidth) + " " +
                           std::to_string(coco_config::kSerialPreviewHeight) + " " +
                           std::to_string(crop_shape[0]) + " " +
                           std::to_string(crop_shape[1]) + " " +
                           std::to_string(preview.size()) + "\n";
      if (!uart->SendBytes(reinterpret_cast<const uint8_t*>(header.data()), header.size()) ||
          !uart->SendBytes(preview.data(), preview.size())) {
        fprintf(stderr, "[SETUP] Failed to send snapshot over UART\n");
        return false;
      }
    } else if (line.rfind("ZONE ", 0) == 0) {
      RectZone parsed;
      if (!ParseRectZoneJson(line.substr(5), &parsed)) {
        uart->SendTextLine("ERR ZONE");
        continue;
      }
      if (!SaveZoneToFile(parsed, coco_config::kZoneConfigPath)) {
        uart->SendTextLine("ERR SAVE");
        continue;
      }
      *zone = parsed;
      uart->SendTextLine("OK ZONE");
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
  processor.Initialize(&img_shape);

  COCO_DETECTOR detector;
  if (!detector.Initialize(model_path, &crop_shape, &det_shape)) {
    fprintf(stderr, "[RESOURCE][ALARM] Detector init failed, aborting.\n");
    processor.Release();
    ssne_release();
    return -1;
  }
  printf("[INIT] Detector loaded: %s\n", model_path.c_str());

  VISUALIZER visualizer;
  visualizer.Initialize(img_shape, "shared_colorLUT.sscl");

  GpioAlarmController gpio_alarm;
  if (!gpio_alarm.Initialize()) {
    visualizer.Release();
    detector.Release();
    processor.Release();
    ssne_release();
    return -1;
  }
  printf("[INIT] GPIO alarm ready\n");

  usleep(200000);
  printf("[INIT] System ready -- input q to quit\n");

  ssne_tensor_t       img_sensor;
  CocoDetectionResult det_result;
  DebounceTracker     tracker;
  SnapshotBuffer      snapshot_buffer;
  RectZone            active_zone;
  UartControlChannel  uart_channel;

  if (LoadZoneFromFile(coco_config::kZoneConfigPath, &active_zone)) {
    printf("[ZONE] Loaded zone: (%d,%d)-(%d,%d)\n",
           active_zone.x1, active_zone.y1, active_zone.x2, active_zone.y2);
  } else {
    printf("[ZONE] No zone config found, detections will run without zone filtering\n");
  }

  if (coco_config::kEnableSerialSetup) {
    if (!uart_channel.Initialize(115200)) {
      gpio_alarm.Release();
      detector.Release();
      processor.Release();
      visualizer.Release();
      if (ssne_release()) {
        fprintf(stderr, "SSNE release failed!\n");
        return -1;
      }
      return -1;
    }

    if (!RunSerialSetup(&uart_channel, &processor, crop_shape, &active_zone)) {
      uart_channel.Release();
      gpio_alarm.Release();
      detector.Release();
      processor.Release();
      visualizer.Release();
      if (ssne_release()) {
        fprintf(stderr, "SSNE release failed!\n");
        return -1;
      }
      return 0;
    }
  }

  std::thread listener_thread(keyboard_listener);
  SnapshotHttpServer snapshot_server(coco_config::kSnapshotHttpPort, &snapshot_buffer);
  std::thread snapshot_thread(&SnapshotHttpServer::Run, &snapshot_server);

  auto last_log_time      = std::chrono::steady_clock::now();
  auto last_snapshot_time = std::chrono::steady_clock::now() -
                            std::chrono::milliseconds(coco_config::kSnapshotUpdateIntervalMs);
  auto fps_window_start   = std::chrono::steady_clock::now();
  constexpr int kDetLogIntervalMs   = 500;
  constexpr int kIdleLogIntervalMs  = 5000;
  constexpr int kFpsLogIntervalMs   = 1000;
  constexpr float kSensorFps        = 60.0f;
  constexpr int kCamFailMax         = 60;  // ~1s of consecutive camera failures
  constexpr int kInferFailMax       = 30;  // ~0.5s of consecutive inference failures

  int fps_frame_count   = 0;
  int cam_fail_count    = 0;
  int infer_fail_count  = 0;

  while (!check_exit_flag()) {
    const auto now = std::chrono::steady_clock::now();

    // --- 摄像头异常处理 ---
    if (!processor.GetImage(&img_sensor)) {
      ++cam_fail_count;
      if (cam_fail_count == 1) {
        fprintf(stderr, "[CAM][ALARM] Camera frame acquisition failed\n");
      }
      if (cam_fail_count >= kCamFailMax) {
        fprintf(stderr, "[CAM][ALARM] Camera unresponsive for %d frames, attempting pipeline restart\n",
                cam_fail_count);
        processor.Release();
        usleep(200000);
        processor.Initialize(&img_shape);
        cam_fail_count = 0;
      }
      continue;
    }
    cam_fail_count = 0;

    ++fps_frame_count;
    const auto fps_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - fps_window_start).count();
    if (fps_elapsed_ms >= kFpsLogIntervalMs) {
      const float fps_app = fps_frame_count * 1000.0f / static_cast<float>(fps_elapsed_ms);
      const float ratio   = fps_app / kSensorFps;
      printf("[FPS]  app=%.1f  sensor=%.0f  R=%.2f\n", fps_app, kSensorFps, ratio);
      fps_frame_count  = 0;
      fps_window_start = now;
    }

    const auto snapshot_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_snapshot_time).count();
    if (snapshot_elapsed_ms >= coco_config::kSnapshotUpdateIntervalMs) {
      std::vector<unsigned char> pgm = BuildPgmSnapshot(img_sensor, crop_shape);
      {
        std::lock_guard<std::mutex> lock(snapshot_buffer.mutex);
        snapshot_buffer.pgm_bytes = pgm;
        snapshot_buffer.width  = crop_shape[0];
        snapshot_buffer.height = crop_shape[1];
        snapshot_buffer.ready  = true;
      }
      if (!SaveSnapshotToFile(pgm, coco_config::kSnapshotFilePath)) {
        printf("[SNAPSHOT] failed to save %s\n", coco_config::kSnapshotFilePath);
      }
      last_snapshot_time = now;
    }

    // --- 推理异常处理 ---
    if (!detector.Predict(&img_sensor, &det_result, coco_config::kConfThreshold)) {
      ++infer_fail_count;
      fprintf(stderr, "[INFER][ALARM] Inference failed (%d consecutive)\n", infer_fail_count);
      if (infer_fail_count >= kInferFailMax) {
        fprintf(stderr, "[INFER][ALARM] Too many inference failures, skipping OSD this cycle\n");
      }
      visualizer.Draw({});
      continue;
    }
    infer_fail_count = 0;

    FilterDetectionsByZone(&det_result, active_zone);
    ConvertCropBoxesToOriginal(&det_result);

    tracker.Update(det_result);
    CocoDetectionResult stable = tracker.ConfirmedDetections();

    const bool has_object = !stable.detections.empty();
    gpio_alarm.Update(has_object);

    const auto log_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_log_time).count();
    const int log_interval = has_object ? kDetLogIntervalMs : kIdleLogIntervalMs;
    if (log_elapsed_ms >= log_interval) {
      if (has_object) {
        for (const auto& det : stable.detections) {
          printf("[DET]  %-10s  conf=%.2f  [%.0f,%.0f,%.0f,%.0f]\n",
                 det.label.c_str(), det.score,
                 det.box_xyxy[0], det.box_xyxy[1], det.box_xyxy[2], det.box_xyxy[3]);
        }
      } else {
        printf("[IDLE] no detection\n");
      }
      last_log_time = now;
    }

    visualizer.Draw(tracker.ConfirmedBoxes());
  }

  if (listener_thread.joinable()) {
    listener_thread.join();
  }
  if (snapshot_thread.joinable()) {
    snapshot_thread.join();
  }

  if (coco_config::kEnableSerialSetup) {
    uart_channel.Release();
  }
  gpio_alarm.Release();
  detector.Release();
  processor.Release();
  visualizer.Release();

  if (ssne_release()) {
    fprintf(stderr, "SSNE release failed!\n");
    return -1;
  }

  return 0;
}
