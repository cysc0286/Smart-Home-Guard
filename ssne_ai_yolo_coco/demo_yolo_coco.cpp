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
#include <cctype>
#include <cstdlib>

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

void ConvertCropBoxesToOriginal(std::vector<std::array<float, 4>>* boxes) {
  for (auto& box : *boxes) {
    box[0] += static_cast<float>(coco_config::kCropOffsetX);
    box[2] += static_cast<float>(coco_config::kCropOffsetX);
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
    if (is_alarm_class && inside) {
      alarm_boxes->push_back(det.box_xyxy);
    } else {
      normal_boxes->push_back(det.box_xyxy);
    }
  }
}

// Zone points are stored in 1440x1080 crop coordinates. Shift x before OSD,
// because the OSD layer uses the full 1920x1080 image coordinates.
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
    visualizer->DrawZonePolygonBBox(pts);
  } else {
    visualizer->DrawZoneRect(zone.X1() + coco_config::kCropOffsetX,
                             zone.Y1(),
                             zone.X2() + coco_config::kCropOffsetX,
                             zone.Y2());
  }
}

void FilterDetectionsByAlarmClasses(CocoDetectionResult* result, const GuardZone& zone) {
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

std::vector<unsigned char> BuildPgmSnapshot(const ssne_tensor_t& img_sensor,
                                            const std::array<int, 2>& crop_shape) {
  const int width = crop_shape[0];
  const int height = crop_shape[1];
  const unsigned char* src =
      static_cast<const unsigned char*>(get_data(const_cast<ssne_tensor_t&>(img_sensor)));

  std::string header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  std::vector<unsigned char> pgm(header.begin(), header.end());
  pgm.reserve(header.size() + width * height);

  // SSNE_YUV422_16 is packed as UYVY on this pipeline; luma is the odd byte.
  for (int i = 0; i < width * height; ++i) {
    pgm.push_back(src[i * 2 + 1]);
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

      // UYVY byte order: U0 Y0 V0 Y1. Adjacent pixels share U/V.
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
  return uart->SendBytes(reinterpret_cast<const uint8_t*>(header.data()), header.size()) &&
         uart->SendBytes(preview.data(), preview.size());
}

bool RunSerialSetup(UartControlChannel* uart,
                    IMAGEPROCESSOR* processor,
                    const std::array<int, 2>& crop_shape,
                    GuardZone* zone,
                    VISUALIZER* visualizer) {
  if (LoadZoneFromFile(coco_config::kZoneConfigPath, zone)) {
    printf("[SETUP] Loaded existing zone: %s\n", zone->Describe().c_str());
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
      if (!SendSerialSnapshot(uart, processor, crop_shape)) {
        fprintf(stderr, "[SETUP] Failed to send snapshot over UART\n");
        return false;
      }
    } else if (line.rfind("ZONE ", 0) == 0) {
      ApplyZoneCommand(uart, line.substr(5), zone, visualizer);
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
                       VISUALIZER* visualizer) {
  if (uart == nullptr || !uart->IsOpen()) {
    return;
  }
  std::string line;
  int handled = 0;
  while (handled < 4 && uart->ReceiveLine(&line, 0)) {
    ++handled;
    if (line == "SNAPSHOT") {
      if (!SendSerialSnapshot(uart, processor, crop_shape)) {
        fprintf(stderr, "[UART] Failed to send runtime snapshot\n");
      }
    } else if (line.rfind("ZONE ", 0) == 0) {
      ApplyZoneCommand(uart, line.substr(5), zone, visualizer);
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
  // 切换到 colorLUT.sscl（21 RGB 条目）以支持白/红/黄三色 OSD 显示
  visualizer.Initialize(img_shape, "colorLUT.sscl");

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
  GuardZone           active_zone;
  UartControlChannel  uart_channel;

  if (LoadZoneFromFile(coco_config::kZoneConfigPath, &active_zone)) {
    printf("[ZONE] Loaded zone: %s\n", active_zone.Describe().c_str());
  } else {
    printf("[ZONE] No zone config found, detections will run without zone filtering\n");
  }

  if (coco_config::kEnableSerialSetup) {
    if (!uart_channel.Initialize(coco_config::kSerialBaudrate)) {
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

    if (!RunSerialSetup(&uart_channel, &processor, crop_shape, &active_zone, &visualizer)) {
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

  // 启动后立即把已加载的 zone 绘制为黄色框
  RefreshZoneOverlay(&visualizer, active_zone);

  std::thread listener_thread(keyboard_listener);
  SnapshotHttpServer snapshot_server(coco_config::kSnapshotHttpPort, &snapshot_buffer);
  std::thread snapshot_thread(&SnapshotHttpServer::Run, &snapshot_server);

  auto last_log_time      = std::chrono::steady_clock::now();
  auto last_snapshot_time = std::chrono::steady_clock::now() -
                            std::chrono::milliseconds(coco_config::kSnapshotUpdateIntervalMs);
  auto fps_window_start   = std::chrono::steady_clock::now();
  auto last_brightness_log = std::chrono::steady_clock::now();
  constexpr int kDetLogIntervalMs    = 500;
  constexpr int kIdleLogIntervalMs   = 5000;
  constexpr int kFpsLogIntervalMs    = 1000;
  constexpr int kBrightnessLogMs     = 5000;
  constexpr int kLatencyReportEveryN = 60;   // 每 60 帧上报一次 P95 延迟
  constexpr float kSensorFps         = 60.0f;
  constexpr int kCamFailMax          = 60;   // ~1s of consecutive camera failures
  constexpr int kInferFailMax        = 30;   // ~0.5s of consecutive inference failures
  constexpr int kDataFailMax         = 30;   // 连续数据异常阈值

  int fps_frame_count   = 0;
  int cam_fail_count    = 0;
  int infer_fail_count  = 0;
  int data_fail_count   = 0;

  // 端到端延迟样本环（帧捕获 -> OSD刷新），用于 P95 统计
  std::vector<long long> latency_samples;
  latency_samples.reserve(kLatencyReportEveryN);

  while (!check_exit_flag()) {
    const auto loop_start = std::chrono::steady_clock::now();
    const auto now = loop_start;

    // --- [异常类1] 摄像头异常处理 ---
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

    // --- [异常类2] 数据异常处理 ---
    // 校验张量数据指针、维度合法性
    {
      void* data_ptr = get_data(img_sensor);
      const bool data_invalid = (data_ptr == nullptr);
      if (data_invalid) {
        ++data_fail_count;
        if (data_fail_count == 1) {
          fprintf(stderr, "[DATA][ALARM] Invalid sensor tensor (null data)\n");
        }
        if (data_fail_count >= kDataFailMax) {
          fprintf(stderr, "[DATA][ALARM] Persistent data corruption, restarting pipeline\n");
          processor.Release();
          usleep(200000);
          processor.Initialize(&img_shape);
          data_fail_count = 0;
        }
        continue;
      }
      data_fail_count = 0;

      // 亮度统计 (鲁棒性): 采样 UYVY 中的 Y 通道
      const auto bright_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_brightness_log).count();
      if (bright_elapsed_ms >= kBrightnessLogMs) {
        const unsigned char* src = static_cast<const unsigned char*>(data_ptr);
        const int sample_count = 256;
        long sum = 0;
        const int total_pixels = crop_shape[0] * crop_shape[1];
        for (int i = 0; i < sample_count; ++i) {
          const int p = (i * total_pixels) / sample_count;
          sum += src[p * 2 + 1];
        }
        const int avg_y = static_cast<int>(sum / sample_count);
        const char* level = avg_y < 40 ? "DARK" : (avg_y > 200 ? "BRIGHT" : "OK");
        printf("[ENV]  avg_luma=%d  level=%s\n", avg_y, level);
        last_brightness_log = now;
      }
    }

    if (coco_config::kEnableSerialSetup) {
      PollRuntimeSerial(&uart_channel, &processor, crop_shape, &active_zone, &visualizer);
    }

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

    // --- [异常类3] 推理异常处理 ---
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

    // Keep detections in crop coordinates through tracking and zone judgement.
    // The PC planner also sends zone coordinates in the 1440x1080 crop space.
    FilterDetectionsByAlarmClasses(&det_result, active_zone);

    tracker.Update(det_result);
    CocoDetectionResult stable_crop = tracker.ConfirmedDetections();

    // Zone judgement is done in crop coordinates; display/log coordinates are
    // shifted back to the full 1920x1080 OSD coordinate space afterwards.
    std::vector<std::array<float, 4>> normal_boxes;
    std::vector<std::array<float, 4>> alarm_boxes;
    ClassifyDetections(stable_crop, active_zone, &normal_boxes, &alarm_boxes);
    ConvertCropBoxesToOriginal(&normal_boxes);
    ConvertCropBoxesToOriginal(&alarm_boxes);

    CocoDetectionResult stable_display = stable_crop;
    ConvertCropBoxesToOriginal(&stable_display);

    const bool has_object       = !stable_crop.detections.empty();
    const bool is_alarm_active  = !alarm_boxes.empty();

    // GPIO 报警仅在 zone 内触发（更准确反映安防意图）
    gpio_alarm.Update(is_alarm_active);

    // 显示/隐藏英文 ALERT 报警位图
    if (is_alarm_active) {
      visualizer.ShowAlarmIndicator(coco_config::kAlarmBitmapPosX, coco_config::kAlarmBitmapPosY);
    } else {
      visualizer.HideAlarmIndicator();
    }

    const auto log_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_log_time).count();
    const int log_interval = has_object ? kDetLogIntervalMs : kIdleLogIntervalMs;
    if (log_elapsed_ms >= log_interval) {
      if (has_object) {
        for (const auto& det : stable_display.detections) {
          printf("[DET]  %-10s  conf=%.2f  [%.0f,%.0f,%.0f,%.0f]\n",
                 det.label.c_str(), det.score,
                 det.box_xyxy[0], det.box_xyxy[1], det.box_xyxy[2], det.box_xyxy[3]);
        }
        if (is_alarm_active) {
          printf("[ALARM] %zu object(s) inside danger zone\n", alarm_boxes.size());
        }
      } else {
        printf("[IDLE] no detection\n");
      }
      last_log_time = now;
    }

    visualizer.DrawDetections(normal_boxes, alarm_boxes);

    // 端到端延迟统计 (帧捕获 -> 绘制完成)
    const auto loop_end = std::chrono::steady_clock::now();
    const long long latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        loop_end - loop_start).count();
    latency_samples.push_back(latency_ms);
    if (static_cast<int>(latency_samples.size()) >= kLatencyReportEveryN) {
      std::vector<long long> sorted = latency_samples;
      std::sort(sorted.begin(), sorted.end());
      const long long p50 = sorted[sorted.size() * 50 / 100];
      const long long p95 = sorted[sorted.size() * 95 / 100];
      const long long p99 = sorted[sorted.size() * 99 / 100];
      printf("[LAT]  p50=%lldms  p95=%lldms  p99=%lldms  (n=%zu)\n",
             p50, p95, p99, sorted.size());
      latency_samples.clear();
    }
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
