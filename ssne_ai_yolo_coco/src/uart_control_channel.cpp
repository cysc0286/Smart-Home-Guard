#include "../include/uart_control_channel.hpp"

#include <algorithm>
#include <chrono>
#include <stdio.h>
#include <unistd.h>

UartControlChannel::UartControlChannel()
    : handle_(NULL), rx_line_buffer_() {}

UartControlChannel::~UartControlChannel() {
  Release();
}

bool UartControlChannel::Initialize(uint32_t baudrate) {
  if (handle_ != NULL) {
    return true;
  }

  handle_ = uart_init();
  if (handle_ == NULL) {
    fprintf(stderr, "[UART] uart_init failed. Check uart_kmod.ko and device permissions.\n");
    return false;
  }

  uart_set_baudrate(handle_, UART_TX0, baudrate);
  uart_set_baudrate(handle_, UART_RX0, baudrate);
  uart_set_parity(handle_, UART_TX0, UART_PARITY_NONE);
  uart_set_parity(handle_, UART_RX0, UART_PARITY_NONE);
  rx_line_buffer_.clear();

  printf("[UART] Ready at %u baud, 8N1, TX0=GPIO0, RX0=GPIO2\n", baudrate);
  return true;
}

void UartControlChannel::Release() {
  if (handle_ != NULL) {
    uart_close(handle_);
    handle_ = NULL;
  }
  rx_line_buffer_.clear();
}

bool UartControlChannel::IsOpen() const {
  return handle_ != NULL;
}

bool UartControlChannel::SendTextLine(const std::string& line) {
  std::string payload = line;
  payload.push_back('\n');
  return SendBytes(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
}

bool UartControlChannel::SendBytes(const uint8_t* data, size_t len) {
  if (handle_ == NULL || (data == NULL && len > 0)) {
    return false;
  }

  size_t offset = 0;
  while (offset < len) {
    const uint32_t chunk = static_cast<uint32_t>(
        std::min<size_t>(len - offset, static_cast<size_t>(kFifoBytes)));
    uart_send_data(handle_, UART_TX0, const_cast<uint8_t*>(data + offset), chunk);
    offset += chunk;
  }
  return true;
}

bool UartControlChannel::ReceiveLine(std::string* out_line, int timeout_ms) {
  if (handle_ == NULL || out_line == NULL) {
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  while (true) {
    uint8_t buffer[kFifoBytes] = {0};
    uint32_t actual_len = 0;
    uart_receive_data(handle_, UART_RX0, buffer, sizeof(buffer), &actual_len);
    if (actual_len > 0) {
      for (uint32_t i = 0; i < actual_len; ++i) {
        const char ch = static_cast<char>(buffer[i]);
        if (ch == '\r') {
          continue;
        }
        if (ch == '\n') {
          *out_line = rx_line_buffer_;
          rx_line_buffer_.clear();
          return true;
        }
        if (rx_line_buffer_.size() < static_cast<size_t>(kMaxLineBytes)) {
          rx_line_buffer_.push_back(ch);
        } else {
          rx_line_buffer_.clear();
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    if (elapsed_ms >= timeout_ms) {
      return false;
    }
    usleep(kPollIntervalUs);
  }
}
