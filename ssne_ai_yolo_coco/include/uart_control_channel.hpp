#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "uart_api.h"

class UartControlChannel {
 public:
  UartControlChannel();
  ~UartControlChannel();

  bool Initialize(uint32_t baudrate);
  void Release();

  bool IsOpen() const;
  bool SendTextLine(const std::string& line);
  bool SendBytes(const uint8_t* data, size_t len);
  bool ReceiveLine(std::string* out_line, int timeout_ms);

 private:
  enum {
    kFifoBytes = 32,
    kPollIntervalUs = 2000,
    kMaxLineBytes = 4096,
    kTxChunkGapUs = 100,
  };

  uart_handle_t handle_;
  std::string rx_line_buffer_;
};
