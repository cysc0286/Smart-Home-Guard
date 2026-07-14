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
    // 32 bytes take about 2.78 ms on a 115200-baud 8N1 link.  The UART API
    // writes into a 32-byte FIFO and requires callers to wait for that FIFO
    // to drain before submitting the next chunk.  A 20-us gap can lose or
    // strand the final FIFO contents, which corrupts SNAPSHOT/ACK boundaries.
    kTxChunkGapUs = 3500,
  };

  uart_handle_t handle_;
  std::string rx_line_buffer_;
};
