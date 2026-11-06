// Copyright 2026 so_arm101 authors. MIT License.
//
// termios による最小のシリアルポート。POSIX のみ、追加依存なし。
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace so_arm101_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// 8N1 / raw で開く。TIOCEXCL で排他にする（他プロセスが同じポートを開けなくなる）。
  /// baud は 1000000 など Linux の標準レートのみ。失敗したら false と err に理由。
  bool open(const std::string & path, int baud, std::string * err);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// 全バイトを送る。失敗したら false。
  bool write_all(const uint8_t * data, size_t len);

  /// ちょうど n バイトを timeout 以内に読む。読めたバイト数を返す（n 未満ならタイムアウトかエラー）。
  size_t read_exact(uint8_t * buf, size_t n, std::chrono::microseconds timeout);

  /// 受信バッファを捨てる（再同期用）。
  void flush_input();

private:
  int fd_{-1};
};

}  // namespace so_arm101_hardware
