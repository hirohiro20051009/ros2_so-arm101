// Copyright 2026 so_arm101 authors. MIT License.
//
// termios による最小のシリアルポート。POSIX のみ、追加依存なし。
#pragma once

#include <chrono>
#include <cstddef>
#include <atomic>
#include <cstdint>
#include <mutex>
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

  /// バスの「1 往復」（要求の送信から応答の受信まで）をポート単位で排他にする RAII。
  /// ポートを使う処理は必ずこれを持つ。別スレッドが同時にポートを使おうとしても、後から来た方は
  /// 先の往復が終わるまで待つので、要求や応答のバイトが混ざらない。同じスレッドからの入れ子は可
  /// （再入可能）。待たされた回数は contention_count() に数える（ドライバは 0 のはず）。
  class Transaction
  {
public:
    explicit Transaction(SerialPort & p)
    : p_(p)
    {
      if (!p_.mu_.try_lock()) {
        p_.contention_.fetch_add(1);
        p_.mu_.lock();
      }
    }
    ~Transaction() {p_.mu_.unlock();}
    Transaction(const Transaction &) = delete;
    Transaction & operator=(const Transaction &) = delete;

private:
    SerialPort & p_;
  };

  /// 別の往復の最中にポートを使おうとして待たされた回数。
  unsigned long contention_count() const {return contention_.load();}

private:
  int fd_{-1};
  std::recursive_mutex mu_;
  std::atomic<unsigned long> contention_{0};
};

}  // namespace so_arm101_hardware
