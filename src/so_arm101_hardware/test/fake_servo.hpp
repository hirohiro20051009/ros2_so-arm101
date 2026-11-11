// Copyright 2026 so_arm101 authors. MIT License.
//
// 試験用の偽 STS3215（pty のマスタ側）。実機・実ポートは開かない。
//   - READ / SYNC_READ / WRITE / SYNC_WRITE / PING に応答する。レジスタはサーボごとのメモリ。
//   - 受けた命令と書き込み（命令・ID・アドレス・データ）を順番に記録する（書き込み順の検査用）。
//   - 電源投入直後の状態（2b-3a の実機確認で得た値）から始める: EEPROM は LeRobot の設定済み、
//     SRAM は Acceleration=0・Lock=1・Torque_Enable=0・Torque_Limit=Max_Torque_Limit・Goal_Velocity=0、
//     Goal_Position=0（実機では不明なので、わざと 0 にして「古い目標」で動かないことを確かめる）。
//   - EEPROM（アドレス < 40）への書き込みは Lock=1 のとき無視する（実機の仕様の想定）。
#pragma once

#include <pty.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace fake
{

struct WriteRecord
{
  uint8_t inst;  // 0x03 / 0x83
  uint8_t id;
  uint8_t addr;
  std::vector<uint8_t> data;
};

class FakeServos
{
public:
  /// ids は {1..6}。gripper_id のサーボだけトルク系を gripper 用の値にする。
  FakeServos(std::vector<uint8_t> ids, uint8_t gripper_id)
  : ids_(ids)
  {
    for (auto id : ids) {
      mem_[id] = {};
      power_on(id, id == gripper_id);
      is_gripper_[id] = (id == gripper_id);
    }
    termios tio{};
    cfmakeraw(&tio);
    openpty(&master_, &slave_, nullptr, &tio, nullptr);
    char name[128];
    ttyname_r(slave_, name, sizeof(name));
    path_ = name;
    thread_ = std::thread([this] {loop();});
  }
  ~FakeServos()
  {
    stop_ = true;
    thread_.join();
    ::close(master_);
    ::close(slave_);
  }
  const std::string & path() const {return path_;}

  // --- 状態の操作・参照 ---
  void set_u8(uint8_t id, uint8_t addr, int v)
  {
    std::lock_guard<std::mutex> g(m_);
    mem_[id][addr] = v & 0xFF;
  }
  void set_u16(uint8_t id, uint8_t addr, int v)
  {
    std::lock_guard<std::mutex> g(m_);
    mem_[id][addr] = v & 0xFF;
    mem_[id][addr + 1] = (v >> 8) & 0xFF;
  }
  int u8(uint8_t id, uint8_t addr)
  {
    std::lock_guard<std::mutex> g(m_);
    return mem_[id][addr];
  }
  int u16(uint8_t id, uint8_t addr)
  {
    std::lock_guard<std::mutex> g(m_);
    return mem_[id][addr] | (mem_[id][addr + 1] << 8);
  }
  /// Present_Position（sign-magnitude bit15）を設定。
  void set_present(uint8_t id, int raw) {set_u16(id, 56, raw < 0 ? ((1 << 15) | -raw) : raw);}
  int goal_position(uint8_t id) {return u16(id, 42);}

  /// サーボの電源を入れ直した状態にする（SRAM が電源投入時の値に戻る。Present_Position は残す）。
  void power_cycle()
  {
    std::lock_guard<std::mutex> g(m_);
    for (auto id : ids_) {sram_defaults(id);}
    ++power_cycles_;
  }
  /// EEPROM の値を変える（診断・apply_servo_config の試験用）。
  void set_eeprom(uint8_t id, uint8_t addr, int v, int len = 1)
  {
    if (len == 2) {set_u16(id, addr, v);} else {set_u8(id, addr, v);}
  }

  // --- 挙動の指定 ---
  /// このアドレスへの書き込みは応答するが無視する（Acceleration が 0 のままになる状況の再現）。
  void ignore_writes_to(uint8_t addr)
  {
    std::lock_guard<std::mutex> g(m_);
    ignored_addr_.insert(addr);
  }
  /// Lock=1 のとき SRAM への書き込みも拒否する（Lock=1 の確認 1 が失敗する状況の再現）。
  void lock_blocks_sram(bool v) {lock_blocks_sram_ = v;}
  void set_silent(bool v) {silent_ = v;}
  /// 通信エラーの再現: (id, addr) への READ の応答を、次の count 回だけ壊す（チェックサムを 1 ビット反転）。
  void corrupt_read_replies(uint8_t id, uint8_t addr, int count)
  {
    std::lock_guard<std::mutex> g(m_);
    corrupt_[{id, addr}] = count;
  }
  /// (id, addr) への READ に、次の count 回だけ応答しない（タイムアウト）。
  void drop_read_replies(uint8_t id, uint8_t addr, int count)
  {
    std::lock_guard<std::mutex> g(m_);
    drop_[{id, addr}] = count;
  }
  /// この ID は全ての応答の ERR バイトに値を入れる（0 で解除）。
  void set_err(uint8_t id, uint8_t err)
  {
    std::lock_guard<std::mutex> g(m_);
    err_[id] = err;
  }

  std::vector<WriteRecord> writes()
  {
    std::lock_guard<std::mutex> g(m_);
    return writes_;
  }
  void clear_writes()
  {
    std::lock_guard<std::mutex> g(m_);
    writes_.clear();
  }
  /// 受けた命令コードの回数。
  std::map<uint8_t, unsigned> instruction_counts()
  {
    std::lock_guard<std::mutex> g(m_);
    return counts_;
  }
  /// 要求パケットの検査（チェックサム不良の個数）。
  unsigned bad_requests()
  {
    std::lock_guard<std::mutex> g(m_);
    return bad_requests_;
  }

private:
  void put(uint8_t id, uint8_t addr, int v, int n)
  {
    for (int i = 0; i < n; ++i) {mem_[id][addr + i] = (v >> (8 * i)) & 0xFF;}
  }
  void sram_defaults(uint8_t id)
  {
    put(id, 40, 0, 1);     // Torque_Enable
    put(id, 41, 0, 1);     // Acceleration（電源投入時は 0）
    put(id, 42, 0, 2);     // Goal_Position（不明。わざと 0）
    put(id, 46, 0, 2);     // Goal_Velocity
    put(id, 48, mem_[id][16] | (mem_[id][17] << 8), 2);  // Torque_Limit = Max_Torque_Limit
    put(id, 55, 1, 1);     // Lock（電源投入時は 1）
  }
  void power_on(uint8_t id, bool gripper)
  {
    put(id, 3, 777, 2);
    put(id, 7, 0, 1);
    put(id, 16, gripper ? 500 : 1000, 2);
    put(id, 21, 16, 1);
    put(id, 22, 32, 1);
    put(id, 23, 0, 1);
    put(id, 28, gripper ? 250 : 500, 2);
    put(id, 33, 0, 1);
    put(id, 36, gripper ? 25 : 80, 1);
    put(id, 62, 119, 1);
    put(id, 63, 30, 1);
    put(id, 65, 0, 1);
    put(id, 85, 254, 1);
    sram_defaults(id);
  }
  void loop()
  {
    std::vector<uint8_t> buf;
    while (!stop_) {
      pollfd p{master_, POLLIN, 0};
      if (::poll(&p, 1, 20) <= 0) {continue;}
      uint8_t tmp[256];
      const ssize_t n = ::read(master_, tmp, sizeof(tmp));
      if (n <= 0) {continue;}
      buf.insert(buf.end(), tmp, tmp + n);
      while (parse(&buf)) {}
    }
  }
  bool parse(std::vector<uint8_t> * b)
  {
    while (b->size() >= 2 && !((*b)[0] == 0xFF && (*b)[1] == 0xFF)) {b->erase(b->begin());}
    if (b->size() < 4) {return false;}
    const size_t total = (*b)[3] + 4u;
    if (b->size() < total) {return false;}
    std::vector<uint8_t> pkt(b->begin(), b->begin() + total);
    b->erase(b->begin(), b->begin() + total);
    handle(pkt);
    return true;
  }
  static uint8_t chk(const std::vector<uint8_t> & body)
  {
    unsigned s = 0;
    for (auto c : body) {s += c;}
    return static_cast<uint8_t>(~s & 0xFF);
  }
  void reply(uint8_t id, const std::vector<uint8_t> & data, bool corrupt = false)
  {
    if (silent_) {return;}
    uint8_t err;
    {
      std::lock_guard<std::mutex> g(m_);
      err = err_.count(id) ? err_[id] : 0;
    }
    std::vector<uint8_t> body{id, static_cast<uint8_t>(data.size() + 2), err};
    body.insert(body.end(), data.begin(), data.end());
    std::vector<uint8_t> out{0xFF, 0xFF};
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(static_cast<uint8_t>(chk(body) ^ (corrupt ? 0x01 : 0x00)));
    ::write(master_, out.data(), out.size());
  }
  void do_write(uint8_t inst, uint8_t id, uint8_t addr, const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> g(m_);
    writes_.push_back({inst, id, addr, data});
    if (!mem_.count(id)) {return;}
    const bool locked = mem_[id][55] == 1;
    for (size_t i = 0; i < data.size(); ++i) {
      const uint8_t a = addr + i;
      if (ignored_addr_.count(a)) {continue;}
      if (a < 40 && locked) {continue;}                              // EEPROM は Lock=1 で保護
      if (a >= 40 && locked && lock_blocks_sram_ && a != 55) {continue;}  // 想定外ケース: Lock=1 で SRAM も拒否
      mem_[id][a] = data[i];
    }
  }
  void handle(const std::vector<uint8_t> & pkt)
  {
    const uint8_t id = pkt[2], inst = pkt[4];
    {
      std::vector<uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
      std::lock_guard<std::mutex> g(m_);
      ++counts_[inst];
      if (chk(body) != pkt.back()) {++bad_requests_;}
    }
    if (inst == 0x01) {
      if (mem_.count(id)) {reply(id, {});}
    } else if (inst == 0x02) {
      read_reply(id, pkt[5], pkt[6]);
    } else if (inst == 0x82) {
      for (size_t i = 7; i < pkt.size() - 1; ++i) {read_reply(pkt[i], pkt[5], pkt[6]);}
    } else if (inst == 0x03) {
      do_write(inst, id, pkt[5], std::vector<uint8_t>(pkt.begin() + 6, pkt.end() - 1));
      if (mem_.count(id)) {reply(id, {});}
    } else if (inst == 0x83) {
      const uint8_t addr = pkt[5], len = pkt[6];
      for (size_t i = 7; i + len < pkt.size(); i += len + 1u) {
        do_write(inst, pkt[i], addr, std::vector<uint8_t>(pkt.begin() + i + 1, pkt.begin() + i + 1 + len));
      }
    }
  }
  void read_reply(uint8_t id, uint8_t addr, uint8_t len)
  {
    std::vector<uint8_t> data;
    bool corrupt = false;
    {
      std::lock_guard<std::mutex> g(m_);
      if (!mem_.count(id)) {return;}
      for (int i = 0; i < len; ++i) {data.push_back(mem_[id][addr + i]);}
      auto d = drop_.find({id, addr});
      if (d != drop_.end() && d->second > 0) {--d->second; return;}
      auto c = corrupt_.find({id, addr});
      if (c != corrupt_.end() && c->second > 0) {--c->second; corrupt = true;}
    }
    reply(id, data, corrupt);
  }

  std::vector<uint8_t> ids_;
  std::map<uint8_t, std::array<uint8_t, 256>> mem_;
  std::map<uint8_t, bool> is_gripper_;
  std::map<uint8_t, uint8_t> err_;
  std::map<std::pair<uint8_t, uint8_t>, int> corrupt_, drop_;
  std::set<uint8_t> ignored_addr_;
  std::vector<WriteRecord> writes_;
  std::map<uint8_t, unsigned> counts_;
  unsigned bad_requests_{0};
  unsigned power_cycles_{0};
  std::atomic<bool> lock_blocks_sram_{false}, silent_{false}, stop_{false};
  std::mutex m_;
  int master_{-1}, slave_{-1};
  std::string path_;
  std::thread thread_;
};

}  // namespace fake
