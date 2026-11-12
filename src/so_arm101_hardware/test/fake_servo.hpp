// Copyright 2026 so_arm101 authors. MIT License.
//
// 試験用の偽 STS3215（pty のマスタ側）。実機・実ポートは開かない。
//   - READ / SYNC_READ / WRITE / SYNC_WRITE / PING に応答する。レジスタはサーボごとのメモリ。
//   - 受けた命令と書き込み（命令・ID・アドレス・データ）を順番に記録する（書き込み順の検査用）。
//   - 電源投入直後の状態（2b-3a の実機確認で得た値）から始める: EEPROM は基準の設定済み（LeRobot 0.4.2 の値）、
//     SRAM は Acceleration=0・Lock=1・Torque_Enable=0・Torque_Limit=Max_Torque_Limit・Goal_Velocity=0、
//     Goal_Position=0（実機では不明なので、わざと 0 にして「古い目標」で動かないことを確かめる）。
//   - EEPROM（アドレス < 40）への書き込みは Lock=1 のとき無視する（実機の仕様の想定）。
#pragma once

#include <pty.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
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
  /// この (id, addr) への WRITE を無視する（応答はする）。id 指定版（キャリブの読み戻し不一致の再現）。
  void ignore_writes_to(uint8_t id, uint8_t addr)
  {
    std::lock_guard<std::mutex> g(m_);
    ignored_id_addr_.insert({id, addr});
  }
  /// この ID への Lock=1 の書き込みだけを無視する（Lock=0 は通る。Lock=1 を確認できない状況の再現）。
  void ignore_lock_one(uint8_t id)
  {
    std::lock_guard<std::mutex> g(m_);
    ignore_lock1_.insert(id);
  }
  /// EEPROM（アドレス < 40）への書き込みが読み出しに反映されるまでの遅れ（ms）。0 で即時。
  void eeprom_commit_delay_ms(int ms)
  {
    std::lock_guard<std::mutex> g(m_);
    commit_delay_ms_ = ms;
  }
  /// (id, addr) への WRITE の応答を、次の count 回だけ捨てる（書き込み自体は行う。通信エラーの再現）。
  void drop_write_replies(uint8_t id, uint8_t addr, int count)
  {
    std::lock_guard<std::mutex> g(m_);
    drop_write_[{id, addr}] = count;
  }
  /// (id, addr) への WRITE の応答を、次の count 回だけ壊す（書き込み自体は行う）。
  void corrupt_write_replies(uint8_t id, uint8_t addr, int count)
  {
    std::lock_guard<std::mutex> g(m_);
    corrupt_write_[{id, addr}] = count;
  }
  /// n 回目の WRITE の後、サーボが落ちた状態にする（n+1 回目以降の WRITE は反映されず、全ての応答が止まる。
  /// 書き込みの途中での電源断・通信断の再現）。restore_power() で戻す。
  void cut_power_after_writes(int n)
  {
    std::lock_guard<std::mutex> g(m_);
    cut_after_ = n;
    applied_writes_ = 0;
  }
  void restore_power()
  {
    std::lock_guard<std::mutex> g(m_);
    cut_after_ = -1;
    applied_writes_ = 0;
    silent_ = false;
  }
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
  /// (id, addr) への次の count 回の READ に、サーボの実際の値ではなく value（リトルエンディアン）を返す
  /// （古い応答・化けた値が「正しいチェックサムの応答」として届く状況の再現。実際の値は変えない）。
  void override_next_reads(uint8_t id, uint8_t addr, int count, int value)
  {
    std::lock_guard<std::mutex> g(m_);
    override_[{id, addr}] = {count, value};
  }
  /// この ID は全ての応答の ERR バイトに値を入れる（0 で解除）。
  void set_err(uint8_t id, uint8_t err)
  {
    std::lock_guard<std::mutex> g(m_);
    err_[id] = err;
  }

  // --- 応答のタイミング（試験用。ドライバは変えない）---
  /// この ID の全応答を、要求を受けた時刻から us だけ遅らせる（既定 0）。
  void set_reply_delay_us(uint8_t id, int us)
  {
    std::lock_guard<std::mutex> g(m_);
    reply_delay_us_[id] = us;
  }
  /// SYNC_READ の応答を us 間隔で並べる（k 番目の応答は k*us 遅れる。既定 0）。
  void set_frame_spacing_us(int us)
  {
    std::lock_guard<std::mutex> g(m_);
    frame_spacing_us_ = us;
  }
  /// (id, addr) への次の count 回の READ 応答だけを us 遅らせる（タイムアウトより遅れて着く応答の再現）。
  void delay_next_replies(uint8_t id, uint8_t addr, int count, int us)
  {
    std::lock_guard<std::mutex> g(m_);
    delay_next_[{id, addr}] = {count, us};
  }
  struct ReplyLogEntry
  {
    std::chrono::steady_clock::time_point at;  // write した時刻
    uint8_t id;
    uint8_t addr;   // READ の応答のときだけ有効（それ以外は 0xFF）
    bool corrupt;
    size_t bytes;
  };
  std::vector<ReplyLogEntry> reply_log()
  {
    std::lock_guard<std::mutex> g(m_);
    return reply_log_;
  }
  void clear_reply_log()
  {
    std::lock_guard<std::mutex> g(m_);
    reply_log_.clear();
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
    put(id, 5, id, 1);     // ID
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
      flush_due();
      pollfd p{master_, POLLIN, 0};
      if (::poll(&p, 1, next_wait_ms()) <= 0) {continue;}
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
  /// 応答をキューに入れる。due 時刻（要求を受けた時刻 + 遅れ）が来たら loop が write する。
  /// 遅れが 0 のときは、従来どおりその場で write する。
  void reply(
    uint8_t id, const std::vector<uint8_t> & data, bool corrupt = false, int extra_delay_us = 0,
    uint8_t addr = 0xFF, bool in_sync = false)
  {
    if (silent_) {return;}
    uint8_t err;
    int delay_us;
    {
      std::lock_guard<std::mutex> g(m_);
      err = err_.count(id) ? err_[id] : 0;
      delay_us = (reply_delay_us_.count(id) ? reply_delay_us_[id] : 0) + extra_delay_us;
      if (in_sync) {delay_us += frame_spacing_us_ * sync_slot_++;}
    }
    std::vector<uint8_t> body{id, static_cast<uint8_t>(data.size() + 2), err};
    body.insert(body.end(), data.begin(), data.end());
    std::vector<uint8_t> out{0xFF, 0xFF};
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(static_cast<uint8_t>(chk(body) ^ (corrupt ? 0x01 : 0x00)));
    std::lock_guard<std::mutex> g(m_);
    // 1 本のバスなので、応答は追い越さない（先に予約された応答より前には出さない）
    auto due = request_time_ + std::chrono::microseconds(delay_us);
    if (!queue_.empty() && due < last_due_) {due = last_due_;}
    if (delay_us == 0 && queue_.empty()) {
      write_out(id, addr, corrupt, out);
    } else {
      last_due_ = due;
      queue_.push_back({due, seq_++, id, addr, corrupt, out});
    }
  }
  struct Queued
  {
    std::chrono::steady_clock::time_point due;
    uint64_t seq;
    uint8_t id;
    uint8_t addr;
    bool corrupt;
    std::vector<uint8_t> bytes;
  };
  void write_out(uint8_t id, uint8_t addr, bool corrupt, const std::vector<uint8_t> & out)  // m_ を持っていること
  {
    ::write(master_, out.data(), out.size());
    reply_log_.push_back({std::chrono::steady_clock::now(), id, addr, corrupt, out.size()});
  }
  void flush_due()
  {
    std::lock_guard<std::mutex> g(m_);
    while (!queue_.empty()) {
      auto it = std::min_element(
        queue_.begin(), queue_.end(), [](const Queued & a, const Queued & b) {
          return a.due != b.due ? a.due < b.due : a.seq < b.seq;
        });
      if (it->due > std::chrono::steady_clock::now()) {break;}
      write_out(it->id, it->addr, it->corrupt, it->bytes);
      queue_.erase(it);
    }
  }
  int next_wait_ms()
  {
    std::lock_guard<std::mutex> g(m_);
    if (queue_.empty()) {return 20;}
    auto d = queue_.front().due;
    for (const auto & q : queue_) {d = std::min(d, q.due);}
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(d - std::chrono::steady_clock::now()).count();
    if (us <= 0) {return 0;}
    return static_cast<int>(std::min<long>(20, (us + 999) / 1000));
  }
  /// 戻り値: 0=普通に応答 / 1=応答を捨てる / 2=応答を壊す / 3=応答しない（サーボが落ちている）
  int do_write(uint8_t inst, uint8_t id, uint8_t addr, const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> g(m_);
    apply_pending();
    if (cut_after_ >= 0 && applied_writes_ >= cut_after_) {
      silent_ = true;  // 落ちた後の WRITE は記録もしない（サーボには届いていない）
      return 3;
    }
    ++applied_writes_;
    writes_.push_back({inst, id, addr, data});
    if (!mem_.count(id)) {return 0;}
    const bool locked = mem_[id][55] == 1;
    for (size_t i = 0; i < data.size(); ++i) {
      const uint8_t a = addr + i;
      if (ignored_addr_.count(a)) {continue;}
      if (ignored_id_addr_.count({id, addr})) {continue;}  // 書き込みの先頭アドレスで指定（2 バイト全体を無視）
      if (a == 55 && data[i] == 1 && ignore_lock1_.count(id)) {continue;}
      if (a < 40 && locked) {continue;}                              // EEPROM は Lock=1 で保護
      if (a >= 40 && locked && lock_blocks_sram_ && a != 55) {continue;}  // 想定外ケース: Lock=1 で SRAM も拒否
      if (a < 40 && commit_delay_ms_ > 0) {
        pending_.push_back(
          {std::chrono::steady_clock::now() + std::chrono::milliseconds(commit_delay_ms_), id, a, data[i]});
        continue;
      }
      mem_[id][a] = data[i];
    }
    auto d = drop_write_.find({id, addr});
    if (d != drop_write_.end() && d->second > 0) {--d->second; return 1;}
    auto c = corrupt_write_.find({id, addr});
    if (c != corrupt_write_.end() && c->second > 0) {--c->second; return 2;}
    return 0;
  }
  void apply_pending()  // m_ を持っていること
  {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->at <= now) {
        mem_[it->id][it->addr] = it->val;
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
  }
  void handle(const std::vector<uint8_t> & pkt)
  {
    const uint8_t id = pkt[2], inst = pkt[4];
    {
      std::vector<uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
      std::lock_guard<std::mutex> g(m_);
      request_time_ = std::chrono::steady_clock::now();
      sync_slot_ = 0;
      ++counts_[inst];
      if (chk(body) != pkt.back()) {++bad_requests_;}
    }
    if (inst == 0x01) {
      if (mem_.count(id)) {reply(id, {});}
    } else if (inst == 0x02) {
      read_reply(id, pkt[5], pkt[6]);
    } else if (inst == 0x82) {
      for (size_t i = 7; i < pkt.size() - 1; ++i) {read_reply(pkt[i], pkt[5], pkt[6], true);}
    } else if (inst == 0x03) {
      const int mode = do_write(inst, id, pkt[5], std::vector<uint8_t>(pkt.begin() + 6, pkt.end() - 1));
      bool known;
      {
        std::lock_guard<std::mutex> g(m_);
        known = mem_.count(id) > 0;
      }
      if (known && (mode == 0 || mode == 2)) {reply(id, {}, mode == 2);}
    } else if (inst == 0x83) {
      const uint8_t addr = pkt[5], len = pkt[6];
      for (size_t i = 7; i + len < pkt.size(); i += len + 1u) {
        do_write(inst, pkt[i], addr, std::vector<uint8_t>(pkt.begin() + i + 1, pkt.begin() + i + 1 + len));
      }
    }
  }
  void read_reply(uint8_t id, uint8_t addr, uint8_t len, bool in_sync = false)
  {
    std::vector<uint8_t> data;
    bool corrupt = false;
    int extra_us = 0;
    {
      std::lock_guard<std::mutex> g(m_);
      if (!mem_.count(id)) {return;}
      apply_pending();
      for (int i = 0; i < len; ++i) {data.push_back(mem_[id][addr + i]);}
      auto ov = override_.find({id, addr});
      if (ov != override_.end() && ov->second.first > 0) {
        --ov->second.first;
        for (int i = 0; i < len; ++i) {data[i] = (ov->second.second >> (8 * i)) & 0xFF;}
      }
      auto d = drop_.find({id, addr});
      if (d != drop_.end() && d->second > 0) {--d->second; return;}
      auto c = corrupt_.find({id, addr});
      if (c != corrupt_.end() && c->second > 0) {--c->second; corrupt = true;}
      auto dn = delay_next_.find({id, addr});
      if (dn != delay_next_.end() && dn->second.first > 0) {--dn->second.first; extra_us = dn->second.second;}
    }
    reply(id, data, corrupt, extra_us, addr, in_sync);
  }

  std::vector<uint8_t> ids_;
  std::map<std::pair<uint8_t, uint8_t>, std::pair<int, int>> override_;  // (id, addr) -> {残り回数, 値}
  std::map<uint8_t, int> reply_delay_us_;
  int frame_spacing_us_{0};
  int sync_slot_{0};
  std::map<std::pair<uint8_t, uint8_t>, std::pair<int, int>> delay_next_;  // (id, addr) -> {残り回数, us}
  std::chrono::steady_clock::time_point request_time_{};
  std::vector<Queued> queue_;
  uint64_t seq_{0};
  std::chrono::steady_clock::time_point last_due_{};
  std::vector<ReplyLogEntry> reply_log_;
  std::map<uint8_t, std::array<uint8_t, 256>> mem_;
  std::map<uint8_t, bool> is_gripper_;
  std::map<uint8_t, uint8_t> err_;
  std::map<std::pair<uint8_t, uint8_t>, int> corrupt_, drop_;
  std::set<uint8_t> ignored_addr_;
  std::set<std::pair<uint8_t, uint8_t>> ignored_id_addr_;
  std::set<uint8_t> ignore_lock1_;
  struct Pending
  {
    std::chrono::steady_clock::time_point at;
    uint8_t id;
    uint8_t addr;
    uint8_t val;
  };
  std::vector<Pending> pending_;
  int commit_delay_ms_{0};
  std::map<std::pair<uint8_t, uint8_t>, int> drop_write_, corrupt_write_;
  int cut_after_{-1};
  int applied_writes_{0};
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
