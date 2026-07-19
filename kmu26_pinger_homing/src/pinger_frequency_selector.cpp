#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iostream>
#include <map>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace kmu26_pinger_homing {

class FingerFrequencySelector final : public rclcpp::Node {
 public:
  FingerFrequencySelector() : Node("pinger_frequency_selector") {
    audio_topic_ = declare_parameter<std::string>("audio_topic", "/audio");
    selected_topic_ = declare_parameter<std::string>(
        "selected_frequency_topic", "/pinger_homing/selected_frequency_hz");
    candidate_topic_ = declare_parameter<std::string>(
        "candidate_topic", "/pinger_homing/frequency_candidates");
    manual_selection_topic_ = declare_parameter<std::string>(
        "manual_selection_topic", "/pinger_homing/manual_selection");
    sample_rate_ = declare_parameter<int>("sample_rate", 96000);
    channels_ = std::max(1, static_cast<int>(declare_parameter<int>("channels", 2)));
    channel_index_ = std::clamp(static_cast<int>(declare_parameter<int>("channel_index", 0)), 0, channels_ - 1);
    monitor_s_ = std::clamp(declare_parameter<double>("monitor_s", 5.0), 1.0, 30.0);
    min_frequency_ = declare_parameter<double>("min_frequency_hz", 15000.0);
    max_frequency_ = declare_parameter<double>("max_frequency_hz", 25000.0);
    frequency_step_ = std::clamp(declare_parameter<double>("frequency_step_hz", 100.0), 25.0, 500.0);
    fine_frequency_step_ = std::clamp(
        declare_parameter<double>("fine_frequency_step_hz", 2.0), 0.25, 10.0);
    fine_half_width_ = std::clamp(
        declare_parameter<double>("fine_half_width_hz", 75.0), 10.0, 250.0);
    window_size_ = std::clamp(static_cast<int>(declare_parameter<int>("window_size", 4096)), 512, 16384);
    hop_size_ = std::clamp(static_cast<int>(declare_parameter<int>("hop_size", 4096)), 512, window_size_);
    auto_select_top_ = declare_parameter<bool>("auto_select_top", false);
    // A frequency choice is valid only for this five-second scan.  Do not
    // latch it: a new test-tank launch must never start probing from the
    // previous run's DDS history and then have its Phase accumulator reset
    // underneath an active ABBA leg.
    selected_pub_ = create_publisher<std_msgs::msg::Float64>(
        selected_topic_, rclcpp::QoS(1));
    candidate_pub_ = create_publisher<std_msgs::msg::String>(
        candidate_topic_, rclcpp::QoS(1).transient_local());
    audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
        audio_topic_, rclcpp::SensorDataQoS(),
        [this](const audio_common_msgs::msg::AudioData::SharedPtr msg) { on_audio(*msg); });
    manual_selection_sub_ = create_subscription<std_msgs::msg::String>(
        manual_selection_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { select_from_text(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { poll_selection(); });
    started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "monitoring %.0f--%.0f Hz for %.1f seconds",
                min_frequency_, max_frequency_, monitor_s_);
  }

 private:
  using Clock = std::chrono::steady_clock;
  struct Candidate { double frequency{0.0}; double magnitude{0.0}; int hits{0}; };

  static int32_t read_s32(const std::vector<uint8_t> &data, std::size_t i) {
    const uint32_t value = static_cast<uint32_t>(data[i]) |
        (static_cast<uint32_t>(data[i + 1]) << 8) |
        (static_cast<uint32_t>(data[i + 2]) << 16) |
        (static_cast<uint32_t>(data[i + 3]) << 24);
    return static_cast<int32_t>(value);
  }

  double magnitude(const std::vector<double> &window, double frequency) const {
    std::complex<double> sum(0.0, 0.0);
    const double step = 2.0 * M_PI * frequency / static_cast<double>(sample_rate_);
    const double denom = static_cast<double>(std::max(1, window_size_ - 1));
    for (std::size_t n = 0; n < window.size(); ++n) {
      const double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / denom));
      const double phase = step * static_cast<double>(sample_cursor_ + n);
      sum += w * window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
    }
    return std::abs(sum) / static_cast<double>(window.size());
  }

  void on_audio(const audio_common_msgs::msg::AudioData &msg) {
    const std::size_t frame_bytes = static_cast<std::size_t>(channels_) * 4U;
    for (std::size_t frame = 0; frame + frame_bytes <= msg.data.size(); frame += frame_bytes) {
      const std::size_t offset = frame + static_cast<std::size_t>(channel_index_) * 4U;
      samples_.push_back(static_cast<double>(read_s32(msg.data, offset)) / 2147483648.0);
    }
    while (samples_.size() >= static_cast<std::size_t>(window_size_) &&
           !monitor_finished_) {
      const std::vector<double> window(samples_.begin(),
                                       samples_.begin() + static_cast<std::ptrdiff_t>(window_size_));
      analyze(window);
      const std::size_t remove = std::min<std::size_t>(hop_size_, samples_.size());
      samples_.erase(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(remove));
      sample_cursor_ += remove;
    }
  }

  void analyze(const std::vector<double> &window) {
    // Coarse monitoring is intentionally inexpensive. Keep one recent raw
    // window so the final candidate can be refined without changing the
    // five-second operator workflow or the upstream audio contract.
    last_window_ = window;
    double best_frequency = min_frequency_;
    double best_magnitude = -1.0;
    for (double f = min_frequency_; f <= max_frequency_ + 0.5 * frequency_step_; f += frequency_step_) {
      const double value = magnitude(window, f);
      if (value > best_magnitude) { best_magnitude = value; best_frequency = f; }
    }
    const double key = std::round(best_frequency / frequency_step_) * frequency_step_;
    auto &candidate = candidates_[key];
    candidate.frequency = key;
    candidate.magnitude += best_magnitude;
    candidate.hits += 1;
  }

  std::vector<Candidate> ranked() const {
    std::vector<Candidate> values;
    for (const auto &entry : candidates_) {
      if (entry.second.hits >= 2) values.push_back(entry.second);
    }
    if (values.empty()) {
      for (const auto &entry : candidates_) values.push_back(entry.second);
    }
    std::sort(values.begin(), values.end(), [](const Candidate &a, const Candidate &b) {
      if (a.hits != b.hits) return a.hits > b.hits;
      return a.magnitude > b.magnitude;
    });
    if (values.size() > 5U) values.resize(5U);
    return values;
  }

  Candidate refine_candidate(Candidate candidate) const {
    if (last_window_.empty()) return candidate;
    const double low = std::max(min_frequency_, candidate.frequency - fine_half_width_);
    const double high = std::min(max_frequency_, candidate.frequency + fine_half_width_);
    double best_frequency = candidate.frequency;
    double best_magnitude = -1.0;
    for (double frequency = low; frequency <= high + 0.5 * fine_frequency_step_;
         frequency += fine_frequency_step_) {
      const double value = magnitude(last_window_, frequency);
      if (value > best_magnitude) {
        best_magnitude = value;
        best_frequency = frequency;
      }
    }
    if (best_magnitude >= 0.0) {
      candidate.frequency = best_frequency;
      candidate.magnitude = best_magnitude;
    }
    return candidate;
  }

  std::vector<Candidate> refined_ranked() const {
    auto values = ranked();
    for (auto &candidate : values) candidate = refine_candidate(candidate);
    return values;
  }

  void finish_monitor() {
    if (monitor_finished_) return;
    monitor_finished_ = true;
    const auto values = refined_ranked();
    std::ostringstream json;
    json << "{\"ready\":true,\"candidates\":[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i) json << ',';
      json << "{\"frequency_hz\":" << values[i].frequency
           << ",\"hits\":" << values[i].hits
           << ",\"score\":" << values[i].magnitude << '}';
    }
    json << "]}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    candidate_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "repeated frequency candidates (top %zu):", values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  [%zu] %.1f Hz (hits=%d score=%.6g)",
                  i + 1, values[i].frequency, values[i].hits, values[i].magnitude);
    }
    if (auto_select_top_ && !values.empty()) select(values.front().frequency);
  }

  void select(double frequency) {
    if (selected_) return;
    if (!std::isfinite(frequency) || frequency < min_frequency_ || frequency > max_frequency_) {
      RCLCPP_WARN(get_logger(), "selection %.2f Hz is outside monitored band", frequency);
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = frequency;
    selected_pub_->publish(msg);
    selected_ = true;
    RCLCPP_INFO(get_logger(), "frequency %.1f Hz selected; homing controller may start", frequency);
  }

  void select_from_text(const std::string &text) {
    if (!monitor_finished_ || selected_) return;
    const auto values = refined_ranked();
    try {
      const double value = std::stod(text);
      if (value >= 1.0 && value <= static_cast<double>(values.size()) &&
          std::floor(value) == value) {
        select(values[static_cast<std::size_t>(value) - 1U].frequency);
      } else {
        select(value);
      }
    } catch (...) {
      RCLCPP_WARN(get_logger(), "enter candidate number 1-%zu or frequency in Hz", values.size());
    }
  }

  void poll_selection() {
    if (!monitor_finished_ && std::chrono::duration<double>(Clock::now() - started_).count() >= monitor_s_) {
      finish_monitor();
    }
    if (!monitor_finished_ || selected_ || auto_select_top_) return;
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    if (::poll(&descriptor, 1, 0) <= 0) return;
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) return;
    select_from_text(line);
  }

  std::string audio_topic_, selected_topic_, candidate_topic_, manual_selection_topic_;
  int sample_rate_{96000}, channels_{2}, channel_index_{0}, window_size_{4096}, hop_size_{4096};
  double monitor_s_{5.0}, min_frequency_{15000.0}, max_frequency_{25000.0}, frequency_step_{100.0};
  double fine_frequency_step_{2.0}, fine_half_width_{75.0};
  bool auto_select_top_{false}, monitor_finished_{false}, selected_{false};
  std::uint64_t sample_cursor_{0};
  std::deque<double> samples_;
  std::vector<double> last_window_;
  std::map<double, Candidate> candidates_;
  Clock::time_point started_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_selection_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kmu26_pinger_homing

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kmu26_pinger_homing::FingerFrequencySelector>());
  rclcpp::shutdown();
  return 0;
}
