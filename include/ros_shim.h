// Drop-in replacements for the small subset of ROS1 APIs Global-LVBA uses.
// Lets us strip the catkin / roscpp build dependency without rewriting the
// call sites: keep .param() / .advertise() / .publish() shapes identical so
// dataset_io / lvba_system code only needs the include line swapped.
//
// Scope on purpose: just enough to compile + run the LVBA pipeline on
// pre-cooked datasets. No publish/subscribe semantics, no rate control, no
// rosparam server -- config comes from a single YAML file.
#ifndef LVBA_ROS_SHIM_H
#define LVBA_ROS_SHIM_H

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace lvba_shim {

// Pull a dotted/slash-separated key like "data_config/data_path" out of a
// YAML tree. Returns an invalid Node if any segment is missing.
inline YAML::Node resolve(const YAML::Node& root, const std::string& dotted) {
  YAML::Node cur = YAML::Clone(root);
  size_t start = 0;
  while (start < dotted.size()) {
    size_t slash = dotted.find('/', start);
    std::string key = dotted.substr(start, slash == std::string::npos
                                              ? dotted.size() - start
                                              : slash - start);
    if (!cur || !cur.IsMap() || !cur[key]) return YAML::Node();
    cur = cur[key];
    if (slash == std::string::npos) break;
    start = slash + 1;
  }
  return cur;
}

// ros::Time stand-in. Only used as a message header stamp -- LVBA doesn't
// read it for any algorithmic logic, so returning seconds-since-epoch is fine.
struct Time {
  double secs{0.0};
  static Time now() {
    using clock = std::chrono::system_clock;
    auto d = clock::now().time_since_epoch();
    return {std::chrono::duration<double>(d).count()};
  }
};

// Minimal sensor_msgs::PointCloud2 stand-in. The pipeline only sets
// .header.stamp and .header.frame_id, then hands the message to a
// Publisher::publish() (no-op here). No serialization, no buffer.
struct StubHeader {
  Time stamp;
  std::string frame_id;
};
struct StubPointCloud2 {
  StubHeader header;
};

// Publisher stub: every .publish() call is a no-op. The LVBA pipeline only
// uses publishers for rviz viz; nothing downstream consumes them and the
// algorithm doesn't read them back. Non-template (matches ros::Publisher's
// real type) with a templated .publish<>() so any message shape compiles.
class Publisher {
public:
  template <typename MsgT>
  void publish(const MsgT&) const { /* no-op */ }
};

// Config: read-only view onto a YAML file. Exposes the ros::NodeHandle
// methods Global-LVBA actually calls (.param<T>, .advertise<T>, .ok()).
class Config {
public:
  Config() = default;
  explicit Config(const std::string& yaml_path) { load(yaml_path); }

  void load(const std::string& yaml_path) {
    root_ = YAML::LoadFile(yaml_path);
  }

  // ros::NodeHandle::param<T>(key, out, default) signature.
  template <typename T>
  bool param(const std::string& key, T& out, const T& def) const {
    YAML::Node n = resolve(root_, key);
    if (!n) { out = def; return false; }
    try {
      out = n.as<T>();
      return true;
    } catch (const YAML::Exception&) {
      out = def;
      return false;
    }
  }

  // ros::NodeHandle::advertise<MsgT>(topic, queue, latch) -> Publisher.
  // Topic name and queue size are ignored; returned Publisher is a no-op.
  template <typename MsgT>
  Publisher advertise(const std::string&, int, bool = false) const {
    return Publisher{};
  }

  // Used as a liveness check inside loops (nh_.ok()). Always true here --
  // no node lifecycle to track.
  bool ok() const { return true; }

private:
  YAML::Node root_;
};

}  // namespace lvba_shim

// Source-compatible aliases so existing files keep reading like ROS code
// with only their include line swapped.
namespace ros {
using NodeHandle = lvba_shim::Config;
using Publisher = lvba_shim::Publisher;
using Time = lvba_shim::Time;
inline void init(int&, char**, const std::string&) { /* no-op */ }
inline void spin() { /* no-op: pipeline is one-shot */ }
}  // namespace ros

// Stand-in for the only sensor_msgs type LVBA references.
namespace sensor_msgs {
using PointCloud2 = lvba_shim::StubPointCloud2;
}

// Replace pcl::toROSMsg with a no-op so the publish call sites compile
// without dragging in pcl_conversions / sensor_msgs message generation.
// The output message is unused by anyone here -- publishers are stubs --
// so leaving it default-initialised is fine.
namespace pcl {
template <typename CloudT>
inline void toROSMsg(const CloudT&, sensor_msgs::PointCloud2&) {
  /* no-op: visualization-only path, downstream Publisher is a stub */
}
}

// ROS_WARN / ROS_INFO / ROS_ERROR map to stderr/stdout prints. Original
// macros take printf-style varargs; we forward to std::fprintf so format
// strings keep working as-is.
#define ROS_INFO(...)  do { std::fprintf(stdout, "[ INFO] " __VA_ARGS__); std::fprintf(stdout, "\n"); } while (0)
#define ROS_WARN(...)  do { std::fprintf(stderr, "[ WARN] " __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_ERROR(...) do { std::fprintf(stderr, "[ERROR] " __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_DEBUG(...) do { } while (0)
#define ROS_INFO_STREAM(x)  do { std::cout << "[ INFO] " << x << '\n'; } while (0)
#define ROS_WARN_STREAM(x)  do { std::cerr << "[ WARN] " << x << '\n'; } while (0)
#define ROS_ERROR_STREAM(x) do { std::cerr << "[ERROR] " << x << '\n'; } while (0)

#endif  // LVBA_ROS_SHIM_H
