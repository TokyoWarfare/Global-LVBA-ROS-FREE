// Global-LVBA entry point. After the ROS strip this is a plain CLI:
//   ./lvba_run config.yaml
// The YAML file is what used to be loaded by `rosparam load` in lvba.launch
// (config/config.yaml in this repo by default).
#include <cstdio>
#include <cstdlib>
#include <string>

#include "ros_shim.h"
#include "lvba_system.h"

int main(int argc, char** argv) {
  std::string config_path =
      std::string(ROOT_DIR) + "config/config.yaml";  // default if no arg
  if (argc > 1) {
    config_path = argv[1];
  }

  ros::NodeHandle nh;
  try {
    nh.load(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[lvba] failed to load config '%s': %s\n",
                 config_path.c_str(), e.what());
    return 1;
  }
  std::printf("[lvba] config: %s\n", config_path.c_str());

  lvba::LvbaSystem lvba_system(nh);
  lvba_system.runFullPipeline();
  return 0;
}
