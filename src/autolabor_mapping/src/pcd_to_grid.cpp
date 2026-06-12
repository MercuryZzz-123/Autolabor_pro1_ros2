// pcd_to_grid.cpp
//
// 把 FAST-LIO 输出的 3D 点云地图(PCD)压成 Nav2 可用的 2D 占据栅格地图(pgm + yaml)。
//
// 思路:对点云做高度切片,
//   - 障碍层(地面以上 [min_height, max_height])内有点的格 -> 占据(墙/家具)
//   - 地面层(地面 ± ground_tol)被扫到但无障碍的格 -> 可通行
//   - 其余从未观测到的格 -> 未知
//
// 高度换算:FAST-LIO 的 PCD 原点在雷达起始位置,雷达离地 sensor_height,
//   所以地面在 z = -sensor_height; "地面以上高度 h" 对应 PCD 里的 z = h - sensor_height。
//   这样本工具填的 min_height/max_height 和导航端 pointcloud_to_scan 的同名参数
//   语义一致,建图与导航的墙体自动对齐。
//
// 用法:
//   pcd_to_grid --input scans.pcd --output map [选项]
// 生成 map.pgm 和 map.yaml。

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

struct Options
{
  std::string input = "scans.pcd";
  std::string output = "map";        // 输出前缀 -> output.pgm / output.yaml
  double resolution = 0.05;          // m/格
  double sensor_height = 0.5;        // 雷达离地高度,用于把"地面以上高度"换算到 PCD z
  double min_height = 0.10;          // 障碍层下界(地面以上)
  double max_height = 1.50;          // 障碍层上界(地面以上)
  double ground_tol = 0.15;          // 地面层厚度(地面 ± 此值视为地面,用于标记可通行)
  int occ_min_points = 1;            // 单格判为占据所需的最少障碍点数(去噪)
  double padding = 1.0;              // 地图四周留白(m)
  bool unknown_to_free = false;      // 把从未观测到的格也输出为可通行(空白底)
  double occupied_thresh = 0.65;     // 写入 yaml 的占据阈值
  double free_thresh = 0.196;        // 写入 yaml 的空闲阈值
};

void print_usage(const char * prog)
{
  std::cout
    << "用法: " << prog << " --input <scans.pcd> --output <map前缀> [选项]\n"
    << "  --input            输入 PCD 路径 (默认 scans.pcd)\n"
    << "  --output           输出前缀,生成 <前缀>.pgm 和 <前缀>.yaml (默认 map)\n"
    << "  --resolution       栅格分辨率 m/格 (默认 0.05)\n"
    << "  --sensor_height    雷达离地高度 m,把'地面以上高度'换算到 PCD z (默认 0.5)\n"
    << "  --min_height       障碍层下界(地面以上 m) (默认 0.10)\n"
    << "  --max_height       障碍层上界(地面以上 m) (默认 1.50)\n"
    << "  --ground_tol       地面层厚度(地面 ± 此值标记可通行 m) (默认 0.15)\n"
    << "  --occ_min_points   单格判为占据所需最少障碍点数 (默认 1)\n"
    << "  --padding          地图四周留白 m (默认 1.0)\n"
    << "  --unknown_to_free  把未观测格也输出为可通行(空白底)\n"
    << "  --occupied_thresh  写入 yaml 的占据阈值 (默认 0.65)\n"
    << "  --free_thresh      写入 yaml 的空闲阈值 (默认 0.196)\n";
}

bool parse_args(int argc, char ** argv, Options & opt)
{
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char * name) -> std::string {
        if (i + 1 >= argc) {
          std::cerr << "缺少参数值: " << name << "\n";
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--help" || a == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (a == "--input") {
      opt.input = need("--input");
    } else if (a == "--output") {
      opt.output = need("--output");
    } else if (a == "--resolution") {
      opt.resolution = std::stod(need("--resolution"));
    } else if (a == "--sensor_height") {
      opt.sensor_height = std::stod(need("--sensor_height"));
    } else if (a == "--min_height") {
      opt.min_height = std::stod(need("--min_height"));
    } else if (a == "--max_height") {
      opt.max_height = std::stod(need("--max_height"));
    } else if (a == "--ground_tol") {
      opt.ground_tol = std::stod(need("--ground_tol"));
    } else if (a == "--occ_min_points") {
      opt.occ_min_points = std::stoi(need("--occ_min_points"));
    } else if (a == "--padding") {
      opt.padding = std::stod(need("--padding"));
    } else if (a == "--unknown_to_free") {
      opt.unknown_to_free = true;
    } else if (a == "--occupied_thresh") {
      opt.occupied_thresh = std::stod(need("--occupied_thresh"));
    } else if (a == "--free_thresh") {
      opt.free_thresh = std::stod(need("--free_thresh"));
    } else {
      std::cerr << "未知参数: " << a << "\n";
      print_usage(argv[0]);
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parse_args(argc, argv, opt)) {
    return 2;
  }
  if (opt.min_height > opt.max_height) {
    std::cerr << "错误: min_height 必须 <= max_height\n";
    return 2;
  }
  if (opt.resolution <= 0.0) {
    std::cerr << "错误: resolution 必须 > 0\n";
    return 2;
  }

  // "地面以上高度" -> PCD 坐标系下的 z 窗口
  const double obs_z_min = opt.min_height - opt.sensor_height;
  const double obs_z_max = opt.max_height - opt.sensor_height;
  const double gnd_z_min = -opt.sensor_height - opt.ground_tol;
  const double gnd_z_max = -opt.sensor_height + opt.ground_tol;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(opt.input, *cloud) < 0) {
    std::cerr << "错误: 无法读取 PCD: " << opt.input << "\n";
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "错误: 点云为空: " << opt.input << "\n";
    return 1;
  }
  std::cout << "读取点云: " << cloud->size() << " 点,来自 " << opt.input << "\n";

  // 第一遍:统计参与投影的点(障碍层或地面层)的 x-y 边界
  double x_min = std::numeric_limits<double>::max();
  double y_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_max = std::numeric_limits<double>::lowest();
  std::size_t used = 0;
  for (const auto & p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const bool is_obs = (p.z >= obs_z_min && p.z <= obs_z_max);
    const bool is_gnd = (p.z >= gnd_z_min && p.z <= gnd_z_max);
    if (!is_obs && !is_gnd) {
      continue;
    }
    x_min = std::min(x_min, static_cast<double>(p.x));
    y_min = std::min(y_min, static_cast<double>(p.y));
    x_max = std::max(x_max, static_cast<double>(p.x));
    y_max = std::max(y_max, static_cast<double>(p.y));
    ++used;
  }
  if (used == 0) {
    std::cerr << "错误: 给定高度窗口内没有点。请检查 sensor_height / min_height / max_height。\n";
    return 1;
  }

  // 四周留白
  x_min -= opt.padding;
  y_min -= opt.padding;
  x_max += opt.padding;
  y_max += opt.padding;

  const int width = static_cast<int>(std::ceil((x_max - x_min) / opt.resolution));
  const int height = static_cast<int>(std::ceil((y_max - y_min) / opt.resolution));
  if (width <= 0 || height <= 0) {
    std::cerr << "错误: 计算出的地图尺寸非法。\n";
    return 1;
  }
  std::cout << "地图尺寸: " << width << " x " << height << " 格 @ "
            << opt.resolution << " m, origin=[" << x_min << ", " << y_min << "]\n";

  // 每格统计:障碍点数 / 是否有地面点
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<int> occ_count(cells, 0);
  std::vector<uint8_t> has_ground(cells, 0);

  // 第二遍:投影到栅格(row 自下而上,row=0 对应 y_min)
  for (const auto & p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const bool is_obs = (p.z >= obs_z_min && p.z <= obs_z_max);
    const bool is_gnd = (p.z >= gnd_z_min && p.z <= gnd_z_max);
    if (!is_obs && !is_gnd) {
      continue;
    }
    const int col = static_cast<int>((p.x - x_min) / opt.resolution);
    const int row = static_cast<int>((p.y - y_min) / opt.resolution);
    if (col < 0 || col >= width || row < 0 || row >= height) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(row) * width + col;
    if (is_obs) {
      occ_count[idx] += 1;
    }
    if (is_gnd) {
      has_ground[idx] = 1;
    }
  }

  // 生成 pgm 像素:占据=0(黑)、空闲=254(白)、未知=205(灰)。
  // pgm 行序自上而下,顶行对应最大 y,故按 (height-1-row) 翻转,与 Nav2 map_server 约定一致。
  const uint8_t OCC = 0, FREE = 254, UNKNOWN = 205;
  std::vector<uint8_t> pixels(cells, UNKNOWN);
  std::size_t n_occ = 0, n_free = 0, n_unknown = 0;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const std::size_t src = static_cast<std::size_t>(row) * width + col;
      uint8_t v;
      if (occ_count[src] >= opt.occ_min_points) {
        v = OCC;
        ++n_occ;
      } else if (has_ground[src]) {
        v = FREE;
        ++n_free;
      } else {
        v = opt.unknown_to_free ? FREE : UNKNOWN;
        ++n_unknown;
      }
      const int pgm_row = (height - 1) - row;
      const std::size_t dst = static_cast<std::size_t>(pgm_row) * width + col;
      pixels[dst] = v;
    }
  }
  std::cout << "栅格统计: 占据=" << n_occ << " 空闲=" << n_free
            << " 未知=" << n_unknown << "\n";

  // 写 PGM (P5 binary)
  const std::string pgm_path = opt.output + ".pgm";
  {
    std::ofstream ofs(pgm_path, std::ios::binary);
    if (!ofs) {
      std::cerr << "错误: 无法写入 " << pgm_path << "\n";
      return 1;
    }
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char *>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
  }

  // 写 YAML (Nav2 map_server 格式),image 用相对文件名,origin 为左下角
  const std::string yaml_path = opt.output + ".yaml";
  {
    std::string image_name = pgm_path;
    const auto slash = image_name.find_last_of("/\\");
    if (slash != std::string::npos) {
      image_name = image_name.substr(slash + 1);
    }
    std::ofstream ofs(yaml_path);
    if (!ofs) {
      std::cerr << "错误: 无法写入 " << yaml_path << "\n";
      return 1;
    }
    ofs << "image: " << image_name << "\n";
    ofs << "resolution: " << opt.resolution << "\n";
    ofs << "origin: [" << x_min << ", " << y_min << ", 0.0]\n";
    ofs << "negate: 0\n";
    ofs << "occupied_thresh: " << opt.occupied_thresh << "\n";
    ofs << "free_thresh: " << opt.free_thresh << "\n";
  }

  std::cout << "已写出: " << pgm_path << " 和 " << yaml_path << "\n";
  return 0;
}
