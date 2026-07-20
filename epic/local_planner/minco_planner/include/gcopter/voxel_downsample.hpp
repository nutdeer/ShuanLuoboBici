/***
 * @Description: 替换 PCL VoxelGrid 的降采样工具（双走廊薄膜方案专用版）
 *
 *   功能等价于 pcl::VoxelGrid<PointXYZ>::filter()，额外输出:
 *     - voxel_raw[i]：与 surf_points[i] 索引对齐的原始点集合
 *       （PCL VoxelGrid 黑盒内部丢弃了这份对应关系）
 *     - voxel_map：体素 key → 原始点（按 key 查询用，例如 bd 边界修复）
 *     - min_p：网格原点（bbox 最小角，与 PCL VoxelGrid 一致）
 *
 *   用法（替换原 PCL VoxelGrid 块）:
 *     #include "gcopter/voxel_downsample.hpp"
 *     auto ds = fast_planner::voxelDownsample(Searched_Points, leaf);
 *     const auto& surf_points = ds.surf_points;   // 喂 convexCover 的 pc
 *     const auto& voxel_raw   = ds.voxel_raw;     // 喂 firi 的 pc_raw
 *
 *   数学等价性:
 *     - 起点：bbox min_p，与 PCL VoxelGrid 一致
 *     - 质心：格内算术平均，与 PCL VoxelGrid 一致
 *     - 迭代顺序：unordered_map 迭代顺序不定，但调用方不依赖顺序
 *
 *   关键保证:
 *     surf_points[i] = mean(voxel_raw[i])
 *     ∀ q ∈ voxel_raw[i]:  ‖q − surf_points[i]‖ ≤ leaf · √3 / 2
 */
#ifndef _VOXEL_DOWNSAMPLE_H_
#define _VOXEL_DOWNSAMPLE_H_

#include <Eigen/Dense>
#include <lidar_map/lidar_map.h>  // PointVector (= vector<PointType, Eigen-aligned>)

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace fast_planner {

// ---- voxel key hash --------------------------------------------------------
struct VoxelHash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::hash<int> h;
    std::size_t seed = 0;
    seed ^= h(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct VoxelEqual {
  bool operator()(const Eigen::Vector3i &a, const Eigen::Vector3i &b) const {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }
};

// ---- 返回值 ----------------------------------------------------------------
struct VoxelDownsampleResult {
  /// 降采样质心（等价于 PCL VoxelGrid::filter() 输出）
  std::vector<Eigen::Vector3d> surf_points;

  /// 与 surf_points 索引对齐的原始点集合
  ///   voxel_raw[i] = surf_points[i] 对应体素内的全部原始点
  /// FIRI 双走廊薄膜检查 raw point 侵入必撞区时使用
  std::vector<std::vector<Eigen::Vector3d>> voxel_raw;

  /// 与 surf_points 索引对齐的体素支撑界（raw 相对质心的轴向半宽，逐轴 >= 0）
  ///   ∀ q ∈ voxel_raw[i], 单位方向 n̂:
  ///     |n̂·(q − surf_points[i])| ≤ s_i(n̂) = |n̂|·voxel_ext[i]
  /// FIRI 最后一轮用它做 O(1) raw 排除，代替遍历 voxel_raw
  std::vector<Eigen::Vector3d> voxel_ext;

  /// 体素 key → 原始点（按 key 查询用，例如 bd 边界修复的外扩过滤）
  std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, VoxelHash, VoxelEqual>
      voxel_map;

  /// 网格原点 = 点云 bbox 最小角（与 PCL VoxelGrid 一致）
  Eigen::Vector3d min_p;
};

// ---- 主函数 ----------------------------------------------------------------
inline VoxelDownsampleResult voxelDownsample(const PointVector &raw_points,
                                             double leaf) {
  VoxelDownsampleResult result;

  if (raw_points.empty()) {
    result.min_p = Eigen::Vector3d::Zero();
    return result;
  }

  // Pass 1: 求 bbox 最小角 —— 对齐 PCL VoxelGrid 的网格原点
  Eigen::Vector3d min_p(std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max());
  for (const auto &p : raw_points) {
    if (p.x < min_p.x()) min_p.x() = p.x;
    if (p.y < min_p.y()) min_p.y() = p.y;
    if (p.z < min_p.z()) min_p.z() = p.z;
  }
  result.min_p = min_p;

  // Pass 2: 单遍遍历 —— 分桶 + 累加质心分子（同时把原始点存进 voxel_map）
  std::unordered_map<Eigen::Vector3i, Eigen::Vector3d, VoxelHash, VoxelEqual> sum_map;
  const double inv_leaf = 1.0 / leaf;

  for (const auto &p : raw_points) {
    Eigen::Vector3i key(
        static_cast<int>(std::floor((p.x - min_p.x()) * inv_leaf)),
        static_cast<int>(std::floor((p.y - min_p.y()) * inv_leaf)),
        static_cast<int>(std::floor((p.z - min_p.z()) * inv_leaf)));
    Eigen::Vector3d pt(p.x, p.y, p.z);
    result.voxel_map[key].push_back(pt);
    auto sum_it = sum_map.find(key);
    if (sum_it == sum_map.end()) {
      sum_map.emplace(key, pt);
    } else {
      sum_it->second += pt;
    }
  }

  // Pass 3: 一次遍历 sum_map —— 同步产出 surf_points 与索引对齐的 voxel_raw
  const std::size_t n_voxels = sum_map.size();
  result.surf_points.reserve(n_voxels);
  result.voxel_raw.reserve(n_voxels);
  result.voxel_ext.reserve(n_voxels);
  for (const auto &kv : sum_map) {
    const Eigen::Vector3i &key = kv.first;
    const Eigen::Vector3d &sum = kv.second;
    const auto &pts = result.voxel_map[key];  // 之前放到 map 里的原始点
    const double n = static_cast<double>(pts.size());
    const Eigen::Vector3d centroid = sum / n;
    Eigen::Vector3d ext = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d &q : pts) {
      ext = ext.cwiseMax((q - centroid).cwiseAbs());
    }
    result.surf_points.emplace_back(centroid);
    result.voxel_raw.emplace_back(pts);
    result.voxel_ext.emplace_back(ext);
  }

  return result;
}

}  // namespace fast_planner

#endif  // _VOXEL_DOWNSAMPLE_H_
