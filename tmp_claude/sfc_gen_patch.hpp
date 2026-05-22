// ============================================================================
// 替换源文件: epic/local_planner/minco_planner/include/gcopter/sfc_gen.hpp
// 替换段落 : namespace sfc_gen { ... } 整段（约 102-228 行）
// 修复内容 : convexCover 健壮性修复 + 救援分支重构 + shortCut 空输入保护
// ============================================================================

namespace sfc_gen {

inline void convexCover(const std::unique_ptr<Visualizer> &vizer,
                        const std::vector<Eigen::Vector3d> &path,
                        const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lowCorner,
                        const Eigen::Vector3d &highCorner,
                        const double &progress, const double &range,
                        std::vector<Eigen::MatrixX4d> &hpolys,
                        const double eps = 1.0e-6,
                        const double dilate_radius_ = 0.1) {
  hpolys.clear();                                       // 修复：恢复 clear，避免追加陷阱
  const int n = path.size();
  if (n < 2) return;                                    // 修复：路径太短直接返回

  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;  bd(5, 2) = -1.0;

  Eigen::MatrixX4d hp, gap;
  Eigen::Vector3d a, b = path[0];
  std::vector<Eigen::Vector3d> valid_pc;
  std::vector<Eigen::Vector3d> bs;
  valid_pc.reserve(points.size());

  // 收缩 hp 的小工具：每行约束 n·x + d ≤ 0 → 把 d 推大，等价多面体每个面向内收 r
  auto shrink_hp = [&](Eigen::MatrixX4d &poly, double r) {
    poly.col(3) = poly.col(3).array()
                + r * poly.leftCols(3).rowwise().norm().array();
  };

  for (int i = 1; i < n;) {
    a = b;
    if ((a - path[i]).norm() > progress) {
      b = (path[i] - a).normalized() * progress + a;
    } else {
      b = path[i];
      i++;
    }
    bs.emplace_back(b);

    bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
    bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
    bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
    bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
    bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
    bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

    valid_pc.clear();
    for (const Eigen::Vector3d &p : points) {
      if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
        valid_pc.emplace_back(p);
      }
    }

    // 修复：valid_pc 为空时 valid_pc[0].data() 是 UB
    const double *pc_data = valid_pc.empty() ? nullptr : valid_pc[0].data();
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
        pc_data, 3, static_cast<int>(valid_pc.size()));

    // 修复：firi 返回 false 时 hp 残留旧值，要丢弃这一段
    if (!firi::firi(bd, pc, a, b, hp)) {
      continue;
    }

    Eigen::MatrixX4d hp_origin = hp;
    shrink_hp(hp, dilate_radius_);

    Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

    // 收缩后 b 还落在边界外侧 → 救援分支
    if (((hp * bh).array() > -eps).cast<int>().sum() > 0) {
      // 修复：iterations 1 → 3，让椭球能长开
      // 修复：救援多面体本身就是单点种子产生的小球，不再做 dilate 收缩，避免塌成空
      Eigen::MatrixX4d hp_a, hp_m, hp_b;
      bool ok_a = firi::firi(bd, pc, a, a, hp_a, 3);
      bool ok_m = firi::firi(bd, pc, (a + b) / 2.0, (a + b) / 2.0, hp_m, 3);
      bool ok_b = firi::firi(bd, pc, b, b, hp_b, 3);
      if (ok_a) hpolys.emplace_back(hp_a);
      if (ok_m) hpolys.emplace_back(hp_m);
      if (ok_b) hpolys.emplace_back(hp_b);

      // 修复：原代码末尾还会无条件再 emplace_back(hp) 一次，造成 b 段重复推。
      //      在救援分支不再追加 hp。
      continue;
    }

    // 非救援分支：把 a 和上一段的衔接缝补上（如有需要）
    if (!hpolys.empty()) {
      const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
      if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
        if (firi::firi(bd, pc, a, a, gap, 3)) {         // 修复：iterations 1→3，并校验返回值
          shrink_hp(gap, dilate_radius_);                // 修复：gap 也按 dilate_radius_ 收缩
          hpolys.emplace_back(gap);
        }
      }
    }

    hpolys.emplace_back(hp);                            // 非救援分支正常推一次
  }
}

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys) {
  if (hpolys.empty()) return;                           // 修复：空输入保护

  std::vector<Eigen::MatrixX4d> htemp = hpolys;
  if (htemp.size() == 1) {
    Eigen::MatrixX4d headPoly = htemp.front();
    htemp.insert(htemp.begin(), headPoly);
  }
  hpolys.clear();

  int M = htemp.size();
  Eigen::MatrixX4d hPoly;
  bool overlap;
  std::deque<int> idices;
  idices.push_front(M - 1);
  for (int i = M - 1; i >= 0; i--) {
    for (int j = 0; j < i; j++) {
      if (j < i - 1) {
        overlap = geo_utils::overlap(htemp[i], htemp[j], 1e-2);
      } else {
        overlap = true;
      }
      if (overlap) {
        idices.push_front(j);
        i = j + 1;
        break;
      }
    }
  }
  for (const auto &ele : idices) {
    hpolys.push_back(htemp[ele]);
  }
}

} // namespace sfc_gen
