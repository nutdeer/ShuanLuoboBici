// ============================================================================
// 替换源文件: epic/local_planner/minco_planner/src/planner_manager.cpp
// 替换段落 :
//   段落 A — planExploreTraj 中 start_idx == -1 处理 + 重叠检查
//            （约 263-306 行）
//   段落 B — planExploreTraj 中 gcopter.setup(...) 调用
//            （约 344-347 行）
//   段落 C — flyToSafeRegion 中 surf_points[0].data() 处
//            （约 611-619 行）
//   段落 D — flyToSafeRegion 中 gcopter.setup(...) 调用
//            （在 flyToSafeRegion 后段，对应原代码 ~700 行附近）
// 修复内容 :
//   - 起点不在走廊时的 erase(-1) UB（解开 return false）
//   - 修双重赋值 `bool overlap = overlap = ...`
//   - flyToSafeRegion 空点云保护
//   - 两处 gcopter.setup(...) 调用补传 dilateRadiusSoft + weightPosSoft
// ============================================================================


// ────────────────────── 段落 A：planExploreTraj 起点保护 + overlap 修正 ──────────────────────
//
// 原 263-306 行：
//   Eigen::Vector4d bh;
//   bh << iniState.topLeftCorner<3, 1>(), 1.0;
//   int start_idx = -1;
//   for (int i = hPolys.size() - 1; i >= 0; i--) { ... }
//   if (start_idx == -1) {
//     ROS_ERROR("current position not in corridor");
//     ...
//     if (!safe) return flyToSafeRegion(is_static);
//     // return false;                ← 这一行被注释掉了，会导致下面 erase(-1) UB
//   }
//   if (start_idx != 0) {
//     hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
//   }
//   sfc_gen::shortCut(hPolys);
//   ...
//   while (back < hPolys.size() - 1) {
//     bool overlap = overlap =                       ← 双重赋值
//         geo_utils::overlap(hPolys[front], hPolys[back], 1e-2);
//
// 替换为：

  Eigen::Vector4d bh;
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  int start_idx = -1;
  for (int i = hPolys.size() - 1; i >= 0; i--) {
    Eigen::MatrixX4d hp = hPolys[i];
    if ((((hp * bh).array() > -1.0e-6).cast<int>().sum() <= 0)) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    ROS_ERROR("current position not in corridor");
    double time;
    bool safe =
        local_data_.traj_id_ >= 1 && checkTrajCollision(time) && time > 2.0;
    if (!safe)
      return flyToSafeRegion(is_static);
    return false;                                     // 修复：起点不在走廊就退出，避免 erase(-1) UB
  }
  if (start_idx != 0) {
    hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
  }
  sfc_gen::shortCut(hPolys);

  ros::Time hpoly_gen_end = ros::Time::now();

  if (hPolys.size() < 2) {
    cout << "hPolys size < 2" << endl;
    return false;
  }
  int front = 0;
  int back = 1;
  while (back < static_cast<int>(hPolys.size()) - 1) {
    bool overlap = geo_utils::overlap(hPolys[front], hPolys[back], 1e-2);   // 修复：去掉双重赋值
    if (overlap) {
      front += 1;
      back  += 1;
    } else {
      break;
    }
  }


// ────────────────────── 段落 B：planExploreTraj 的 gcopter.setup() 调用 ──────────────────────
//
// 原 344-347 行：
//   if (!gcopter.setup(
//           gcopter_config_->weightT, gcopter_config_->dilateRadiusSoft, iniState,
//           finState, hPolys, INFINITY, gcopter_config_->smoothingEps,
//           quadratureRes, magnitudeBounds, penaltyWeights, physicalParams)) {
//
// 注：原来这里第二个参数误传 dilateRadiusSoft，但因为没真正生效，等价于 hard 不被使用，
//      惩罚是直接打在 sfc_gen 已经按 hard 收缩好的多面体上。
//      新接口里第二个参数明确表示 "hard" 用于内部记录；
//      软启动 = dilateRadiusSoft（>= hard）→ soft_extra = soft − hard，由 setup 内部算。
//
// 替换为：

  if (!gcopter.setup(
          gcopter_config_->weightT,
          gcopter_config_->dilateRadiusHard,             // 修正：明确传 hard
          iniState, finState, hPolys, INFINITY,
          gcopter_config_->smoothingEps, quadratureRes,
          magnitudeBounds, penaltyWeights, physicalParams,
          gcopter_config_->dilateRadiusSoft,             // 新增：软启动外径（应 ≥ hard）
          gcopter_config_->weightPosSoft)) {             // 新增：软通道权重
    std::cout << "\n\n\n\n\n\n\n\nsetup failed!" << std::endl;
    return false;
  }


// ────────────────────── 段落 C：flyToSafeRegion 空点云保护 ──────────────────────
//
// 原 611-616 行：
//   Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
//       surf_points[0].data(), 3, surf_points.size());
//   Eigen::MatrixX4d hp;
//   firi::firi(bd, pc, ...);
//
// 替换为：

  const double *pc_data =
      surf_points.empty() ? nullptr : surf_points[0].data();
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
      pc_data, 3, static_cast<int>(surf_points.size()));
  Eigen::MatrixX4d hp;
  if (!firi::firi(bd, pc,
                  topo_graph_->odom_node_->center_.cast<double>(),
                  topo_graph_->odom_node_->center_.cast<double>(),
                  hp)) {
    ROS_ERROR("flyToSafeRegion: firi failed");           // 修复：检查 firi 返回值
    return false;
  }


// ────────────────────── 段落 D：flyToSafeRegion 的 gcopter.setup() 调用 ──────────────────────
//
// 在 flyToSafeRegion 后半段同样有一处 gcopter.setup(...) 调用，参考 planExploreTraj
// 的改法做同样补传：

  if (!gcopter.setup(
          gcopter_config_->weightT,
          gcopter_config_->dilateRadiusHard,             // 修正
          iniState, finState, hPolys, INFINITY,
          gcopter_config_->smoothingEps, quadratureRes,
          magnitudeBounds, penaltyWeights, physicalParams,
          gcopter_config_->dilateRadiusSoft,             // 新增
          gcopter_config_->weightPosSoft)) {             // 新增
    std::cout << "\n\n\n\n\n\n\n\nsetup failed (flyToSafeRegion)!" << std::endl;
    return false;
  }
