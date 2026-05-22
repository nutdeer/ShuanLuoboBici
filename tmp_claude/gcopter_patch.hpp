// ============================================================================
// 替换源文件: epic/local_planner/minco_planner/include/gcopter/gcopter.hpp
// 替换段落 :
//   段落 A — 类成员声明（约 78-83 行附近，dilate_radius_ 那一块）
//   段落 B — attachPenaltyFunctional 的函数签名 + 位置惩罚那一段
//            （约 342-347 行 函数签名 + 436-447 行 位置惩罚）
//   段落 C — costFunctional 调用 attachPenaltyFunctional 处
//            （约 593-596 行）
//   段落 D — setup() 函数签名与函数体
//            （约 844-862 行的开头几行）
// 修复内容 :
//   - 增加双通道软约束（pseudo-Huber），与 EGO-Planner v2 风格对齐。
//   - setup 增加 dilate_radius_soft 与 weight_pos_soft 两个参数。
//   - 类成员保留原 dilate_radius_（兼容），新增 soft_extra_ 与 weightPosSoft_。
// ============================================================================

// ────────────────────── 段落 A：类私有成员（替换原来的 79 行附近） ──────────────────────
//
//   原:
//     double dilate_radius_ = 0.2;
//     double smoothEps;
//     ...
//
//   改为：
//
private:
  // ... 上面其它成员保持不变 ...
  double dilate_radius_ = 0.2;     // hard radius，保留（用于历史接口兼容）
  double soft_extra_   = 0.0;      // soft - hard，>=0；软通道在已收缩边界外延 soft_extra_ 内开始触发
  double weightPosSoft_ = 0.0;     // 软通道权重
  double smoothEps;
  // ... 其余保持不变 ...


// ────────────────────── 段落 B：attachPenaltyFunctional ──────────────────────
//
//   完整函数签名替换为：
//
  static inline void attachPenaltyFunctional(
      const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
      const Eigen::VectorXi &hIdx, const PolyhedraH &hPolys,
      const double &smoothFactor, const int &integralResolution,
      const Eigen::VectorXd &magnitudeBounds,
      const Eigen::VectorXd &penaltyWeights, flatness::FlatnessMap &flatMap,
      double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC,
      double &dilate_radius_,                   // 保留参数（即使函数体不再使用，避免破坏 ABI）
      const double &soft_extra,                 // 新增：软启动距离
      const double &weight_pos_soft) {          // 新增：软通道权重
    const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
    const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
    const double thetaMax  = magnitudeBounds(2);
    const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
    const double thrustRadi = 0.5 * std::fabs(magnitudeBounds(4) - magnitudeBounds(3));
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos    = penaltyWeights(0);
    const double weightVel    = penaltyWeights(1);
    const double weightOmg    = penaltyWeights(2);
    const double weightTheta  = penaltyWeights(3);
    const double weightThrust = penaltyWeights(4);

    // pseudo-Huber 拐点：与 EGO-Planner 一致取 5cm
    constexpr double kSoftHuberR    = 0.05;
    constexpr double kSoftHuberRSqr = kSoftHuberR * kSoftHuberR;

    // ... （原函数体里的局部变量声明、 beta0_xy / beta1_xy / ... 全部保留不变）...
    // 直到位置惩罚那一段：

    // ──────────── 位置惩罚（双通道：硬 smoothedL1 + 软 pseudo-Huber） ────────────
    L = hIdx(i);
    K = hPolys[L].rows();
    for (int k = 0; k < K; k++) {
      outerNormal = hPolys[L].block<1, 3>(k, 0);
      // 注：setup 里已对 hPolys 行作单位归一化，所以 violaPos 直接是有符号距离
      violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);

      // ---- 硬通道：维持原 smoothedL1 实现 ----
      if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
        gradPos += weightPos * violaPosPenaD * outerNormal;
        pena    += weightPos * violaPosPena;
      }

      // ---- 软通道：pseudo-Huber，向内延伸 soft_extra ----
      if (weight_pos_soft > 0.0 && soft_extra > 0.0) {
        double violaPosSoft = violaPos + soft_extra;     // 软启动距离
        if (violaPosSoft > 0.0) {
          double term = std::sqrt(1.0 + violaPosSoft * violaPosSoft / kSoftHuberRSqr);
          // 梯度：weight_pos_soft * (violaPosSoft/term) * outerNormal，term→∞ 时饱和到 weight_pos_soft·sign
          gradPos += weight_pos_soft * (violaPosSoft / term) * outerNormal;
          pena    += weight_pos_soft * kSoftHuberRSqr * (term - 1.0);
        }
      }
    }
    // ──────────── 位置惩罚结束 ────────────

    // ... 后面 violaVel / violaOmg / violaTheta / violaThrust 那几段保持不变 ...
  }


// ────────────────────── 段落 C：costFunctional 调用处 ──────────────────────
//
//   原:
//     attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx, obj.hPolytopes,
//                             obj.smoothEps, obj.integralRes, obj.magnitudeBd, obj.penaltyWt,
//                             obj.flatmap, cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
//                             obj.dilate_radius_);
//   改为：
//
    attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx, obj.hPolytopes,
                            obj.smoothEps, obj.integralRes, obj.magnitudeBd, obj.penaltyWt,
                            obj.flatmap, cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
                            obj.dilate_radius_,
                            obj.soft_extra_,                  // 新增
                            obj.weightPosSoft_);              // 新增


// ────────────────────── 段落 D：setup() ──────────────────────
//
//   函数签名替换为（增加最末两个参数；老调用方可传 0 来保持单通道行为）：
//
  inline bool setup(const double &timeWeight,
                    const double &dilate_radius,                     // hard
                    const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                    const Eigen::Matrix<double, 3, 4> &terminalPVAJ,
                    const PolyhedraH &safeCorridor,
                    const double &lengthPerPiece,
                    const double &smoothingFactor,
                    const int &integralResolution,
                    const Eigen::VectorXd &magnitudeBounds,
                    const Eigen::VectorXd &penaltyWeights,
                    const Eigen::VectorXd &physicalParams,
                    const double &dilate_radius_soft = 0.0,           // 新增：软启动外径
                    const double &weight_pos_soft    = 0.0) {         // 新增：软通道权重
    dilate_radius_   = dilate_radius;                                 // hard，保留
    soft_extra_      = std::max(0.0, dilate_radius_soft - dilate_radius);
    weightPosSoft_   = std::max(0.0, weight_pos_soft);
    rho              = timeWeight;
    headPVAJ         = initialPVAJ;
    tailPVAJ         = terminalPVAJ;

    hPolytopes = safeCorridor;
    for (size_t i = 0; i < hPolytopes.size(); i++) {
      const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();
      hPolytopes[i].array().colwise() /= norms;
    }
    if (!processCorridor(hPolytopes, vPolytopes)) {
      return false;
    }

    // ... 函数体后续部分保持不变 ...
    // (polyN = hPolytopes.size(); ... 直到 return true;)
  }
