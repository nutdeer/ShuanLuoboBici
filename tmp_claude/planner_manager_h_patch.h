// ============================================================================
// 替换源文件: epic/local_planner/minco_planner/include/plan_manage/planner_manager.h
// 替换段落 : struct GcopterConfig { ... } 整段
//            （约 29-81 行）
// 修复内容 : 增加 weightPosSoft 字段 + 读取 ROS 参数 WeightPosSoft
// ============================================================================

struct GcopterConfig {
  std::string mapTopic;
  std::string targetTopic;
  double dilateRadiusSoft, dilateRadiusHard;
  double timeoutRRT;
  double maxVelMag;
  double maxBdrMag;
  double maxTiltAngle;
  double minThrust;
  double maxThrust;
  double vehicleMass;
  double gravAcc;
  double horizDrag;
  double vertDrag;
  double parasDrag;
  double speedEps;
  double weightT;
  double WeightSafeT;
  std::vector<double> chiVec;
  double weightPosSoft = 0.0;        // 新增：软通道权重，默认 0 表示沿用单通道
  double smoothingEps;
  int integralIntervs;
  double relCostTol;
  double corridor_size;
  double yaw_max_vel;
  double yaw_rho_vis;
  double yaw_time_fwd;

  void init(const ros::NodeHandle &nh_priv) {
    nh_priv.getParam("DilateRadiusSoft", dilateRadiusSoft);
    nh_priv.getParam("DilateRadiusHard", dilateRadiusHard);
    nh_priv.getParam("MaxVelMag", maxVelMag);
    nh_priv.getParam("maxBdrMag", maxBdrMag);
    nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
    nh_priv.getParam("MinThrust", minThrust);
    nh_priv.getParam("MaxThrust", maxThrust);
    nh_priv.getParam("VehicleMass", vehicleMass);
    nh_priv.getParam("GravAcc", gravAcc);
    nh_priv.getParam("HorizDrag", horizDrag);
    nh_priv.getParam("VertDrag", vertDrag);
    nh_priv.getParam("ParasDrag", parasDrag);
    nh_priv.getParam("SpeedEps", speedEps);
    nh_priv.getParam("WeightT", weightT);
    nh_priv.getParam("WeightSafeT", WeightSafeT);
    nh_priv.getParam("ChiVec", chiVec);
    nh_priv.param("WeightPosSoft", weightPosSoft, 0.0);   // 新增；缺省 0 → 软通道关闭
    nh_priv.getParam("SmoothingEps", smoothingEps);
    nh_priv.getParam("IntegralIntervs", integralIntervs);
    nh_priv.getParam("RelCostTol", relCostTol);
    nh_priv.getParam("MaxCorridorSize", corridor_size);
    nh_priv.getParam("yaw_rho_vis", yaw_rho_vis);
    nh_priv.getParam("yaw_max_vel", yaw_max_vel);
    nh_priv.getParam("yaw_time_fwd", yaw_time_fwd);
  }
};
