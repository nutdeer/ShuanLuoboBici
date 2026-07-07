/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

// #include <ompl/base/DiscreteMotionValidator.h>
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/geometric/planners/rrt/InformedRRTstar.h>
// #include <ompl/util/Console.h>

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <misc/visualizer.hpp>

#include "firi.hpp"
#include "geo_utils.hpp"

namespace sfc_gen {

// template <typename Map>
// inline double planPath(const Eigen::Vector3d &s, const Eigen::Vector3d &g, const Eigen::Vector3d &lb, const Eigen::Vector3d &hb, const Map *mapPtr, const double &timeout,
//                        std::vector<Eigen::Vector3d> &p) {
//   auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

//   ompl::base::RealVectorBounds bounds(3);
//   bounds.setLow(0, 0.0);
//   bounds.setHigh(0, hb(0) - lb(0));
//   bounds.setLow(1, 0.0);
//   bounds.setHigh(1, hb(1) - lb(1));
//   bounds.setLow(2, 0.0);
//   bounds.setHigh(2, hb(2) - lb(2));
//   space->setBounds(bounds); // 给ompl设置搜索范围

//   auto si(std::make_shared<ompl::base::SpaceInformation>(space));

//   si->setStateValidityChecker([&](const ompl::base::State *state) {
//     const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
//     const Eigen::Vector3d position(lb(0) + (*pos)[0], lb(1) + (*pos)[1], lb(2) + (*pos)[2]);
//     return mapPtr->query(position) == 0;
//   }); // 检查是否是free
//   si->setup();

//   ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

//   ompl::base::ScopedState<> start(space), goal(space);
//   start[0] = s(0) - lb(0);
//   start[1] = s(1) - lb(1);
//   start[2] = s(2) - lb(2);
//   goal[0] = g(0) - lb(0);
//   goal[1] = g(1) - lb(1);
//   goal[2] = g(2) - lb(2);

//   auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
//   pdef->setStartAndGoalStates(start, goal);
//   pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
//   auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
//   planner->setProblemDefinition(pdef);
//   planner->setup();

//   ompl::base::PlannerStatus solved;
//   solved = planner->ompl::base::Planner::solve(timeout);

//   double cost = INFINITY;
//   if (solved) {
//     p.clear();
//     const ompl::geometric::PathGeometric path_ = ompl::geometric::PathGeometric(dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
//     for (size_t i = 0; i < path_.getStateCount(); i++) {
//       const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//       p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
//     }
//     cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
//   }

//   return cost;
// }

/**
 * Generates the convex cover for a given path and set of points.
 *
 * @param path         the path represented as a vector of 3D vectors
 * @param points       map: point cloud
 * @param lowCorner    the lower corner of the bounding box represented as a 3D
 * vector
 * @param highCorner   the upper corner of the bounding box represented as a 3D
 * vector
 * @param progress     沿路径切分步长 as a double value 7
 * @param range        the range as a double value 3  每个凸多面体的搜索范围
 * @param hpolys       the vector of 4x4 matrices representing the convex  输出：凸多面体序列（每个是 MatrixX4d，半空间表示）
 * polygons
 * @param eps          the epsilon value as a double (optional, default value
 * is 1.0e-6)
 *
 * @throws dilate_radius_   硬约束半径
 */
inline void convexCover(const std::unique_ptr<Visualizer> &vizer, const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lowCorner, const Eigen::Vector3d &highCorner, const double &progress, const double &range, std::vector<Eigen::MatrixX4d> &hpolys,
                        const double eps = 1.0e-6,
                        const double dilate_radius_ = 0.1,
                        
                        const std::vector<std::vector<Eigen::Vector3d>>* voxel_raw = nullptr,  // 这个是下采样的原试点表
                        double voxel_radius = 0.1,   // 这个是最大可能嵌入的深度
                        const double drone_r = 0.1  // 飞机半径  
                      ) {
  // hpolys.clear();
  const int n = path.size();
  // 矩阵 6 个面
  // 这里会用小矩形再挑选一遍点
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;

  Eigen::MatrixX4d hp, gap;
  Eigen::Vector3d a, b = path[0];
  std::vector<Eigen::Vector3d> valid_pc;
  std::vector<Eigen::Vector3d> bs;
  valid_pc.reserve(points.size());
  // valid_pc_raw[j]  第 j 个下采样质心对应的原始点集合
  /*
  ex
  valid_pc_raw[0] = {
    Eigen::Vector3d(1.0, 2.0, 0.5),
    Eigen::Vector3d(1.1, 2.0, 0.5),
    Eigen::Vector3d(0.9, 2.1, 0.6)
  };

  valid_pc_raw[1] = {
    Eigen::Vector3d(4.0, 1.0, 0.2),
    Eigen::Vector3d(4.1, 1.1, 0.2)
  };
  */
  std::vector<std::vector<Eigen::Vector3d>> valid_pc_raw;
  if(voxel_raw) valid_pc_raw.reserve(points.size());  // 分最外层尺寸

  auto shrink_hp = [&](Eigen::MatrixX4d &hp, double radius) {
    hp.col(3) = hp.col(3).array() + radius * hp.leftCols(3).rowwise().norm().array();
  };

  for (int i = 1; i < n;) {
    a = b;
    // 路径太长了，沿着方向拓展最大距离progress
    if ((a - path[i]).norm() > progress) {  // 就是按 7 的距离切分路径
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
    if (voxel_raw) valid_pc_raw.clear();

    // zwx test
    static long double num_zwx_test = 1.0;
    static long double num_zwx_test_remake_because_b = 1.0;

    // 这里是对下采样再做一个小框筛选
    for (const Eigen::Vector3d &p : points) {  // 同时检查 6 个面
      if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
        // 小于0表示点在某一个框里，可以用ikd-tree的接口代替这个函数，利用bd进行box-select
        valid_pc.emplace_back(p);
      }
    } // 筛选 bd 中的点

    for (size_t k=0; k< points.size(); ++k)
    {
      const Eigen::Vector3d &p = points[k];  // 只读
      if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < voxel_radius)  // 向外拓宽 一个误差半径
      {
        valid_pc.emplace_back(p);
        if (voxel_raw) valid_pc_raw.emplace_back(voxel_raw->at(k));
      }

    }  // 筛选出来 在这个小空间里面会用到的点

    // 如果box没有点云，valid_pc 是空的，valid_pc[0]非法
    const double *data_tmp = valid_pc.empty() ? nullptr : valid_pc[0].data();
    const std::vector<std::vector<Eigen::Vector3d>>* pc_raw_ptr = voxel_raw ? &valid_pc_raw : nullptr;
    // 转换下格式发给FIRI
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(data_tmp, 3, valid_pc.size());  
    
    firi::firi(bd, pc, a, b, hp, 4, eps, pc_raw_ptr, drone_r); // 计算出包含a和b的凸包 ，就是必须包含a和b,这样a和b的路径也都在飞行走廊里了
    // const int M = bd.rows();  // 边界面数量 M = bd.rows() 6 ; N = pc.cols(); 障碍点数量
    // 0.2 下采样是 pc 2000 个点
    // ROS_WARN_STREAM_THROTTLE(1.0, "FUCK EVERYONE:" << pc.cols());
    // Eigen::MatrixX4d hp_origin = hp;
    // 将凸包向里收缩，收缩大小为膨胀半径
    shrink_hp(hp, dilate_radius_);

    Eigen::Vector4d bh(b(0), b(1), b(2), 1.0); // 其次坐标 b 

    // 适当放宽条件，不能没有可行解
    num_zwx_test +=1.0;
    // 如果 b 不在当前多面体内部
    if (((hp * bh).array() > -eps).cast<int>().sum() > 0) {
      firi::firi(bd, pc, a, a, hp, 1);
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      hpolys.emplace_back(hp);
      firi::firi(bd, pc, (a + b) / 2.0, (a + b) / 2.0, hp, 1);
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      hpolys.emplace_back(hp);
      firi::firi(bd, pc, b, b, hp, 1);
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      hpolys.emplace_back(hp);

      num_zwx_test_remake_because_b += 1.0;
      ROS_WARN_STREAM_THROTTLE(1.0, "[SFC gen] b outside poly,but have to use. re_firi proportion= " << num_zwx_test_remake_because_b/num_zwx_test *100.0 << "%");

    }
    // 补救措施： 
    if (hpolys.size() != 0) {  // 防止空vector 最后一个多面体
      const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
      // 如果是 a 不在
      if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() + ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
        firi::firi(bd, pc, a, a, gap, 1);
        hpolys.emplace_back(gap);
      }
    }

    hpolys.emplace_back(hp);
  }
}

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys) {
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

#endif
