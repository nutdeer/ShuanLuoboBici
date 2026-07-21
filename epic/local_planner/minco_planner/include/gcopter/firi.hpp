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

/* This is an old version of FIRI for temporary usage here. */

#ifndef FIRI_HPP
#define FIRI_HPP

#include "lbfgs.hpp"
#include "sdlp.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace firi {

inline void chol3d(const Eigen::Matrix3d& A, Eigen::Matrix3d& L) {
	L(0, 0) = sqrt(A(0, 0));
	L(0, 1) = 0.0;
	L(0, 2) = 0.0;
	L(1, 0) = 0.5 * (A(0, 1) + A(1, 0)) / L(0, 0);
	L(1, 1) = sqrt(A(1, 1) - L(1, 0) * L(1, 0));
	L(1, 2) = 0.0;
	L(2, 0) = 0.5 * (A(0, 2) + A(2, 0)) / L(0, 0);
	L(2, 1) = (0.5 * (A(1, 2) + A(2, 1)) - L(2, 0) * L(1, 0)) / L(1, 1);
	L(2, 2) = sqrt(A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));
	return;
}

inline bool smoothedL1(const double& mu, const double& x, double& f,
                       double& df) {
	if (x < 0.0) {
		return false;
	} else if (x > mu) {
		f = x - 0.5 * mu;
		df = 1.0;
		return true;
	} else {
		const double xdmu = x / mu;
		const double sqrxdmu = xdmu * xdmu;
		const double mumxd2 = mu - 0.5 * x;
		f = mumxd2 * sqrxdmu * xdmu;
		df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
		return true;
	}
}

inline double costMVIE(void* data, const Eigen::VectorXd& x,
                       Eigen::VectorXd& grad) {
	const int64_t* pM = (int64_t*)data;
	const double* pSmoothEps = (double*)(pM + 1);
	const double* pPenaltyWt = pSmoothEps + 1;
	const double* pA = pPenaltyWt + 1;

	const int M = *pM;
	const double smoothEps = *pSmoothEps;
	const double penaltyWt = *pPenaltyWt;
	Eigen::Map<const Eigen::MatrixX3d> A(pA, M, 3);
	Eigen::Map<const Eigen::Vector3d> p(x.data());
	Eigen::Map<const Eigen::Vector3d> rtd(x.data() + 3);
	Eigen::Map<const Eigen::Vector3d> cde(x.data() + 6);
	Eigen::Map<Eigen::Vector3d> gdp(grad.data());
	Eigen::Map<Eigen::Vector3d> gdrtd(grad.data() + 3);
	Eigen::Map<Eigen::Vector3d> gdcde(grad.data() + 6);

	double cost = 0;
	gdp.setZero();
	gdrtd.setZero();
	gdcde.setZero();

	Eigen::Matrix3d L;
	L(0, 0) = rtd(0) * rtd(0) + DBL_EPSILON;
	L(0, 1) = 0.0;
	L(0, 2) = 0.0;
	L(1, 0) = cde(0);
	L(1, 1) = rtd(1) * rtd(1) + DBL_EPSILON;
	L(1, 2) = 0.0;
	L(2, 0) = cde(2);
	L(2, 1) = cde(1);
	L(2, 2) = rtd(2) * rtd(2) + DBL_EPSILON;

	const Eigen::MatrixX3d AL = A * L;
	const Eigen::VectorXd normAL = AL.rowwise().norm();  // rowwise() 是逐行
	const Eigen::Matrix3Xd adjNormAL =
	    (AL.array().colwise() / normAL.array()).transpose();
	const Eigen::VectorXd consViola = (normAL + A * p).array() - 1.0;

	double c, dc;
	Eigen::Vector3d vec;
	for (int i = 0; i < M; ++i) {
		if (smoothedL1(smoothEps, consViola(i), c, dc)) {
			cost += c;
			vec = dc * A.row(i).transpose();
			gdp += vec;
			gdrtd += adjNormAL.col(i).cwiseProduct(vec);
			gdcde(0) += adjNormAL(0, i) * vec(1);
			gdcde(1) += adjNormAL(1, i) * vec(2);
			gdcde(2) += adjNormAL(0, i) * vec(2);
		}
	}
	cost *= penaltyWt;
	gdp *= penaltyWt;
	gdrtd *= penaltyWt;
	gdcde *= penaltyWt;

	cost -= log(L(0, 0)) + log(L(1, 1)) + log(L(2, 2));
	gdrtd(0) -= 1.0 / L(0, 0);
	gdrtd(1) -= 1.0 / L(1, 1);
	gdrtd(2) -= 1.0 / L(2, 2);

	gdrtd(0) *= 2.0 * rtd(0);
	gdrtd(1) *= 2.0 * rtd(1);
	gdrtd(2) *= 2.0 * rtd(2);

	return cost;
}

// Each row of hPoly is defined by h0, h1, h2, h3 as
// h0*x + h1*y + h2*z + h3 <= 0
// R, p, r are ALWAYS taken as the initial guess
// R is also assumed to be a rotation matrix
inline bool maxVolInsEllipsoid(const Eigen::MatrixX4d& hPoly,
                               Eigen::Matrix3d& R, Eigen::Vector3d& p,
                               Eigen::Vector3d& r) {
	// Find the deepest interior point
	const int M = hPoly.rows();
	Eigen::MatrixX4d Alp(M, 4);
	Eigen::VectorXd blp(M);
	Eigen::Vector4d clp, xlp;
	const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
	Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
	Alp.rightCols<1>().setConstant(1.0);
	blp = -hPoly.rightCols<1>().array() / hNorm;
	clp.setZero();
	clp(3) = -1.0;
	const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
	if (!(maxdepth > 0.0) || std::isinf(maxdepth)) {
		return false;
	}
	const Eigen::Vector3d interior = xlp.head<3>();

	// Prepare the data for MVIE optimization
	uint8_t* optData =
	    new uint8_t[sizeof(int64_t) + (2 + 3 * M) * sizeof(double)];
	int64_t* pM = (int64_t*)optData;
	double* pSmoothEps = (double*)(pM + 1);
	double* pPenaltyWt = pSmoothEps + 1;
	double* pA = pPenaltyWt + 1;

	*pM = M;
	Eigen::Map<Eigen::MatrixX3d> A(pA, M, 3);
	A = Alp.leftCols<3>().array().colwise() /
	    (blp - Alp.leftCols<3>() * interior).array();

	Eigen::VectorXd x(9);
	const Eigen::Matrix3d Q =
	    R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
	Eigen::Matrix3d L;
	chol3d(Q, L);

	x.head<3>() = p - interior;
	x(3) = sqrt(L(0, 0));
	x(4) = sqrt(L(1, 1));
	x(5) = sqrt(L(2, 2));
	x(6) = L(1, 0);
	x(7) = L(2, 1);
	x(8) = L(2, 0);

	double minCost;
	lbfgs::lbfgs_parameter_t paramsMVIE;
	paramsMVIE.mem_size = 18;
	paramsMVIE.g_epsilon = 0.0;
	paramsMVIE.min_step = 1.0e-16;
	paramsMVIE.past = 3;
	paramsMVIE.delta = 1.0e-7;
	*pSmoothEps = 1.0e-2;
	*pPenaltyWt = 1.0e+3;

	int ret = lbfgs::lbfgs_optimize(x, minCost, &costMVIE, nullptr, nullptr,
	                                optData, paramsMVIE);

	if (ret < 0) {
		printf("FIRI WARNING: %s\n", lbfgs::lbfgs_strerror(ret));
	}

	p = x.head<3>() + interior;
	L(0, 0) = x(3) * x(3);
	L(0, 1) = 0.0;
	L(0, 2) = 0.0;
	L(1, 0) = x(6);
	L(1, 1) = x(4) * x(4);
	L(1, 2) = 0.0;
	L(2, 0) = x(8);
	L(2, 1) = x(7);
	L(2, 2) = x(5) * x(5);
	Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner>
	    svd(L, Eigen::ComputeFullU);
	const Eigen::Matrix3d U = svd.matrixU();
	const Eigen::Vector3d S = svd.singularValues();
	if (U.determinant() < 0.0) {
		R.col(0) = U.col(1);
		R.col(1) = U.col(0);
		R.col(2) = U.col(2);
		r(0) = S(1);
		r(1) = S(0);
		r(2) = S(2);
	} else {
		R = U;
		r = S;
	}

	delete[] optData;

	return ret >= 0;
}

/**
 * Calculates the firi algorithm.
 *
 * @param bd the matrix bd
 * @param pc the matrix pc
 * @param a the vector
 * @param b the vector
 * @param hPoly the output matrix hPoly
 * @param iterations the number of iterations (default: 4)
 * @param epsilon the epsilon value (default: 1.0e-6)
 * @note 生成包含a b 不包含pc的最大凸包
 * @return true if the algorithm completes successfully, false otherwise
 *   a 和 b 是凸包必须包含的两个"锚点"。
 * deepseek
  凸多面体包含 a 和 b → 由凸性，整个线段 [a, b] 都在里面 →
  保证走廊沿路径方向连通。
  算法每轮迭代的切线生成中（第 292-316 行），核心逻辑是三层兜底：
  对每个障碍物点，在单位球上做切平面：
    第1层：检查切平面是否切掉了 a？
           ↓ 是
           调整切平面，让它刚好擦过 a，但仍然把障碍物挡在外面
    第2层：再检查是否切掉了 b？
           ↓ 是
           调整切平面，让它刚好擦过 b
    第3层：如果调整后仍然切掉了 a？
           ↓ 是
           用 a、b、障碍物点三个点确定一个平面，
           即法向量 = (a - 障碍物) × (b - 障碍物)
           保证 a 和 b 都在平面正确的一侧
  这三层检查就是为了无论如何都不能把 a 或 b 排除在凸包之外。
 */
inline bool firi(const Eigen::MatrixX4d& bd, const Eigen::Matrix3Xd& pc,
                 const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                 Eigen::MatrixX4d& hPoly, const int iterations = 4,
                 const double epsilon = 1.0e-6,
				// 增加内嵌的障碍点的移除
				const std::vector<std::vector<Eigen::Vector3d>>* pc_raw = nullptr,
				double drone_r = 0.0,  // 需显示调用
				// pc_ext[i]: 体素 i 内 raw 相对质心 pc.col(i) 的轴向半宽（世界系，逐轴>=0）
				// 支撑值 s_i(n̂) = |n̂|·pc_ext[i] 是该体素全部 raw 沿 n̂ 偏移的上界
				const std::vector<Eigen::Vector3d>* pc_ext = nullptr,
				// raw 深度容差：允许 raw 到最终走廊的距离降到 drone_r - delta_tol
				const double delta_tol = 0.0
				) {
	const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
	const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

	// boundary约束 bd * ah <=0
	if ((bd * ah).maxCoeff() > 0.0 || (bd * bh).maxCoeff() > 0.0) {
		return false; // a and b are out of boundary
	}

	const int M = bd.rows();  // M is aabb relate
	const int N = pc.cols();

	Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // 椭球三个旋转
	Eigen::Vector3d p = 0.5 * (a + b); // 椭球中心
	Eigen::Vector3d r = Eigen::Vector3d::Ones(); // 椭球三个长轴的半径
	Eigen::MatrixX4d forwardH(M + N, 4);
	int nH = 0;

	for (int loop = 0; loop < iterations; ++loop) {
		const Eigen::Matrix3d forward = r.cwiseInverse().asDiagonal() * R.transpose(); 
		const Eigen::Matrix3d backward = R * r.asDiagonal();                           
		const Eigen::MatrixX3d forwardB = bd.leftCols<3>() * backward;  // 这两个也是 aabb relate
		const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<3>() * p;
		const Eigen::VectorXd bd_margin = drone_r * bd.leftCols<3>().rowwise().norm();  // 只在选面时使用

		const Eigen::Matrix3Xd forwardPC = forward * (pc.colwise() - p); 

		const Eigen::Vector3d fwd_a = forward * (a - p); 
		const Eigen::Vector3d fwd_b = forward * (b - p);

		const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());
		Eigen::MatrixX4d tangents(N, 4); 

		Eigen::VectorXd distRs(N);
		
		if( loop == iterations -1 ) 
		{
			// 0 is fast ; 1 is slow  
			Eigen::Matrix<uint8_t, -1, 1> slowFlags = Eigen::Matrix<uint8_t, -1, 1>::Zero(N);
			int fast_count = 0;
			int slow_count = 0;

			Eigen::Matrix<uint8_t, -1, 1> cutAFlags = Eigen::Matrix<uint8_t, -1, 1>::Zero(N);
			Eigen::Matrix<uint8_t, -1, 1> cutBFlags = Eigen::Matrix<uint8_t, -1, 1>::Zero(N);

			// 切线生成核心
			// 带 fwd 的都是 球面空间
			for (int i = 0; i < N; i++) 
			{   // 每一个障碍点的遍历
				const Eigen::Vector3d pt_e = forwardPC.col(i);
				const double dist = pt_e.norm(); 
				distRs(i) = dist;
				if (dist < epsilon) 
				{
					ROS_WARN_STREAM("[FIRI] division zero error likely happen. because distRs.");
					return false;
				}

				const Eigen::Vector3d n_e = pt_e / dist;  // 平面法向量
				const double d_e = -dist;

				// 上面的两段代码就实现了球空间的 初始切面
				// n_w: 世界系法向（未归一化）。世界距离 δ_w 对应球空间函数值 δ_w*‖n_w‖
				const Eigen::Vector3d n_w = forward.transpose() * n_e;
				const double n_w_norm = n_w.norm();
				if (!(n_w_norm > epsilon)) {
					return false;
				}
				// 体素支撑界：该体素全部 raw 沿 -n̂_w 方向最多比质心深入 s_i，
				// 把 max(0, s_i - delta_tol) 叠进本面的内缩量，支撑体素的 raw
				// 就直接满足 drone_r - delta_tol，无需事后修复
				double margin_world = drone_r;
				if (pc_ext != nullptr) {
					const double support =
						n_w.cwiseAbs().dot((*pc_ext)[i]) / n_w_norm;
					margin_world += std::max(0.0, support - delta_tol);
				}
				const double direct_margin = margin_world * n_w_norm;
				// fast-slow check 
				const bool keep_fwd_a = n_e.dot(fwd_a) + d_e + direct_margin <= epsilon;
				const bool keep_fwd_b = n_e.dot(fwd_b) + d_e + direct_margin <= epsilon;
				const bool fast = keep_fwd_a && keep_fwd_b;
				cutAFlags(i) = keep_fwd_a ? 0 : 1;
				cutBFlags(i) = keep_fwd_b ? 0 : 1;
				if (fast)
				{
					fast_count++;
					tangents.block<1, 3>(i, 0) = n_e.transpose();
					tangents(i, 3) = d_e + direct_margin;
					continue;
				}
				
				// 接下来就是比较难处理的 slow 点
				// slow 不应该通过
				slow_count++;
				slowFlags(i)=1;
				const Eigen::Vector3d margin_pt_e = pt_e - direct_margin * n_e;

				tangents.block<1, 3>(i, 0) = n_e.transpose();
				tangents(i, 3) = d_e ;
				// 是 a 被切
				if (not keep_fwd_a)
				{
					// 所以应该重新构造平面
					// 利用 directionmargin point做
					const Eigen::Vector3d delta = margin_pt_e - fwd_a;
					const double dist_delta = delta.squaredNorm();
					if (dist_delta > epsilon)
					{
						tangents.block<1,3>(i,0) = (fwd_a - (delta.dot(fwd_a) / dist_delta) * delta).transpose();
						distRs(i) = tangents.block<1,3>(i,0).norm();
						tangents(i,3) = -distRs(i);
						tangents.block<1,3>(i,0) /= distRs(i);
					}
					// 只要障碍点应该在内侧，就说明保护球传过了a b，生成失效
					const double obs_side = tangents.block<1,3>(i, 0).dot(pt_e) + tangents(i,3);
					if(obs_side <= epsilon)
					{
						ROS_WARN_STREAM(
							"[FIRI] new tangent failed: pt_e is inside, fallback to 3-point plane. "
							<< "obs_side=" << obs_side
							<< ", loop=" << loop);
						// 让外面的三单点firi兜底
						return false;
					}
				}
				
				// 如果是 b 被切
				const double is_b_safe = tangents.block<1,3>(i, 0).dot(fwd_b) + tangents(i,3);
				if(is_b_safe >= epsilon)
				{
					// 说明 b 被切到外面了
					const Eigen::Vector3d delta = margin_pt_e - fwd_b;
					const double dist_delta = delta.squaredNorm();
					if (dist_delta > epsilon)
					{
						tangents.block<1,3>(i,0) = (fwd_b - (delta.dot(fwd_b) / dist_delta) * delta).transpose();
						distRs(i) = tangents.block<1,3>(i,0).norm();
						tangents(i,3) = -distRs(i);
						tangents.block<1,3>(i,0) /= distRs(i);
					}
					// 只要障碍点应该在内侧，就说明保护球传过了a b，生成失效
					const double obs_side = tangents.block<1,3>(i, 0).dot(pt_e) + tangents(i,3);
					if(obs_side <= epsilon)
					{
						ROS_WARN_STREAM(
							"[FIRI] new tangent failed: pt_e is inside, fallback to 3-point plane. "
							<< "obs_side=" << obs_side
							<< ", loop=" << loop);
						// 让外面的三单点firi兜底
						return false;
					}
				}

				// a又被切了
				const double is_a_safe = tangents.block<1,3>(i, 0).dot(fwd_a) + tangents(i,3);
				if(is_a_safe >= epsilon)
				{
					// 平行 a-b 且过 margin_pt_e ，垂直与 a-b-margin_pt_e的平面
					const Eigen::Vector3d ab = fwd_b - fwd_a;
					const Eigen::Vector3d am = margin_pt_e - fwd_a;

					// a, b, margin_pt_e 三点平面的法向
					const Eigen::Vector3d normal_a_b_pt_e = ab.cross(am);
					Eigen::Vector3d n_raw = ab.cross(normal_a_b_pt_e);
					const double n_norm = n_raw.norm();
					if (n_norm <= epsilon) 
					{
					ROS_WARN_STREAM(
						"[FIRI] margin fallback plane degenerate. "
						<< "n_norm=" << n_norm
						<< ", loop=" << loop);
					return false;
					}
					Eigen::Vector3d n_e_new = n_raw / n_norm;
					double d = -n_e_new.dot(margin_pt_e);  // 黑塞标准型

					const double side_a = n_e_new.dot(fwd_a) + d;  // fwd_a 到平面的有符距离
					if(side_a > epsilon)
					{
						n_e_new = -n_e_new;
						d = -d;
					}  // 反转平面
					const double side_pt = n_e_new.dot(pt_e) + d;  // 原始障碍物点到平面的有符号距离
					if(side_pt < epsilon)
					{
						// 说明平面把 a b pt_e 弄到面的一侧了，说明就是彻底反转到了另一面，无法保证安全了
						return false;
					}
					
					tangents.block<1, 3>(i, 0) = n_e_new.transpose();
					tangents(i, 3) = d;
					distRs(i) = std::abs(d);
				}

			}
			if (slow_count>0)
			{
				ROS_INFO_STREAM_THROTTLE(1.0,
					"[FIRI] end with fast-slow mode. slow_count = " << slow_count);  // 这个输出做成结流没意义
			}
		}
		else  // 非最后一轮沿用原始 FIRI 切面，用于更新椭球
		{
			// 切线生成核心
			for (int i = 0; i < N; i++) { 
				distRs(i) = forwardPC.col(i).norm();                                    
				tangents(i, 3) = -distRs(i);                                            
				tangents.block<1, 3>(i, 0) = forwardPC.col(i).transpose() / distRs(i);  
				if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon) { 

					const Eigen::Vector3d delta = forwardPC.col(i) - fwd_a;
					tangents.block<1, 3>(i, 0) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta; 
					distRs(i) = tangents.block<1, 3>(i, 0).norm();                                         
					tangents(i, 3) = -distRs(i);
					tangents.block<1, 3>(i, 0) /= distRs(i);
				}
				if (tangents.block<1, 3>(i, 0).dot(fwd_b) + tangents(i, 3) > epsilon) { 
					const Eigen::Vector3d delta = forwardPC.col(i) - fwd_b;
					tangents.block<1, 3>(i, 0) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
					distRs(i) = tangents.block<1, 3>(i, 0).norm();
					tangents(i, 3) = -distRs(i);
					tangents.block<1, 3>(i, 0) /= distRs(i);
				}
				if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon) { 
					tangents.block<1, 3>(i, 0) = (fwd_a - forwardPC.col(i)).cross(fwd_b - forwardPC.col(i)).normalized();
					tangents(i, 3) = -tangents.block<1, 3>(i, 0).dot(fwd_a);
					tangents.row(i) *= tangents(i, 3) > 0.0 ? -1.0 : 1.0;
				}
			}
		}
		// 两类切平面的合并
		Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
		Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

		nH = 0;

		bool completed = false;
		int bdMinId = 0, pcMinId = 0;
		double minSqrD = distDs.minCoeff(&bdMinId);
		double minSqrR;
		if (distRs.size() != 0) {
			minSqrR = distRs.minCoeff(&pcMinId);
		}
		else
		{
			minSqrR = INFINITY;
		}
		for (int i = 0; !completed && i < (M + N); ++i) {
			if (minSqrD < minSqrR) {  // 选 aabb 面。distDs 是 aabb 到中心的距离；minSqrR 是 障碍点到中心的距离
				forwardH.block<1, 3>(nH, 0) = forwardB.row(bdMinId);  // bdMinId 是被选中的bd面编号
				forwardH(nH, 3) = forwardD(bdMinId) + bd_margin(bdMinId);
				// 平面内推但是不使用内推后的距离排序选框而是旧的，因为希望只有在被迫选中时才使用保守面
				// 实际上 所有 aabb框都被选完才会结束； 是 aabb框 维护了走廊的封闭性，所以这里有点小问题可能还需要改
				bdFlags(bdMinId) = 0;
			} else {  // 选 obs_tangent 切面是已经缩放过的
				forwardH.row(nH) = tangents.row(pcMinId);
				pcFlags(pcMinId) = 0;
			}
			// 假设任务已经完成 后面如果又发现没有排除的障碍点就继续处理
			completed = true;
			// --------
			// 找下一个面
			// --------
			minSqrD = INFINITY;
			for (int j = 0; j < M; ++j) {
				if (bdFlags(j)) {
					completed = false;
					if (minSqrD > distDs(j)) {
						bdMinId = j;
						minSqrD = distDs(j);
					}
				}
			}
			minSqrR = INFINITY;
			for (int j = 0; j < N; ++j) {
				if (pcFlags(j)) {
					if (forwardH.block<1, 3>(nH, 0).dot(forwardPC.col(j)) + forwardH(nH, 3) > -epsilon) {
						pcFlags(j) = 0;  // 只要新的平面 （nh） 能把这个点排除在 +eps > 0 就标记
					} else {
						completed = false;
						if (minSqrR > distRs(j)) {  // 否则需要考虑更新
							pcMinId = j;
							minSqrR = distRs(j);
						}
					}
				}
			}
			++nH;
		}
		// 转回世界坐标
		hPoly.resize(nH, 4);
		for (int i = 0; i < nH; ++i) {
			hPoly.block<1, 3>(i, 0) = forwardH.block<1, 3>(i, 0) * forward;
			hPoly(i, 3) = forwardH(i, 3) - hPoly.block<1, 3>(i, 0).dot(p);
		}

		if (loop == iterations - 1) {
			break;
		}

		maxVolInsEllipsoid(hPoly, R, p, r); //最大内接椭球
	}

	// loop 已经完整执行 后处理
	// ---------- 体素支撑界后处理 v3 ----------
	// ---------- voxel support boundary post-processing v3 ----------
	// 这一段是 deepseek 生成的注释：
	// 走廊主体沿用上面的低成本剪枝（面数、耗时与原版一致；不做严格
	// drone_r 逐面剪枝——因为这样会把面数推到 N100 不可用的水平
	// 安全性改由这里保证：对全部质心做一次 O(N*F) 的多面体距离下界检查
	//   dist(q, corridor) >= max_f signed_f(q)
	// 该下界远比“对覆盖它的那张超平面的距离”宽松，绝大多数点一次比较
	// 即提前退出。只有下界不足的极少数点需要修复：
	//   1) 优先平行内推其最近面（不增加面数，只收缩）
	//   2) 内推会切 a/b 时，为该点添加一张线段分离面（严格保 a/b 与半径）
	//   3) 仍不可行（点离 a-b 过近）则返回 false，交外层三单点 FIRI
	// deepseek end
	
	/*for each voxel j {
		best = max signed distance over faces;

		if (best >= t_base + ||ext_j||) {
		continue;  // 保守快速通过
		}

		support = s_j(normal_of_best_face);
		need = t_base + support - best;

		if (need > epsilon) {
		residuals.push_back({j, best_face, need});
		}
		}*/
	if (pc_ext != nullptr && drone_r > epsilon && N > 0) {  // 合法性检查
		const double t_base = drone_r - delta_tol;
		const int base_faces = hPoly.rows();  	 // 记录原面数
		Eigen::VectorXd face_norms(base_faces);  // 记录面法向量长
		for (int f = 0; f < base_faces; ++f) {
			face_norms(f) = hPoly.block<1, 3>(f, 0).norm();
			if (!(face_norms(f) > epsilon)) {
				return false;
			}
		}

		struct RawDeficit {  // 记录不安全面的残差结构 point_index face_index safe_need_dist
			int point;
			int face;
			double need;
		};
		std::vector<RawDeficit> residuals;
		for (int j = 0; j < N; ++j) {
			const Eigen::Vector3d q = pc.col(j);  // 取列 col 
			// s(n̂) = |n̂|·ext ≤ ‖ext‖，用于提前退出
			const double safe_bound = t_base + (*pc_ext)[j].norm();  // 用无关上界先把safe全部排除
			double best = -std::numeric_limits<double>::infinity();
			int f_near = -1;
			for (int f = 0; f < base_faces; ++f) {  // 遍历所有面 如果安全上界都安全那就安全
				const double d =
					(hPoly.block<1, 3>(f, 0).dot(q) + hPoly(f, 3)) /
					face_norms(f);  // d = (n^T*q + d) / ||n||
				if (d > best) {  // 保存最大带符号距离
					best = d;
					f_near = f;
					if (best >= safe_bound) {
						break;  // 该点必然安全
					}  // 如果有个面已经把 点排除了，直接 退出
				}
			}
			// 						  不可能会出现情况2，但是如果代码出问题出现情况2索引失效，不能执行下面的代码
			if (best >= safe_bound || f_near < 0) {  // 前面退出了之后，如果发现这个点已经被排除，说明点是安全的，直接跳过这个点
				continue;
			}
			// 没有通过下界检查，可能是不安全的
			const Eigen::Vector3d n_hat =
				hPoly.block<1, 3>(f_near, 0).transpose() / face_norms(f_near);
			const double support = n_hat.cwiseAbs().dot((*pc_ext)[j]);
			const double need = t_base + support - best;  // best 是点到平面的距离下界 是正表示需要的安全距离比实际当前点面距离更多
			if (need > epsilon) {
				residuals.push_back({j, f_near, need});  // 把不安全点 以及 对应他的面 记录一下
			}
		}
		if (!residuals.empty()) {
			const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
			const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

			// step 2 按面找最大内推量，一次平行 推 覆盖同面全部点。
			// 端点饱和：完整内推会切 a/b 时不再放弃，推到 a/b 允许的极限
			std::vector<double> face_push(base_faces, 0.0);  // 准备内推每张面
			for (const RawDeficit& r : residuals) {  // for each voxel j {
				face_push[r.face] = std::max(face_push[r.face], r.need);
			}
			int pushed_faces = 0;
			for (int f = 0; f < base_faces; ++f) {  // 只是处理基础面
				if (!(face_push[f] > 0.0)) {
					continue;
				}
				const double allowance = std::min(
					-(hPoly.row(f).dot(ah)) / face_norms(f),
					-(hPoly.row(f).dot(bh)) / face_norms(f));  // a/b 到该面的世界余量
				const double actual_push =
					std::min(face_push[f], allowance - epsilon);
				if (actual_push > 0.0) {
					hPoly(f, 3) += actual_push * face_norms(f);
					++pushed_faces;
				}
				// 推不满的面，其残差由 step 3 共享分离面处理
			}

			// step 3 复查残差点，need 大的优先作为共享面种子
			std::sort(residuals.begin(), residuals.end(),
					  [](const RawDeficit& lhs, const RawDeficit& rhs) {
						  return lhs.need > rhs.need;
					  });
			// 任意面证书：安全语义是“存在一张面 f 使
			// signed_f(q) >= t_base + s_f(ext)”，证书可来自任何面（比如刚加的
			// 分离面） 原来只查 argmax 面

			// 新加的面可能会排除体素
			auto isCertified = [&](const int point_idx) {
				const Eigen::Vector3d q = pc.col(point_idx);
				const double safe_bound = t_base + (*pc_ext)[point_idx].norm();
				for (int f = 0; f < hPoly.rows(); ++f) {
					const double norm_f = hPoly.block<1, 3>(f, 0).norm();
					const double d =
						(hPoly.block<1, 3>(f, 0).dot(q) + hPoly(f, 3)) / norm_f;
					if (d >= safe_bound) {
						return true;  // 支撑上界口径都满足，必然安全
					}
					const Eigen::Vector3d n_f =
						hPoly.block<1, 3>(f, 0).transpose() / norm_f;
					if (d >= t_base + n_f.cwiseAbs().dot((*pc_ext)[point_idx]) -
								 epsilon) {
						return true;
					}
				}
				return false;
			};
			int added_planes = 0;
			const Eigen::Vector3d ab = b - a;
			const double ab_sq = ab.squaredNorm();
			for (const RawDeficit& r : residuals) {
				if (isCertified(r.point)) {
					continue;
				}
				const Eigen::Vector3d q = pc.col(r.point);

				// 经过检查，这个点没有被任何面认证，需要新增共享分离面

				// 共享分离面
				// 旧代码是 只贴住种子点的一张面 现在改成 尽量覆盖一批残差点的共享面
				
				// 可保护）。偏置不再贴着种子（旧实现只认证种子自己，邻居
				// 差几毫米认证不过又各自加面 → 一片墙退化成 O(residuals)
				// 张扇形面，run3 实测最坏 48 张）。改为在不切 a/b 的极限
				// d_limit 内取 max{d_i}，让一张面认证所有够得着的残差。
				double t = 0.0;
				if (ab_sq > epsilon) {
					t = std::min(1.0, std::max(0.0, (q - a).dot(ab) / ab_sq));
				}
				const Eigen::Vector3d closest = a + t * ab;
				const Eigen::Vector3d delta = q - closest;
				const double dist_ab = delta.norm();
				if (!(dist_ab > epsilon)) {
					return false;
				}
				const Eigen::Vector3d n_hat = delta / dist_ab;
				const double support = n_hat.cwiseAbs().dot((*pc_ext)[r.point]);
				const double clearance = t_base + support;

				if (!(dist_ab > clearance + epsilon)) {  //障碍点到路径的距离 必须小于 安全余量
					// 保 a-b 与 raw 安全半径在几何上冲突，交三单点兜底
					ROS_WARN_STREAM(
						"[FIRI raw] point too close to a-b segment: dist="
							 << dist_ab << ", need=" << clearance  << "generate fail at post-processing step3");
					return false;
				}
				// 要求 a,b 都在走廊内：
				// n_hat dot a + d <= 0
  				// n_hat dot b + d <= 0
				// 移项：
				// d <= -n_hat dot a
				// d <= -n_hat dot b

				// d 越大走廊收缩越多，收缩恒安全；唯一约束是不切 a/b
				const double d_limit = -std::max(n_hat.dot(a), n_hat.dot(b)) - epsilon;  // 可接受的最大收缩
				const double d_seed = -n_hat.dot(q) + clearance;
				if (d_seed > d_limit) {
					// 种子自身的安全偏置就会切 a/b（几何冲突），交兜底
					return false;
				}
				double d_new = d_seed;
				for (const RawDeficit& other : residuals) {
					const Eigen::Vector3d qo = pc.col(other.point);  // 对于 p_other 
					// 如果 qo 在面外侧，要满足：
					// n_hat dot q_o + d >= t_base + support(n_hat, ext_o)
					const double d_o = t_base +
						n_hat.cwiseAbs().dot((*pc_ext)[other.point]) -
						n_hat.dot(qo);
					if (d_o > d_new && d_o <= d_limit) {  // d_new 尽量覆盖更多
						d_new = d_o;
					}
				}
				Eigen::Vector4d plane;
				plane.head<3>() = n_hat;
				plane(3) = d_new;
				if (plane.dot(ah) > epsilon || plane.dot(bh) > epsilon) {
					return false;
				}
				const int old_rows = hPoly.rows();
				hPoly.conservativeResize(old_rows + 1, Eigen::NoChange);
				hPoly.row(old_rows) = plane.transpose();
				++added_planes;
			}
			ROS_INFO_STREAM_THROTTLE(
				3.0,"[FIRI raw] residuals=" << residuals.size()
						<< " pushed_faces=" << pushed_faces
						<< " added_planes=" << added_planes
						<< " faces=" << base_faces << "->" << hPoly.rows());
		}
	}

	return true;
}

} // namespace firi

#endif
