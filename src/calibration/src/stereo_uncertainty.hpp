#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace spr_calib {

struct StereoUncertainty {
  bool available = false;
  std::string reason = "not computed";
  cv::Mat covariance;
  cv::Mat correlation;
  cv::Mat information;
  std::vector<double> stddev;
  double scaled_condition = std::numeric_limits<double>::quiet_NaN();
  double residual_variance_px2 = std::numeric_limits<double>::quiet_NaN();
  double fitted_rms_px = std::numeric_limits<double>::quiet_NaN();
  double rank_relative_tolerance = 1e-10;
  int rank = 0;
  int degrees_of_freedom = 0;
  int fitted_board_poses = 0;
  double jacobian_max_relative_error = 0.0;
  bool nominal_transform_unchanged = false;
};

namespace stereo_uncertainty_detail {

struct Observation {
  std::vector<cv::Point3d> object;
  cv::Mat measured;
};

struct Model {
  cv::Mat K0, D0, K1, D1, R, T;
};

using Pose = cv::Vec<double, 6>;

inline cv::Mat predict(const Observation& observation, const Pose& pose,
                       const Model& model, cv::Mat* intrinsic_jacobian = nullptr,
                       cv::Mat* right_point_jacobian = nullptr) {
  cv::Mat rotation0;
  cv::Rodrigues(cv::Vec3d(pose[0], pose[1], pose[2]), rotation0);
  const cv::Mat translation0 = (cv::Mat_<double>(3,1) << pose[3], pose[4], pose[5]);
  const cv::Mat rotation1 = model.R * rotation0;
  const cv::Mat translation1 = model.R * translation0 + model.T;
  for (const auto& point : observation.object) {
    const double z0 = rotation0.at<double>(2,0)*point.x + rotation0.at<double>(2,1)*point.y +
                      rotation0.at<double>(2,2)*point.z + translation0.at<double>(2);
    const double z1 = rotation1.at<double>(2,0)*point.x + rotation1.at<double>(2,1)*point.y +
                      rotation1.at<double>(2,2)*point.z + translation1.at<double>(2);
    if (z0 <= 1e-6 || z1 <= 1e-6) return {};
  }
  cv::Mat rvec1;
  cv::Rodrigues(rotation1, rvec1);
  std::vector<cv::Point2d> left, right;
  cv::Mat J0, J1;
  if (intrinsic_jacobian || right_point_jacobian) {
    cv::projectPoints(observation.object, cv::Vec3d(pose[0],pose[1],pose[2]),
                      translation0, model.K0, model.D0, left, J0);
    cv::projectPoints(observation.object, rvec1, translation1, model.K1, model.D1, right, J1);
  } else {
    cv::projectPoints(observation.object, cv::Vec3d(pose[0],pose[1],pose[2]),
                      translation0, model.K0, model.D0, left);
    cv::projectPoints(observation.object, rvec1, translation1, model.K1, model.D1, right);
  }
  const int count = static_cast<int>(left.size());
  cv::Mat result(4*count,1,CV_64F);
  for (int i = 0; i < count; ++i) {
    result.at<double>(2*i) = left[static_cast<size_t>(i)].x;
    result.at<double>(2*i+1) = left[static_cast<size_t>(i)].y;
    result.at<double>(2*count+2*i) = right[static_cast<size_t>(i)].x;
    result.at<double>(2*count+2*i+1) = right[static_cast<size_t>(i)].y;
  }
  if (intrinsic_jacobian) {
    *intrinsic_jacobian = cv::Mat::zeros(4*count,22,CV_64F);
    J0.colRange(6,14).copyTo((*intrinsic_jacobian)(cv::Rect(0,0,8,2*count)));
    J1.colRange(6,14).copyTo((*intrinsic_jacobian)(cv::Rect(8,2*count,8,2*count)));
  }
  if (right_point_jacobian) *right_point_jacobian = J1.colRange(3,6).clone();
  return result;
}

inline cv::Mat poseJacobian(const Observation& observation, const Pose& pose,
                            const Model& model) {
  cv::Mat result(static_cast<int>(observation.object.size())*4,6,CV_64F);
  constexpr double step = 1e-6;
  for (int column = 0; column < 6; ++column) {
    Pose plus = pose, minus = pose;
    plus[column] += step;
    minus[column] -= step;
    const auto a = predict(observation, plus, model);
    const auto b = predict(observation, minus, model);
    if (a.empty() || b.empty()) throw std::runtime_error("invalid board pose in numerical Jacobian");
    const cv::Mat derivative = (a-b)/(2*step);
    derivative.copyTo(result.col(column));
  }
  return result;
}

inline cv::Mat scaledColumns(const cv::Mat& matrix, cv::Mat& scale) {
  scale = cv::Mat::zeros(matrix.cols,matrix.cols,CV_64F);
  for (int i = 0; i < matrix.cols; ++i) {
    const double norm = cv::norm(matrix.col(i));
    if (!std::isfinite(norm) || norm <= 0) throw std::runtime_error("zero or invalid Jacobian column");
    scale.at<double>(i,i) = 1.0/norm;
  }
  return matrix * scale;
}

inline Pose fitBoardPose(const Observation& observation, Pose pose, const Model& model) {
  double damping = 1e-3;
  auto prediction = predict(observation,pose,model);
  if (prediction.empty()) throw std::runtime_error("board pose is behind a stereo camera");
  double cost = cv::norm(observation.measured-prediction,cv::NORM_L2SQR);
  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto J = poseJacobian(observation,pose,model);
    cv::Mat scale;
    const auto normalized = scaledColumns(J,scale);
    const cv::Mat residual = observation.measured-prediction;
    const cv::Mat normal = normalized.t()*normalized;
    const cv::Mat rhs = normalized.t()*residual;
    bool accepted = false;
    for (int trial = 0; trial < 10; ++trial) {
      cv::Mat step;
      if (!cv::solve(normal + damping*cv::Mat::eye(6,6,CV_64F),rhs,step,cv::DECOMP_SVD))
        throw std::runtime_error("board-pose refinement solve failed");
      step = scale*step;
      if (cv::norm(step) < 1e-10) return pose;
      Pose candidate = pose;
      for (int i = 0; i < 6; ++i) candidate[i] += step.at<double>(i);
      const auto candidate_prediction = predict(observation,candidate,model);
      const double candidate_cost = candidate_prediction.empty() ?
          std::numeric_limits<double>::infinity() :
          cv::norm(observation.measured-candidate_prediction,cv::NORM_L2SQR);
      if (std::isfinite(candidate_cost) && candidate_cost < cost) {
        const double improvement = cost-candidate_cost;
        pose = candidate;
        prediction = candidate_prediction;
        cost = candidate_cost;
        damping *= 0.2;
        accepted = true;
        if (improvement < 1e-10*(1.0+cost)) return pose;
        break;
      }
      damping *= 10.0;
    }
    if (!accepted) {
      // At a stationary solution numerical differencing may not improve the
      // cost. A nonstationary failure must not be reported as a fitted pose.
      if (cv::norm(rhs,cv::NORM_INF) < 1e-5) return pose;
      throw std::runtime_error("joint board-pose refinement did not converge");
    }
  }
  throw std::runtime_error("joint board-pose refinement iteration limit reached");
}

inline cv::Mat globalJacobian(const Observation& observation, const Pose& pose,
                              const Model& model, double& max_relative_error) {
  cv::Mat result, point_jacobian, board_rotation;
  predict(observation,pose,model,&result,&point_jacobian);
  cv::Rodrigues(cv::Vec3d(pose[0],pose[1],pose[2]),board_rotation);
  const cv::Mat board_translation = (cv::Mat_<double>(3,1) << pose[3],pose[4],pose[5]);
  const int count = static_cast<int>(observation.object.size());
  for (int i = 0; i < count; ++i) {
    const auto& point = observation.object[static_cast<size_t>(i)];
    const cv::Mat p = (cv::Mat_<double>(3,1) << point.x,point.y,point.z);
    const cv::Mat rotated = model.R*(board_rotation*p+board_translation);
    const double x=rotated.at<double>(0), y=rotated.at<double>(1), z=rotated.at<double>(2);
    const cv::Mat negative_skew = (cv::Mat_<double>(3,3) << 0,z,-y,-z,0,x,y,-x,0);
    const cv::Mat point_block = point_jacobian.rowRange(2*i,2*i+2);
    const cv::Mat rotation_block = point_block*negative_skew;
    rotation_block.copyTo(result(cv::Rect(16,2*count+2*i,3,2)));
    point_block.copyTo(result(cv::Rect(19,2*count+2*i,3,2)));
  }
  const cv::Mat nominal_R = model.R.clone(), nominal_T = model.T.clone();
  constexpr double step = 1e-6;
  for (int column = 0; column < 6; ++column) {
    Model plus = model, minus = model;
    if (column < 3) {
      cv::Vec3d delta(0,0,0);
      delta[column] = step;
      cv::Mat positive_rotation, negative_rotation;
      cv::Rodrigues(delta,positive_rotation);
      cv::Rodrigues(-delta,negative_rotation);
      // Initialize distinct matrices before assigning: MatExpr assignment
      // into a shallow Model copy can otherwise overwrite the nominal R.
      const cv::Mat rotated_plus = positive_rotation*model.R;
      const cv::Mat rotated_minus = negative_rotation*model.R;
      plus.R = rotated_plus;
      minus.R = rotated_minus;
    } else {
      plus.T = model.T.clone();
      minus.T = model.T.clone();
      plus.T.at<double>(column-3) += step;
      minus.T.at<double>(column-3) -= step;
    }
    const auto a = predict(observation,pose,plus);
    const auto b = predict(observation,pose,minus);
    if (a.empty() || b.empty()) throw std::runtime_error("invalid stereo transform in numerical Jacobian");
    const cv::Mat derivative = (a-b)/(2*step);
    const double relative_error = cv::norm(derivative-result.col(16+column))/
        std::max(1.0,cv::norm(result.col(16+column)));
    max_relative_error = std::max(max_relative_error,relative_error);
    if (!std::isfinite(relative_error) || relative_error > 1e-5)
      throw std::runtime_error("analytic stereo transform Jacobian disagrees with independent central differences");
    if (cv::norm(model.R-nominal_R) != 0 || cv::norm(model.T-nominal_T) != 0)
      throw std::runtime_error("stereo Jacobian mutated the nominal transform");
  }
  return result;
}

}  // namespace stereo_uncertainty_detail

template<typename View>
StereoUncertainty estimateStereoUncertainty(const std::vector<View>& views,
    const cv::Mat& K0, const cv::Mat& D0, const cv::Mat& K1, const cv::Mat& D1,
    const cv::Mat& R, const cv::Mat& T) {
  using namespace stereo_uncertainty_detail;
  StereoUncertainty result;
  try {
    const Model model{K0.clone(),D0.clone(),K1.clone(),D1.clone(),R.clone(),T.clone()};
    const cv::Mat nominal_R = R.clone(), nominal_T = T.clone();
    cv::Mat information = cv::Mat::zeros(22,22,CV_64F);
    double squared = 0.0;
    int residuals = 0;
    for (const auto& view : views) {
      Observation observation;
      for (const auto& p : view.object_points) observation.object.emplace_back(p.x,p.y,p.z);
      const int count = static_cast<int>(observation.object.size());
      if (count < 6 || view.left.size() != observation.object.size() || view.right.size() != observation.object.size())
        throw std::runtime_error("invalid stereo observation sizes");
      observation.measured = cv::Mat(4*count,1,CV_64F);
      for (int i = 0; i < count; ++i) {
        observation.measured.at<double>(2*i) = view.left[static_cast<size_t>(i)].x;
        observation.measured.at<double>(2*i+1) = view.left[static_cast<size_t>(i)].y;
        observation.measured.at<double>(2*count+2*i) = view.right[static_cast<size_t>(i)].x;
        observation.measured.at<double>(2*count+2*i+1) = view.right[static_cast<size_t>(i)].y;
      }
      cv::Mat rvec,tvec;
      if (!cv::solvePnP(observation.object,view.left,K0,D0,rvec,tvec))
        throw std::runtime_error("initial board-pose PnP failed");
      Pose pose(rvec.at<double>(0),rvec.at<double>(1),rvec.at<double>(2),
                tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));
      pose = fitBoardPose(observation,pose,model);
      const auto prediction = predict(observation,pose,model);
      const auto global = globalJacobian(observation,pose,model,result.jacobian_max_relative_error);
      const auto nuisance = poseJacobian(observation,pose,model);
      cv::Mat nuisance_scale;
      const auto normalized_nuisance = scaledColumns(nuisance,nuisance_scale);
      const cv::SVD nuisance_svd(normalized_nuisance,0);
      if (nuisance_svd.w.at<double>(5) <= nuisance_svd.w.at<double>(0)*result.rank_relative_tolerance)
        throw std::runtime_error("a board pose has rank-deficient image information");
      // Orthogonal residualization is the Schur complement, avoiding the
      // cancellation from Hgg-Hgp*inverse(Hpp)*Hpg in mixed physical units.
      const cv::Mat marginalized = global - nuisance_svd.u*(nuisance_svd.u.t()*global);
      information += marginalized.t()*marginalized;
      squared += cv::norm(observation.measured-prediction,cv::NORM_L2SQR);
      residuals += 4*count;
      ++result.fitted_board_poses;
    }
    result.degrees_of_freedom = residuals-6*static_cast<int>(views.size())-22;
    if (result.degrees_of_freedom <= 0 || !std::isfinite(squared) || squared <= 0)
      throw std::runtime_error("residual variance cannot be estimated from these observations");
    result.residual_variance_px2 = squared/result.degrees_of_freedom;
    result.fitted_rms_px = std::sqrt(squared/(residuals/2.0));
    cv::Mat scale = cv::Mat::zeros(22,22,CV_64F);
    for (int i = 0; i < 22; ++i) {
      const double diagonal = information.at<double>(i,i);
      if (!std::isfinite(diagonal) || diagonal <= 0)
        throw std::runtime_error("a global parameter has no marginal information");
      scale.at<double>(i,i) = 1.0/std::sqrt(diagonal);
    }
    const cv::Mat normalized = scale*information*scale;
    const cv::SVD svd(normalized,cv::SVD::FULL_UV);
    const double largest = svd.w.at<double>(0);
    const double smallest = svd.w.at<double>(21);
    result.scaled_condition = smallest > 0 ? largest/smallest : std::numeric_limits<double>::infinity();
    for (int i = 0; i < 22; ++i)
      if (svd.w.at<double>(i) > largest*result.rank_relative_tolerance) ++result.rank;
    if (result.rank != 22) throw std::runtime_error("global stereo information is rank deficient; no pseudoinverse covariance is published");
    cv::Mat inverse = cv::Mat::zeros(22,22,CV_64F);
    for (int i = 0; i < 22; ++i) inverse.at<double>(i,i) = 1.0/svd.w.at<double>(i);
    result.information = information;
    result.covariance = scale*svd.vt.t()*inverse*svd.u.t()*scale*result.residual_variance_px2;
    result.covariance = (result.covariance+result.covariance.t())*0.5;
    if (!cv::checkRange(result.covariance)) throw std::runtime_error("non-finite stereo covariance");
    result.correlation = cv::Mat(22,22,CV_64F);
    for (int i = 0; i < 22; ++i) {
      const double variance = result.covariance.at<double>(i,i);
      if (variance <= 0) throw std::runtime_error("nonpositive stereo parameter variance");
      result.stddev.push_back(std::sqrt(variance));
    }
    for (int r = 0; r < 22; ++r)
      for (int c = 0; c < 22; ++c) {
        const double value = result.covariance.at<double>(r,c)/(result.stddev[static_cast<size_t>(r)]*result.stddev[static_cast<size_t>(c)]);
        if (!std::isfinite(value) || std::abs(value) > 1.0+1e-8)
          throw std::runtime_error("invalid stereo parameter correlation");
        result.correlation.at<double>(r,c) = value;
      }
    result.available = true;
    result.nominal_transform_unchanged = cv::norm(model.R-nominal_R) == 0 &&
        cv::norm(model.T-nominal_T) == 0 && cv::norm(R-nominal_R) == 0 && cv::norm(T-nominal_T) == 0;
    if (!result.nominal_transform_unchanged) throw std::runtime_error("uncertainty estimation mutated the calibration transform");
    result.reason = "local linearized covariance estimated from real image residuals; no bootstrap/systematic-error model";
  } catch (const std::exception& error) {
    result.available = false;
    result.reason = error.what();
    result.covariance.release();
    result.correlation.release();
    result.information.release();
    result.stddev.clear();
  }
  return result;
}

}  // namespace spr_calib
