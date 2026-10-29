#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace spr_calib {

struct BoardConfig {
  int cols = 8;             // inner corners per row (default orientation)
  int rows = 11;            // inner corners per column (default orientation)
  double square_m = 0.025;  // physical square edge length [m]
  bool auto_orientation = true;
};

inline std::vector<cv::Point3f> makeObjectPoints(int cols, int rows,
                                                 double square_m) {
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<size_t>(cols) * static_cast<size_t>(rows));
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      points.emplace_back(static_cast<float>(c * square_m),
                          static_cast<float>(r * square_m), 0.0F);
    }
  }
  return points;
}

constexpr int kDetectFlagsExhaustive = cv::CALIB_CB_NORMALIZE_IMAGE |
                                       cv::CALIB_CB_EXHAUSTIVE |
                                       cv::CALIB_CB_ACCURACY;
constexpr int kDetectFlagsFast =
    cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY;

inline bool detectCorners(const cv::Mat& gray, int cols, int rows,
                          std::vector<cv::Point2f>& corners,
                          int flags = kDetectFlagsExhaustive) {
  corners.clear();
  if (!cv::findChessboardCornersSB(gray, cv::Size(cols, rows), corners, flags)) {
    // Real D435i IR frames can contain all corners while the sector-based
    // detector fails under low contrast. Refine the classic detector on the
    // original image; never infer or fill missing corners.
    corners.clear();
    const int classic_flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                              cv::CALIB_CB_NORMALIZE_IMAGE |
                              cv::CALIB_CB_FILTER_QUADS;
    if (!cv::findChessboardCorners(gray, cv::Size(cols, rows), corners,
                                   classic_flags)) return false;
    cv::cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::COUNT |
                                      cv::TermCriteria::EPS, 40, 0.001));
  }
  return corners.size() == static_cast<size_t>(cols) * static_cast<size_t>(rows);
}

// Tries the configured orientation first, then the transposed one. The
// returned `pattern` must be used to build the object points for this view.
inline bool detectBoard(const cv::Mat& gray, const BoardConfig& config,
                        std::vector<cv::Point2f>& corners, cv::Size& pattern,
                        int flags = kDetectFlagsExhaustive) {
  if (detectCorners(gray, config.cols, config.rows, corners, flags)) {
    pattern = cv::Size(config.cols, config.rows);
    return true;
  }
  if (config.auto_orientation &&
      detectCorners(gray, config.rows, config.cols, corners, flags)) {
    pattern = cv::Size(config.rows, config.cols);
    return true;
  }
  return false;
}

// SB and classic may choose opposite checkerboard origins. For the D435i's
// similarly oriented stereo sensors, choose the right ordering whose centered
// grid agrees with the left one. The remaining translation is the disparity.
inline void alignStereoCornerOrder(const std::vector<cv::Point2f>& left,
                                   std::vector<cv::Point2f>& right) {
  if (left.empty() || left.size() != right.size()) return;
  cv::Point2d mean_left(0,0), mean_right(0,0);
  for (size_t i = 0; i < left.size(); ++i) {
    mean_left += cv::Point2d(left[i]);
    mean_right += cv::Point2d(right[i]);
  }
  mean_left *= 1.0 / left.size();
  mean_right *= 1.0 / right.size();
  double direct = 0.0, reversed = 0.0;
  for (size_t i = 0; i < left.size(); ++i) {
    const auto centered = cv::Point2d(left[i]) - mean_left;
    const auto error = centered - (cv::Point2d(right[i]) - mean_right);
    const auto reverse_error = centered - (cv::Point2d(right[right.size()-1-i]) - mean_right);
    direct += error.dot(error);
    reversed += reverse_error.dot(reverse_error);
  }
  if (reversed < direct) std::reverse(right.begin(), right.end());
}

inline bool boardInsideImage(const std::vector<cv::Point2f>& corners,
                             const cv::Size& image_size, double margin_px) {
  if (corners.empty()) {
    return false;
  }
  for (const cv::Point2f& p : corners) {
    if (p.x < margin_px || p.y < margin_px ||
        p.x > image_size.width - 1.0 - margin_px ||
        p.y > image_size.height - 1.0 - margin_px) {
      return false;
    }
  }
  return true;
}

inline double laplacianVariance(const cv::Mat& gray,
                                const std::vector<cv::Point2f>& corners,
                                int padding_px = 10) {
  if (gray.empty() || corners.empty()) {
    return 0.0;
  }
  cv::Rect roi = cv::boundingRect(corners);
  roi.x -= padding_px;
  roi.y -= padding_px;
  roi.width += 2 * padding_px;
  roi.height += 2 * padding_px;
  roi &= cv::Rect(0, 0, gray.cols, gray.rows);
  if (roi.width < 3 || roi.height < 3) {
    return 0.0;
  }
  cv::Mat laplacian;
  cv::Laplacian(gray(roi), laplacian, CV_64F);
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  return stddev[0] * stddev[0];
}

inline int coverageBinIndex(const cv::Point2f& p, const cv::Size& image_size,
                            int bins_x, int bins_y) {
  int bx = static_cast<int>(p.x * bins_x /
                            static_cast<double>(image_size.width));
  int by = static_cast<int>(p.y * bins_y /
                            static_cast<double>(image_size.height));
  bx = std::clamp(bx, 0, bins_x - 1);
  by = std::clamp(by, 0, bins_y - 1);
  return by * bins_x + bx;
}

inline std::vector<int> coverageBins(const std::vector<cv::Point2f>& corners,
                                     const cv::Size& image_size,
                                     int bins_x = 8, int bins_y = 6) {
  std::vector<int> bins;
  bins.reserve(corners.size());
  for (const cv::Point2f& p : corners) {
    const int idx = coverageBinIndex(p, image_size, bins_x, bins_y);
    if (std::find(bins.begin(), bins.end(), idx) == bins.end()) {
      bins.push_back(idx);
    }
  }
  return bins;
}

inline double rotationAngleDeg(const cv::Mat& R1, const cv::Mat& R2) {
  const cv::Mat R = R1.t() * R2;
  const double trace =
      R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
  const double c = std::clamp((trace - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / CV_PI;
}

}  // namespace spr_calib
