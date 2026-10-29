#include "board.hpp"
#include "yaml_io.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using ImageConstPtr = Image::ConstSharedPtr;
using CameraInfoConstPtr = CameraInfo::ConstSharedPtr;

constexpr int kBinsX = 8;
constexpr int kBinsY = 6;
constexpr int kBinCount = kBinsX * kBinsY;

struct AcceptedView {
  std::string name;
  int64_t stamp_ns = 0;
  int64_t right_stamp_ns = 0;
  double blur_left = 0.0;
  double blur_right = 0.0;
  double min_margin = 0.0;
  bool pose_valid = false;
  cv::Vec3d rvec{0.0, 0.0, 0.0};
  cv::Vec3d tvec{0.0, 0.0, 0.0};
  cv::Point2f centroid{0.0F, 0.0F};
  double scale_px = 0.0;
  std::vector<int> bins;
};

class CaptureNode final : public rclcpp::Node {
 public:
  CaptureNode() : Node("calibration_capture") {
    left_image_topic_ = declare_parameter<std::string>(
        "left_image_topic", "/camera/infra1/image_rect_raw");
    right_image_topic_ = declare_parameter<std::string>(
        "right_image_topic", "/camera/infra2/image_rect_raw");
    left_info_topic_ = declare_parameter<std::string>(
        "left_info_topic", "/camera/infra1/camera_info");
    right_info_topic_ = declare_parameter<std::string>(
        "right_info_topic", "/camera/infra2/camera_info");
    output_dir_ = declare_parameter<std::string>("output_dir", "data");

    board_config_.cols = declare_parameter<int>("board_cols", 8);
    board_config_.rows = declare_parameter<int>("board_rows", 11);
    board_config_.square_m = declare_parameter<double>("square_size", 0.025);
    board_config_.auto_orientation =
        declare_parameter<bool>("auto_orientation", true);

    auto_save_ = declare_parameter<bool>("auto_save", true);
    min_pairs_ = declare_parameter<int>("min_pairs", 30);
    blur_threshold_ = declare_parameter<double>("blur_threshold", 100.0);
    edge_margin_ = declare_parameter<double>("edge_margin", 15.0);
    diversity_translation_ =
        declare_parameter<double>("diversity_translation", 0.03);
    diversity_rotation_deg_ =
        declare_parameter<double>("diversity_rotation_deg", 8.0);
    save_cooldown_ = declare_parameter<double>("save_cooldown", 0.5);
    const double detect_rate_hz =
        declare_parameter<double>("detect_rate_hz", 10.0);
    const bool exhaustive =
        declare_parameter<bool>("exhaustive_detection", false);
    show_ui_ = declare_parameter<bool>("show_ui", true);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 10.0);
    expected_width_ = declare_parameter<int>("expected_width", 0);
    expected_height_ = declare_parameter<int>("expected_height", 0);
    expected_encoding_ = declare_parameter<std::string>("expected_encoding", "mono8");
    max_stereo_delta_sec_ = declare_parameter<double>("max_stereo_delta_sec", 0.001);
    const int opencv_threads = declare_parameter<int>("opencv_threads", 2);
    if (opencv_threads < 1) throw std::runtime_error("opencv_threads must be at least 1");
    cv::setNumThreads(opencv_threads);

    detect_flags_ = exhaustive ? spr_calib::kDetectFlagsExhaustive
                               : spr_calib::kDetectFlagsFast;
    if (detect_rate_hz > 0.0) {
      detect_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / detect_rate_hz));
    }
    last_detect_ = std::chrono::steady_clock::now() - detect_period_ - detect_period_;

    if (board_config_.cols < 2 || board_config_.rows < 2 ||
        !std::isfinite(board_config_.square_m) || board_config_.square_m <= 0.0 ||
        min_pairs_ < 3 || !std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0 ||
        !std::isfinite(max_stereo_delta_sec_) || max_stereo_delta_sec_ <= 0.0) {
      throw std::runtime_error("invalid board configuration");
    }

    createSession();
    setupUi();
    setupSubscriptions();
    last_input_ = std::chrono::steady_clock::now();
    watchdog_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_input_).count() > input_timeout_sec_) {
        RCLCPP_ERROR(get_logger(), "no synchronized stereo input for %.1f s; check topics/QoS/driver", input_timeout_sec_);
        input_failed_ = true;
        quit_requested_ = true;
      }
    });
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(get_logger(),
          "capture status: both_detected=%s pattern=%dx%d blur=%.1f/%.1f margin=%.1f accepted=%zu coverage=%d%% reason=%s",
          both_detected_ ? "true" : "false", current_pattern_.width,
          current_pattern_.height, left_blur_, right_blur_, min_margin_,
          accepted_.size(), coveragePercent(), status_reason_.c_str());
    });
    RCLCPP_INFO(get_logger(), "OpenCV threads: %d", opencv_threads);
  }

  bool quitRequested() const { return quit_requested_; }
  bool showUi() const { return show_ui_; }

  bool finalize() {
    writeIndex();
    writeMeta();
    const bool ready = !input_failed_ && accepted_.size() >= static_cast<size_t>(min_pairs_) &&
                       coveragePercent() >= 80;
    if (!ready) {
      RCLCPP_ERROR(get_logger(), "capture incomplete: %zu pairs (need %d), left coverage %d%% (need 80%%). Empty/incomplete sessions are not calibration results.",
                   accepted_.size(), min_pairs_, coveragePercent());
    }
    return ready;
  }

  void render() {
    if (!show_ui_) {
      return;
    }
    try {
      cv::Mat left_display;
      cv::Mat right_display;
      if (latest_left_gray_.empty() || latest_right_gray_.empty()) {
        left_display = cv::Mat(480, 848, CV_8UC1, cv::Scalar(40));
        right_display = cv::Mat(480, 848, CV_8UC1, cv::Scalar(40));
        cv::putText(left_display, "waiting for both IR streams",
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(220), 2);
      } else {
        left_display = latest_left_gray_.clone();
        right_display = latest_right_gray_.clone();
        if (both_detected_ && current_pattern_.width > 0) {
          cv::drawChessboardCorners(left_display, current_pattern_,
                                    left_corners_, true);
          cv::drawChessboardCorners(right_display, current_pattern_,
                                    right_corners_, true);
        }
      }
      drawStatus(left_display);
      drawStatus(right_display);
      cv::imshow("left", left_display);
      cv::imshow("right", right_display);
      drawCoverage();

      const int key = cv::waitKey(1);
      if (key == ' ' || key == 32) {
        forceSave();
      } else if (key == 8 || key == 127) {
        deleteLast();
      } else if (key == 'r' || key == 'R') {
        pattern_locked_ = false;
        status_reason_ = "orientation lock reset";
        RCLCPP_INFO(get_logger(), "board orientation lock reset");
      } else if (key == 'q' || key == 'Q' || key == 27) {
        quit_requested_ = true;
      }
    } catch (const cv::Exception& error) {
      show_ui_ = false;
      RCLCPP_WARN(get_logger(), "GUI disabled after OpenCV error: %s",
                  error.what());
    }
  }

 private:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void createSession() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);
    std::ostringstream stamp;
    stamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    session_start_ = stamp.str();

    const fs::path base = fs::path(output_dir_);
    fs::path session = base / session_start_;
    for (int suffix = 1; fs::exists(session); ++suffix) {
      session = base / (session_start_ + "_" + std::to_string(suffix));
    }
    session_dir_ = session;
    fs::create_directories(session_dir_ / "left");
    fs::create_directories(session_dir_ / "right");
    writeMeta();
    writeIndex();
    RCLCPP_INFO(get_logger(), "capture session: %s",
                fs::absolute(session_dir_).c_str());
  }

  void setupUi() {
    if (!show_ui_) {
      return;
    }
    try {
      cv::namedWindow("left", cv::WINDOW_NORMAL);
      cv::namedWindow("right", cv::WINDOW_NORMAL);
      cv::namedWindow("coverage", cv::WINDOW_NORMAL);
      cv::resizeWindow("left", 848, 480);
      cv::resizeWindow("right", 848, 480);
      cv::resizeWindow("coverage", 480, 360);
    } catch (const cv::Exception& error) {
      show_ui_ = false;
      RCLCPP_WARN(get_logger(), "GUI unavailable, running headless: %s",
                  error.what());
    }
  }

  void setupSubscriptions() {
    info_sub_left_ = create_subscription<CameraInfo>(
        left_info_topic_, rclcpp::SensorDataQoS(),
        [this](CameraInfoConstPtr message) {
          onCameraInfo(message, true);
        });
    info_sub_right_ = create_subscription<CameraInfo>(
        right_info_topic_, rclcpp::SensorDataQoS(),
        [this](CameraInfoConstPtr message) {
          onCameraInfo(message, false);
        });

    left_sub_.subscribe(this, left_image_topic_, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_image_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub_,
                                           right_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_stereo_delta_sec_));
    sync_->registerCallback(std::bind(&CaptureNode::onImages, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "subscribed: %s + %s",
                left_image_topic_.c_str(), right_image_topic_.c_str());
  }

  void onCameraInfo(const CameraInfoConstPtr& message, bool left) {
    if (message->k[0] <= 0.0) {
      return;
    }
    bool changed = false;
    if (left) {
      if (!factory_left_valid_) {
        factory_left_valid_ = true;
        changed = true;
      }
      factory_k_left_ = cameraMatrixFromInfo(*message);
      factory_d_left_ = distortionFromInfo(*message);
    } else {
      if (!factory_right_valid_) {
        factory_right_valid_ = true;
        changed = true;
      }
      factory_k_right_ = cameraMatrixFromInfo(*message);
      factory_d_right_ = distortionFromInfo(*message);
    }
    if (changed) {
      writeMeta();
      RCLCPP_INFO(get_logger(), "%s camera_info: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                  left ? "left" : "right", message->k[0], message->k[4],
                  message->k[2], message->k[5]);
    }
  }

  static cv::Mat cameraMatrixFromInfo(const CameraInfo& message) {
    cv::Mat matrix = cv::Mat::zeros(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        matrix.at<double>(row, col) = message.k[row * 3 + col];
      }
    }
    return matrix;
  }

  static cv::Mat distortionFromInfo(const CameraInfo& message) {
    cv::Mat distortion(1, static_cast<int>(message.d.size()), CV_64F);
    for (size_t i = 0; i < message.d.size(); ++i) {
      distortion.at<double>(0, static_cast<int>(i)) = message.d[i];
    }
    return distortion;
  }

  static cv::Mat toMono8(const Image& message) {
    if (message.encoding != "mono8" && message.encoding != "8UC1") {
      throw std::runtime_error("unsupported image encoding: " +
                               message.encoding);
    }
    if (message.width == 0 || message.height == 0) {
      throw std::runtime_error("empty image");
    }
    const size_t required =
        static_cast<size_t>(message.step) * static_cast<size_t>(message.height);
    if (required > message.data.size()) {
      throw std::runtime_error("image data is smaller than step*height");
    }
    const cv::Mat view(static_cast<int>(message.height),
                       static_cast<int>(message.width), CV_8UC1,
                       const_cast<uint8_t*>(message.data.data()),
                       static_cast<size_t>(message.step));
    return view.clone();
  }

  bool detectBoth(const cv::Mat& left, const cv::Mat& right,
                  std::vector<cv::Point2f>& left_corners,
                  std::vector<cv::Point2f>& right_corners,
                  cv::Size& pattern) {
    if (pattern_locked_) {
      pattern = locked_pattern_;
      return spr_calib::detectCorners(left, pattern.width, pattern.height,
                                      left_corners, detect_flags_) &&
             spr_calib::detectCorners(right, pattern.width, pattern.height,
                                      right_corners, detect_flags_);
    }

    cv::Size left_pattern;
    cv::Size right_pattern;
    if (!spr_calib::detectBoard(left, board_config_, left_corners,
                                left_pattern, detect_flags_) ||
        !spr_calib::detectBoard(right, board_config_, right_corners,
                                right_pattern, detect_flags_)) {
      return false;
    }
    if (left_pattern != right_pattern) {
      status_reason_ = "left/right board orientation mismatch";
      return false;
    }
    locked_pattern_ = left_pattern;
    pattern_locked_ = true;
    pattern = left_pattern;
    RCLCPP_INFO(get_logger(), "locked board orientation %dx%d",
                pattern.width, pattern.height);
    return true;
  }

  static double minMargin(const std::vector<cv::Point2f>& corners,
                          const cv::Size& size) {
    if (corners.empty()) {
      return 0.0;
    }
    double margin = std::numeric_limits<double>::max();
    for (const cv::Point2f& p : corners) {
      margin = std::min({margin, static_cast<double>(p.x),
                         static_cast<double>(p.y),
                         size.width - 1.0 - p.x,
                         size.height - 1.0 - p.y});
    }
    return margin;
  }

  static void centroidScale(const std::vector<cv::Point2f>& corners,
                            cv::Point2f& centroid, double& scale) {
    if (corners.empty()) {
      centroid = cv::Point2f(0.0F, 0.0F);
      scale = 0.0;
      return;
    }
    cv::Point2f sum(0.0F, 0.0F);
    for (const cv::Point2f& p : corners) {
      sum += p;
    }
    centroid = sum * (1.0F / static_cast<float>(corners.size()));
    double distance = 0.0;
    for (const cv::Point2f& p : corners) {
      distance += cv::norm(p - centroid);
    }
    scale = distance / static_cast<double>(corners.size());
  }

  bool solveBoardPose(const std::vector<cv::Point2f>& corners,
                      const cv::Size& pattern, cv::Vec3d& rvec,
                      cv::Vec3d& tvec) const {
    if (!factory_left_valid_) {
      return false;
    }
    const std::vector<cv::Point3f> object_points = spr_calib::makeObjectPoints(
        pattern.width, pattern.height, board_config_.square_m);
    cv::Mat rotation;
    cv::Mat translation;
    try {
      if (!cv::solvePnP(object_points, corners, factory_k_left_,
                        cv::Mat::zeros(4, 1, CV_64F), rotation, translation,
                        false, cv::SOLVEPNP_IPPE)) {
        return false;
      }
    } catch (const cv::Exception&) {
      return false;
    }
    rvec = cv::Vec3d(rotation.at<double>(0), rotation.at<double>(1),
                     rotation.at<double>(2));
    tvec = cv::Vec3d(translation.at<double>(0), translation.at<double>(1),
                     translation.at<double>(2));
    return true;
  }

  bool diverseEnough(const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                     bool pose_valid,
                     const std::vector<cv::Point2f>& corners) const {
    if (accepted_.empty()) {
      return true;
    }
    if (pose_valid) {
      cv::Mat rotation;
      cv::Rodrigues(cv::Mat(rvec), rotation);
      for (const AcceptedView& view : accepted_) {
        if (!view.pose_valid) {
          continue;
        }
        cv::Mat accepted_rotation;
        cv::Rodrigues(cv::Mat(view.rvec), accepted_rotation);
        const double angle =
            spr_calib::rotationAngleDeg(rotation, accepted_rotation);
        const double distance =
            cv::norm(cv::Mat(tvec), cv::Mat(view.tvec), cv::NORM_L2);
        if (angle < diversity_rotation_deg_ &&
            distance < diversity_translation_) {
          return false;
        }
      }
      return true;
    }

    cv::Point2f centroid;
    double scale = 0.0;
    centroidScale(corners, centroid, scale);
    for (const AcceptedView& view : accepted_) {
      if (view.scale_px <= 0.0) {
        continue;
      }
      if (cv::norm(centroid - view.centroid) < 30.0 &&
          std::abs(scale / view.scale_px - 1.0) < 0.10) {
        return false;
      }
    }
    return true;
  }

  void onImages(const ImageConstPtr& left_message,
                const ImageConstPtr& right_message) {
    if (quit_requested_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    last_input_ = now;
    if (detect_period_.count() > 0 && now - last_detect_ < detect_period_) {
      return;
    }
    last_detect_ = now;

    if (!expected_encoding_.empty() &&
        (left_message->encoding != expected_encoding_ || right_message->encoding != expected_encoding_)) {
      RCLCPP_ERROR(get_logger(), "expected IR encoding %s, received %s/%s",
                   expected_encoding_.c_str(), left_message->encoding.c_str(), right_message->encoding.c_str());
      input_failed_ = true;
      quit_requested_ = true;
      return;
    }

    cv::Mat left;
    cv::Mat right;
    try {
      left = toMono8(*left_message);
      right = toMono8(*right_message);
    } catch (const std::exception& error) {
      if (!conversion_error_logged_) {
        RCLCPP_ERROR(get_logger(), "image conversion failed: %s",
                     error.what());
        conversion_error_logged_ = true;
      }
      return;
    }

    if (left.size() != right.size()) {
      if (!size_mismatch_logged_) {
        RCLCPP_ERROR(get_logger(), "left/right image sizes differ");
        size_mismatch_logged_ = true;
      }
      return;
    }
    if ((expected_width_ > 0 && left.cols != expected_width_) ||
        (expected_height_ > 0 && left.rows != expected_height_) ||
        (image_size_.width > 0 && left.size() != image_size_)) {
      RCLCPP_ERROR(get_logger(), "unexpected image profile %dx%d; capture cannot mix resolutions", left.cols, left.rows);
      input_failed_ = true;
      quit_requested_ = true;
      return;
    }

    if (image_size_.width == 0) {
      image_size_ = left.size();
      writeMeta();
      RCLCPP_INFO(get_logger(), "image size: %dx%d", image_size_.width,
                  image_size_.height);
    }

    latest_left_gray_ = left;
    latest_right_gray_ = right;
    latest_stamp_ns_ =
        rclcpp::Time(left_message->header.stamp).nanoseconds();
    latest_right_stamp_ns_ = rclcpp::Time(right_message->header.stamp).nanoseconds();
    left_frame_id_ = left_message->header.frame_id;
    right_frame_id_ = right_message->header.frame_id;

    if (!refreshDetection(left, right)) return;

    const bool margin_ok = min_margin_ >= edge_margin_;
    const bool blur_ok = std::min(left_blur_, right_blur_) >= blur_threshold_;
    const bool diverse = diverseEnough(rvec_, tvec_, pose_valid_, left_corners_);
    const bool cooldown_ok =
        std::chrono::duration<double>(now - last_save_).count() >= save_cooldown_;

    if (!margin_ok) {
      status_reason_ = "board too close to image edge";
    } else if (!blur_ok) {
      status_reason_ = "image too blurry";
    } else if (!diverse) {
      status_reason_ = "pose too similar to an accepted view";
    } else if (!cooldown_ok) {
      status_reason_ = "save cooldown";
    } else if (!auto_save_) {
      status_reason_ = "ready (auto-save off, press SPACE)";
    } else {
      status_reason_ = "saving";
    }

    if (auto_save_ && margin_ok && blur_ok && diverse && cooldown_ok) {
      saveCurrent(false);
    }
  }

  bool refreshDetection(const cv::Mat& left, const cv::Mat& right) {
    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    cv::Size pattern;
    both_detected_ = detectBoth(left, right, left_corners, right_corners,
                                pattern);
    if (!both_detected_) {
      left_corners_.clear();
      right_corners_.clear();
      current_pattern_ = cv::Size(0, 0);
      left_blur_ = 0.0;
      right_blur_ = 0.0;
      min_margin_ = 0.0;
      pose_valid_ = false;
      if (status_reason_.empty() ||
          status_reason_.find("orientation mismatch") == std::string::npos) {
        status_reason_ = "board not detected in both views";
      }
      return false;
    }

    left_corners_ = std::move(left_corners);
    right_corners_ = std::move(right_corners);
    spr_calib::alignStereoCornerOrder(left_corners_, right_corners_);
    current_pattern_ = pattern;
    left_blur_ = spr_calib::laplacianVariance(left, left_corners_);
    right_blur_ = spr_calib::laplacianVariance(right, right_corners_);
    min_margin_ = std::min(minMargin(left_corners_, left.size()),
                           minMargin(right_corners_, right.size()));
    pose_valid_ = solveBoardPose(left_corners_, pattern, rvec_, tvec_);

    return true;
  }

  static std::string formatIndex(int index) {
    std::ostringstream os;
    os << std::setw(6) << std::setfill('0') << index;
    return os.str();
  }

  void saveCurrent(bool forced) {
    if (latest_left_gray_.empty() || latest_right_gray_.empty()) {
      RCLCPP_WARN(get_logger(), "cannot save: no current stereo image pair");
      return;
    }
    if (forced) {
      const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_detect_).count();
      if (age > 1.0) {
        status_reason_ = "manual save ignored: stereo snapshot is stale";
        RCLCPP_WARN(get_logger(), "%s (%.3f s)", status_reason_.c_str(), age);
        return;
      }
      // Re-detect these exact stored image buffers. Manual save may bypass
      // diversity/cooldown, but never corner completeness, blur, or margins.
      if (!refreshDetection(latest_left_gray_, latest_right_gray_)) {
        RCLCPP_WARN(get_logger(), "cannot save: current images do not contain both complete boards");
        return;
      }
    }
    if (!both_detected_ || left_corners_.size() != static_cast<size_t>(board_config_.cols * board_config_.rows) ||
        right_corners_.size() != left_corners_.size() || current_pattern_.width == 0) {
      RCLCPP_WARN(get_logger(), "cannot save: board not detected in both views");
      status_reason_ = "manual save ignored: no complete board";
      return;
    }
    if (min_margin_ < edge_margin_ || std::min(left_blur_, right_blur_) < blur_threshold_) {
      status_reason_ = min_margin_ < edge_margin_ ? "save rejected: board margin" : "save rejected: image blur";
      RCLCPP_WARN(get_logger(), "%s: blur %.1f/%.1f (need %.1f), margin %.1f (need %.1f)",
                   status_reason_.c_str(), left_blur_, right_blur_, blur_threshold_, min_margin_, edge_margin_);
      return;
    }

    const std::string name = formatIndex(next_index_);
    const fs::path left_path = session_dir_ / "left" / (name + ".png");
    const fs::path right_path = session_dir_ / "right" / (name + ".png");
    if (!cv::imwrite(left_path.string(), latest_left_gray_)) {
      RCLCPP_ERROR(get_logger(), "failed to write %s", left_path.c_str());
      return;
    }
    if (!cv::imwrite(right_path.string(), latest_right_gray_)) {
      fs::remove(left_path);
      RCLCPP_ERROR(get_logger(), "failed to write %s", right_path.c_str());
      return;
    }

    AcceptedView view;
    view.name = name;
    view.stamp_ns = latest_stamp_ns_;
    view.right_stamp_ns = latest_right_stamp_ns_;
    view.blur_left = left_blur_;
    view.blur_right = right_blur_;
    view.min_margin = min_margin_;
    view.pose_valid = pose_valid_;
    view.rvec = rvec_;
    view.tvec = tvec_;
    centroidScale(left_corners_, view.centroid, view.scale_px);
    view.bins = spr_calib::coverageBins(left_corners_, latest_left_gray_.size(),
                                        kBinsX, kBinsY);
    for (const int bin : view.bins) {
      ++coverage_counts_[static_cast<size_t>(bin)];
    }
    accepted_.push_back(std::move(view));
    ++next_index_;
    last_save_ = std::chrono::steady_clock::now();
    writeIndex();
    writeMeta();
    status_reason_ = forced ? "saved manually" : "saved automatically";
    RCLCPP_INFO(get_logger(), "saved %s (%s), accepted=%zu", name.c_str(),
                forced ? "manual" : "auto", accepted_.size());
  }

  void forceSave() { saveCurrent(true); }

  void deleteLast() {
    if (accepted_.empty()) {
      status_reason_ = "nothing to delete";
      return;
    }
    const AcceptedView view = accepted_.back();
    fs::remove(session_dir_ / "left" / (view.name + ".png"));
    fs::remove(session_dir_ / "right" / (view.name + ".png"));
    for (const int bin : view.bins) {
      int& count = coverage_counts_[static_cast<size_t>(bin)];
      count = std::max(0, count - 1);
    }
    accepted_.pop_back();
    writeIndex();
    writeMeta();
    status_reason_ = "deleted last pair";
    RCLCPP_INFO(get_logger(), "deleted %s, accepted=%zu", view.name.c_str(),
                accepted_.size());
  }

  int coveragePercent() const {
    const int hit = static_cast<int>(
        std::count_if(coverage_counts_.begin(), coverage_counts_.end(),
                      [](int count) { return count > 0; }));
    return static_cast<int>(std::lround(100.0 * hit / kBinCount));
  }

  void drawStatus(cv::Mat& image) const {
    std::vector<std::string> lines;
    lines.push_back("accepted: " + std::to_string(accepted_.size()) + " / " +
                    std::to_string(min_pairs_));
    lines.push_back("coverage: " + std::to_string(coveragePercent()) + "%");
    if (both_detected_) {
      std::ostringstream blur;
      blur << "blur L/R: " << std::fixed << std::setprecision(0)
           << left_blur_ << " / " << right_blur_ << " (min "
           << blur_threshold_ << ")";
      lines.push_back(blur.str());
      std::ostringstream margin;
      margin << "edge margin: " << std::fixed << std::setprecision(1)
             << min_margin_ << " px (min " << edge_margin_ << ")";
      lines.push_back(margin.str());
      lines.push_back("pattern: " + std::to_string(current_pattern_.width) +
                      "x" + std::to_string(current_pattern_.height));
    } else {
      lines.push_back("blur L/R: n/a");
      lines.push_back("edge margin: n/a");
      lines.push_back("pattern: searching");
    }
    lines.push_back("status: " + status_reason_);
    lines.push_back(
        "SPACE save | BACKSPACE delete last | R reset orientation | Q quit");

    const int line_height = 18;
    const int width = 520;
    const int height = 12 + line_height * static_cast<int>(lines.size());
    cv::rectangle(image, cv::Rect(0, 0, width, height),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    int y = 20;
    for (const std::string& line : lines) {
      cv::putText(image, line, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      y += line_height;
    }
  }

  void drawCoverage() const {
    cv::Mat heat(kBinsY, kBinsX, CV_32F);
    int max_count = 1;
    for (int index = 0; index < kBinCount; ++index) {
      const int count = coverage_counts_[static_cast<size_t>(index)];
      heat.at<float>(index / kBinsX, index % kBinsX) =
          static_cast<float>(count);
      max_count = std::max(max_count, count);
    }
    cv::Mat heat_big;
    cv::resize(heat, heat_big, cv::Size(480, 360), 0.0, 0.0,
               cv::INTER_NEAREST);
    heat_big = heat_big / static_cast<float>(max_count);
    cv::Mat heat_u8;
    heat_big.convertTo(heat_u8, CV_8U, 255.0);
    cv::Mat color;
    cv::applyColorMap(heat_u8, color, cv::COLORMAP_JET);
    for (int x = 0; x <= kBinsX; ++x) {
      const int px = x * 480 / kBinsX;
      cv::line(color, cv::Point(px, 0), cv::Point(px, 360),
               cv::Scalar(240, 240, 240), 1);
    }
    for (int y = 0; y <= kBinsY; ++y) {
      const int py = y * 360 / kBinsY;
      cv::line(color, cv::Point(0, py), cv::Point(480, py),
               cv::Scalar(240, 240, 240), 1);
    }
    cv::putText(color, "coverage " + std::to_string(coveragePercent()) + "%",
                cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::imshow("coverage", color);
  }

  void writeIndex() const {
    std::ofstream os(session_dir_ / "index.csv");
    if (!os) {
      throw std::runtime_error("cannot write capture index");
    }
    os << "name,stamp_ns,blur_left,blur_right,min_margin,pose_valid,"
          "rx,ry,rz,tx,ty,tz,centroid_x,centroid_y,scale_px,right_stamp_ns,stereo_delta_ns\n";
    os << std::setprecision(9);
    for (const AcceptedView& view : accepted_) {
      os << view.name << ',' << view.stamp_ns << ',' << view.blur_left << ','
         << view.blur_right << ',' << view.min_margin << ','
         << (view.pose_valid ? 1 : 0) << ',' << view.rvec[0] << ','
         << view.rvec[1] << ',' << view.rvec[2] << ',' << view.tvec[0] << ','
         << view.tvec[1] << ',' << view.tvec[2] << ',' << view.centroid.x << ','
         << view.centroid.y << ',' << view.scale_px << ',' << view.right_stamp_ns << ','
         << view.right_stamp_ns - view.stamp_ns << '\n';
    }
  }

  void writeMeta() const {
    YAML::Node meta;
    meta["schema_version"] = 1;
    meta["board_cols"] = board_config_.cols;
    meta["board_rows"] = board_config_.rows;
    meta["square_size_m"] = board_config_.square_m;
    meta["auto_orientation"] = board_config_.auto_orientation;
    meta["resolution_width"] = image_size_.width;
    meta["resolution_height"] = image_size_.height;
    meta["left_topic"] = left_image_topic_;
    meta["right_topic"] = right_image_topic_;
    meta["left_frame_id"] = left_frame_id_;
    meta["right_frame_id"] = right_frame_id_;
    meta["expected_encoding"] = expected_encoding_;
    meta["max_stereo_delta_sec"] = max_stereo_delta_sec_;
    meta["factory_valid"] = factory_left_valid_ && factory_right_valid_;
    if (factory_left_valid_) {
      meta["factory_k_left"] = spr_calib::matrixYaml(factory_k_left_);
      meta["factory_d_left"] = spr_calib::matrixYaml(factory_d_left_);
    }
    if (factory_right_valid_) {
      meta["factory_k_right"] = spr_calib::matrixYaml(factory_k_right_);
      meta["factory_d_right"] = spr_calib::matrixYaml(factory_d_right_);
    }
    meta["min_pairs"] = min_pairs_;
    meta["accepted_pairs"] = accepted_.size();
    meta["input_failed"] = input_failed_;
    meta["coverage_left_percent"] = coveragePercent();
    meta["status"] = accepted_.empty() ? "empty" : (input_failed_ ? "input_error" :
        (accepted_.size() >= static_cast<size_t>(min_pairs_) && coveragePercent() >= 80 ? "ready" : "incomplete"));
    meta["blur_threshold"] = blur_threshold_;
    meta["edge_margin"] = edge_margin_;
    meta["diversity_translation_m"] = diversity_translation_;
    meta["diversity_rotation_deg"] = diversity_rotation_deg_;
    meta["auto_save"] = auto_save_;
    meta["session_start"] = session_start_;
    meta["source"] = "derived_pngs; retain original rosbag2 for provenance";
    spr_calib::writeYaml(session_dir_ / "meta.yaml", meta);
  }

  // Parameters and state.
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_info_topic_;
  std::string right_info_topic_;
  std::string output_dir_;
  spr_calib::BoardConfig board_config_;
  bool auto_save_ = true;
  int min_pairs_ = 30;
  double blur_threshold_ = 100.0;
  double edge_margin_ = 15.0;
  double diversity_translation_ = 0.03;
  double diversity_rotation_deg_ = 8.0;
  double save_cooldown_ = 0.5;
  bool show_ui_ = true;
  double input_timeout_sec_ = 10.0;
  int expected_width_ = 0;
  int expected_height_ = 0;
  std::string expected_encoding_ = "mono8";
  double max_stereo_delta_sec_ = 0.001;
  bool input_failed_ = false;
  std::chrono::steady_clock::time_point last_input_{};
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  int detect_flags_ = spr_calib::kDetectFlagsFast;
  std::chrono::steady_clock::duration detect_period_{};

  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_left_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_right_;

  fs::path session_dir_;
  std::string session_start_;
  cv::Size image_size_{0, 0};
  bool factory_left_valid_ = false;
  bool factory_right_valid_ = false;
  cv::Mat factory_k_left_;
  cv::Mat factory_d_left_;
  cv::Mat factory_k_right_;
  cv::Mat factory_d_right_;

  cv::Mat latest_left_gray_;
  cv::Mat latest_right_gray_;
  int64_t latest_stamp_ns_ = 0;
  int64_t latest_right_stamp_ns_ = 0;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::vector<cv::Point2f> left_corners_;
  std::vector<cv::Point2f> right_corners_;
  cv::Size current_pattern_{0, 0};
  cv::Size locked_pattern_{0, 0};
  bool pattern_locked_ = false;
  bool both_detected_ = false;
  double left_blur_ = 0.0;
  double right_blur_ = 0.0;
  double min_margin_ = 0.0;
  bool pose_valid_ = false;
  cv::Vec3d rvec_{0.0, 0.0, 0.0};
  cv::Vec3d tvec_{0.0, 0.0, 0.0};
  std::string status_reason_ = "waiting for images";

  std::vector<AcceptedView> accepted_;
  std::array<int, kBinCount> coverage_counts_{};
  int next_index_ = 0;
  std::chrono::steady_clock::time_point last_detect_{};
  std::chrono::steady_clock::time_point last_save_{};
  bool quit_requested_ = false;
  bool conversion_error_logged_ = false;
  bool size_mismatch_logged_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<CaptureNode> node;
  try {
    node = std::make_shared<CaptureNode>();
  } catch (const std::exception& error) {
    std::cerr << "failed to start capture node: " << error.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->quitRequested()) {
    executor.spin_some();
    node->render();
    if (!node->showUi()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  bool ready = false;
  try {
    ready = node->finalize();
  } catch (const std::exception& error) {
    std::cerr << "failed to finalize capture: " << error.what() << "\n";
  }
  rclcpp::shutdown();
  return ready ? 0 : 1;
}
