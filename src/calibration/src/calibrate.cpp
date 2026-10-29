#include "board.hpp"
#include "yaml_io.hpp"
#include "stereo_uncertainty.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string session_dir;
  std::string output_dir;
  double target_rms = 0.5;
  double max_view_rms = 1.0;
  int min_views = 20;
  std::vector<std::string> exclude;
  int min_pairs = 30;
  double min_coverage = 0.8;
  double holdout_fraction = 0.2;
  bool diagnostic = false;
  bool self_test = false;
  bool verbose = false;
};

struct Meta {
  int board_cols = 8;
  int board_rows = 11;
  double square_m = 0.025;
  bool auto_orientation = true;
  int image_width = 0;
  int image_height = 0;
  std::string left_topic;
  std::string right_topic;
  bool factory_valid = false;
  cv::Mat factory_k_left;
  cv::Mat factory_d_left;
  cv::Mat factory_k_right;
  cv::Mat factory_d_right;
};

struct View {
  std::string name;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> left;
  std::vector<cv::Point2f> right;
  cv::Size pattern;
};

struct CalibResult {
  cv::Mat K0;
  cv::Mat D0;
  cv::Mat K1;
  cv::Mat D1;
  cv::Mat R;
  cv::Mat T;
  cv::Mat E;
  cv::Mat F;
  cv::Mat covariance0;
  cv::Mat covariance1;
  cv::Mat correlation0;
  cv::Mat correlation1;
  double information_condition0 = 0.0;
  double information_condition1 = 0.0;
  double mono_rms0 = 0.0;
  double mono_rms1 = 0.0;
  double stereo_rms = 0.0;
  std::vector<double> view_rms;
};

struct IterationRecord {
  int iteration = 0;
  int active_views = 0;
  double mono_rms0 = 0.0;
  double mono_rms1 = 0.0;
  double stereo_rms = 0.0;
  double max_view_rms = 0.0;
  std::string removed;
  double removed_rms = 0.0;
};

struct RunResult {
  bool converged = false;
  bool have_last = false;
  std::string failure_reason;
  std::vector<IterationRecord> history;
  std::vector<std::string> pruned;
  std::vector<View> active;
  CalibResult last_calib;
  CalibResult final_calib;
  spr_calib::StereoUncertainty stereo_uncertainty;
};

struct Validation {
  bool evaluated = false;
  double baseline_m = 0.0;
  double epipolar_rms_px = 0.0;
  double epipolar_mean_abs_px = 0.0;
  double coverage_left = 0.0;
  double coverage_right = 0.0;
  size_t holdout_views = 0;
  double holdout_rms_px = std::numeric_limits<double>::quiet_NaN();
  double holdout_max_view_rms_px = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::string> holdout_names;
};

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error(message);
}

std::string fmt(double value, int precision = 12) {
  std::ostringstream os;
  os << std::setprecision(precision) << value;
  return os.str();
}

std::vector<std::string> splitCsv(const std::string& text) {
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    parts.push_back(item.substr(begin, end - begin + 1));
  }
  return parts;
}

std::string normalizedStem(const std::string& name) {
  return fs::path(name).stem().string();
}

bool isExcluded(const std::string& name,
                const std::vector<std::string>& exclusions) {
  const std::string stem = normalizedStem(name);
  for (const std::string& item : exclusions) {
    if (stem == normalizedStem(item)) {
      return true;
    }
  }
  return false;
}

void printUsage() {
  std::cout
      << "Usage: calibrate --session <dir> [options]\n"
      << "       calibrate --self-test\n\n"
      << "Options:\n"
      << "  --session <dir>        Capture session directory (contains left/, "
         "right/, meta.yaml)\n"
      << "  --output <dir>         Output directory (default: "
         "<workspace>/results/<session>)\n"
      << "  --target-rms <px>      Stop pruning when stereo RMS <= this value "
         "(default 0.5)\n"
      << "  --max-view-rms <px>    Per-view RMS threshold (default 1.0)\n"
      << "  --min-views <n>        Never prune below this many views (default "
         "20)\n"
      << "  --min-pairs <n>        Minimum usable input pairs (default 30)\n"
      << "  --min-coverage <0..1>  Required 8x6 cell coverage in BOTH cameras (default 0.8)\n"
      << "  --holdout-fraction <f> Independent holdout fraction (default 0.2)\n"
      << "  --exclude <a,b,...>    Image names/stems to exclude\n"
      << "  --diagnostic           Real-data fit only; no calibration exports or formal success\n"
      << "  --self-test            Run a synthetic calibration self-test (no "
         "camera needed)\n"
      << "  --verbose              Print progress to stdout\n"
      << "  -h, --help             Show this help\n";
}

double parseDouble(const std::string& text, const std::string& option) {
  try {
    size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) {
      fail("invalid value for " + option + ": " + text);
    }
    return value;
  } catch (const std::exception&) {
    fail("invalid value for " + option + ": " + text);
  }
}

int parseInt(const std::string& text, const std::string& option) {
  try {
    size_t used = 0;
    const long value = std::stol(text, &used);
    if (used != text.size() || value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max()) {
      fail("invalid value for " + option + ": " + text);
    }
    return static_cast<int>(value);
  } catch (const std::exception&) {
    fail("invalid value for " + option + ": " + text);
  }
}

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const std::string& option) -> std::string {
      if (i + 1 >= argc) {
        fail("missing value for " + option);
      }
      return argv[++i];
    };
    if (arg == "--session") {
      opt.session_dir = next(arg);
    } else if (arg == "--output") {
      opt.output_dir = next(arg);
    } else if (arg == "--target-rms") {
      opt.target_rms = parseDouble(next(arg), arg);
    } else if (arg == "--max-view-rms") {
      opt.max_view_rms = parseDouble(next(arg), arg);
    } else if (arg == "--min-views") {
      opt.min_views = parseInt(next(arg), arg);
    } else if (arg == "--min-pairs") {
      opt.min_pairs = parseInt(next(arg), arg);
    } else if (arg == "--min-coverage") {
      opt.min_coverage = parseDouble(next(arg), arg);
    } else if (arg == "--holdout-fraction") {
      opt.holdout_fraction = parseDouble(next(arg), arg);
    } else if (arg == "--exclude") {
      const std::vector<std::string> parts = splitCsv(next(arg));
      opt.exclude.insert(opt.exclude.end(), parts.begin(), parts.end());
    } else if (arg == "--diagnostic") {
      opt.diagnostic = true;
    } else if (arg == "--self-test") {
      opt.self_test = true;
    } else if (arg == "--verbose") {
      opt.verbose = true;
    } else if (arg == "-h" || arg == "--help") {
      printUsage();
      std::exit(0);
    } else {
      fail("unknown argument: " + arg);
    }
  }

  if (!opt.self_test) {
    if (opt.session_dir.empty()) {
      fail("--session is required (or use --self-test)");
    }
  }
  if (opt.target_rms <= 0.0) {
    fail("--target-rms must be positive");
  }
  if (opt.max_view_rms <= 0.0) {
    fail("--max-view-rms must be positive");
  }
  if (opt.min_views < 3) {
    fail("--min-views must be at least 3");
  }
  if (opt.max_view_rms < opt.target_rms) {
    fail("--max-view-rms must be >= --target-rms");
  }
  if (opt.min_pairs < opt.min_views || opt.min_coverage < 0.0 ||
      opt.min_coverage > 1.0 || opt.holdout_fraction <= 0.0 || opt.holdout_fraction >= 0.5) {
    fail("invalid pair count, coverage, or holdout fraction");
  }
  if (!opt.diagnostic && (opt.min_pairs < 30 || opt.min_views < 20 ||
      opt.min_coverage < 0.8 || opt.target_rms > 0.5 || opt.max_view_rms > 1.0))
    fail("relaxed thresholds require --diagnostic; diagnostic runs cannot export calibration files");
  return opt;
}

Meta loadLegacyMeta(const fs::path& path) {
  cv::FileStorage fs(path.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    fail("cannot open meta file: " + path.string());
  }
  Meta meta;
  if (!fs["board_cols"].empty()) {
    fs["board_cols"] >> meta.board_cols;
  }
  if (!fs["board_rows"].empty()) {
    fs["board_rows"] >> meta.board_rows;
  }
  if (!fs["square_size_m"].empty()) {
    fs["square_size_m"] >> meta.square_m;
  }
  if (!fs["auto_orientation"].empty()) {
    int value = 1;
    fs["auto_orientation"] >> value;
    meta.auto_orientation = value != 0;
  }
  if (!fs["resolution_width"].empty()) {
    fs["resolution_width"] >> meta.image_width;
  }
  if (!fs["resolution_height"].empty()) {
    fs["resolution_height"] >> meta.image_height;
  }
  if (!fs["left_topic"].empty()) {
    fs["left_topic"] >> meta.left_topic;
  }
  if (!fs["right_topic"].empty()) {
    fs["right_topic"] >> meta.right_topic;
  }
  if (!fs["factory_valid"].empty()) {
    int value = 0;
    fs["factory_valid"] >> value;
    meta.factory_valid = value != 0;
  }
  if (meta.factory_valid) {
    fs["factory_k_left"] >> meta.factory_k_left;
    fs["factory_d_left"] >> meta.factory_d_left;
    fs["factory_k_right"] >> meta.factory_k_right;
    fs["factory_d_right"] >> meta.factory_d_right;
  }

  if (meta.board_cols < 2 || meta.board_rows < 2) {
    fail("invalid board dimensions in meta.yaml");
  }
  if (!std::isfinite(meta.square_m) || meta.square_m <= 0.0) {
    fail("invalid square_size_m in meta.yaml");
  }
  if (meta.image_width <= 0 || meta.image_height <= 0) {
    fail("missing resolution in meta.yaml");
  }
  return meta;
}

Meta loadMeta(const fs::path& path) {
  std::ifstream input(path);
  if (!input) fail("cannot open meta file: " + path.string());
  std::string first;
  std::getline(input, first);
  if (first.rfind("%YAML:1.0", 0) == 0) return loadLegacyMeta(path);
  const auto node = YAML::LoadFile(path.string());
  Meta meta;
  meta.board_cols = node["board_cols"].as<int>();
  meta.board_rows = node["board_rows"].as<int>();
  meta.square_m = node["square_size_m"].as<double>();
  meta.auto_orientation = node["auto_orientation"].as<bool>(true);
  meta.image_width = node["resolution_width"].as<int>();
  meta.image_height = node["resolution_height"].as<int>();
  meta.left_topic = node["left_topic"].as<std::string>("");
  meta.right_topic = node["right_topic"].as<std::string>("");
  meta.factory_valid = node["factory_valid"].as<bool>(false);
  if (meta.factory_valid) {
    meta.factory_k_left = spr_calib::yamlMatrix(node["factory_k_left"]);
    meta.factory_d_left = spr_calib::yamlMatrix(node["factory_d_left"]);
    meta.factory_k_right = spr_calib::yamlMatrix(node["factory_k_right"]);
    meta.factory_d_right = spr_calib::yamlMatrix(node["factory_d_right"]);
  }
  if (meta.board_cols < 2 || meta.board_rows < 2 || !std::isfinite(meta.square_m) ||
      meta.square_m <= 0.0 || meta.image_width <= 0 || meta.image_height <= 0)
    fail("invalid board geometry or missing resolution in meta.yaml");
  return meta;
}

bool isPng(const fs::path& path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext == ".png";
}

std::vector<View> loadViews(const Options& opt, const Meta& meta,
                            std::vector<std::string>& skipped) {
  const fs::path left_dir = fs::path(opt.session_dir) / "left";
  const fs::path right_dir = fs::path(opt.session_dir) / "right";
  if (!fs::is_directory(left_dir)) {
    fail("missing left image directory: " + left_dir.string());
  }
  if (!fs::is_directory(right_dir)) {
    fail("missing right image directory: " + right_dir.string());
  }

  std::vector<std::string> names;
  for (const auto& entry : fs::directory_iterator(left_dir)) {
    if (entry.is_regular_file() && isPng(entry.path())) {
      names.push_back(entry.path().stem().string());
    }
  }
  std::sort(names.begin(), names.end());
  if (names.empty()) {
    fail("no PNG images found in " + left_dir.string());
  }

  spr_calib::BoardConfig config;
  config.cols = meta.board_cols;
  config.rows = meta.board_rows;
  config.square_m = meta.square_m;
  config.auto_orientation = meta.auto_orientation;

  std::vector<View> views;
  views.reserve(names.size());
  for (const std::string& name : names) {
    if (isExcluded(name, opt.exclude)) {
      skipped.push_back(name + " (user excluded)");
      continue;
    }
    const fs::path left_path = left_dir / (name + ".png");
    const fs::path right_path = right_dir / (name + ".png");
    if (!fs::is_regular_file(right_path)) {
      skipped.push_back(name + " (missing right image)");
      continue;
    }
    const cv::Mat left = cv::imread(left_path.string(), cv::IMREAD_GRAYSCALE);
    const cv::Mat right = cv::imread(right_path.string(), cv::IMREAD_GRAYSCALE);
    if (left.empty() || right.empty()) {
      skipped.push_back(name + " (cannot read image)");
      continue;
    }
    if (left.cols != meta.image_width || left.rows != meta.image_height ||
        right.cols != meta.image_width || right.rows != meta.image_height) {
      skipped.push_back(name + " (resolution mismatch)");
      continue;
    }

    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    cv::Size left_pattern;
    cv::Size right_pattern;
    if (!spr_calib::detectBoard(left, config, left_corners, left_pattern) ||
        !spr_calib::detectBoard(right, config, right_corners, right_pattern)) {
      skipped.push_back(name + " (board detection failed)");
      continue;
    }
    if (left_pattern != right_pattern) {
      skipped.push_back(name + " (left/right board orientation mismatch)");
      continue;
    }
    spr_calib::alignStereoCornerOrder(left_corners, right_corners);

    View view;
    view.name = name;
    view.pattern = left_pattern;
    view.object_points = spr_calib::makeObjectPoints(
        left_pattern.width, left_pattern.height, meta.square_m);
    view.left = std::move(left_corners);
    view.right = std::move(right_corners);
    views.push_back(std::move(view));
  }
  return views;
}

// Marginalize each board pose from the projection Jacobian. This reports the
// local linearized covariance and correlation of [fx,fy,cx,cy,k1,k2,p1,p2].
// It is a noise-model estimate, not proof of physical calibration accuracy.
void intrinsicUncertainty(const std::vector<std::vector<cv::Point3f>>& objects,
                          const std::vector<std::vector<cv::Point2f>>& images,
                          const std::vector<cv::Mat>& rvecs,
                          const std::vector<cv::Mat>& tvecs,
                          const cv::Mat& K, const cv::Mat& D,
                          cv::Mat& covariance, cv::Mat& correlation,
                          double& condition) {
  cv::Mat information = cv::Mat::zeros(8, 8, CV_64F);
  double squared = 0.0;
  int residuals = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    cv::Mat jacobian;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objects[i], rvecs[i], tvecs[i], K, D, projected, jacobian);
    const cv::Mat pose = jacobian.colRange(0, 6);
    const cv::Mat intrinsics = jacobian.colRange(6, 14);
    cv::Mat inverse_pose;
    cv::invert(pose.t() * pose, inverse_pose, cv::DECOMP_SVD);
    const cv::Mat cross = intrinsics.t() * pose;
    information += intrinsics.t() * intrinsics - cross * inverse_pose * cross.t();
    for (size_t j = 0; j < projected.size(); ++j) {
      const auto error = projected[j] - images[i][j];
      squared += error.dot(error);
      residuals += 2;
    }
  }
  information = (information + information.t()) * 0.5;
  cv::Mat eigenvalues;
  cv::eigen(information, eigenvalues);
  const double smallest = eigenvalues.at<double>(7);
  condition = smallest > 0.0 ? eigenvalues.at<double>(0) / smallest :
                              std::numeric_limits<double>::infinity();
  if (!std::isfinite(condition)) {
    covariance = cv::Mat(8,8,CV_64F,cv::Scalar(std::numeric_limits<double>::quiet_NaN()));
    correlation = covariance.clone();
    return;
  }
  cv::invert(information, covariance, cv::DECOMP_SVD);
  covariance *= squared / std::max(1, residuals - 6 * static_cast<int>(objects.size()) - 8);
  correlation = cv::Mat::zeros(8, 8, CV_64F);
  for (int r = 0; r < 8; ++r) {
    for (int c = 0; c < 8; ++c) {
      const double denominator = std::sqrt(std::max(0.0, covariance.at<double>(r,r) * covariance.at<double>(c,c)));
      correlation.at<double>(r,c) = denominator > 0.0 ?
          std::clamp(covariance.at<double>(r,c) / denominator, -1.0, 1.0) : std::numeric_limits<double>::quiet_NaN();
    }
  }
}

CalibResult calibrateOnce(const std::vector<View>& views,
                          const cv::Size& image_size) {
  if (views.size() < 3) {
    fail("need at least 3 usable views");
  }

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points0;
  std::vector<std::vector<cv::Point2f>> image_points1;
  object_points.reserve(views.size());
  image_points0.reserve(views.size());
  image_points1.reserve(views.size());
  for (const View& view : views) {
    object_points.push_back(view.object_points);
    image_points0.push_back(view.left);
    image_points1.push_back(view.right);
  }

  const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 200, 1e-9);
  CalibResult result;
  std::vector<cv::Mat> rvecs0;
  std::vector<cv::Mat> tvecs0;
  std::vector<cv::Mat> rvecs1;
  std::vector<cv::Mat> tvecs1;

  result.mono_rms0 = cv::calibrateCamera(
      object_points, image_points0, image_size, result.K0, result.D0, rvecs0,
      tvecs0, cv::CALIB_FIX_K3, criteria);
  result.mono_rms1 = cv::calibrateCamera(
      object_points, image_points1, image_size, result.K1, result.D1, rvecs1,
      tvecs1, cv::CALIB_FIX_K3, criteria);

  intrinsicUncertainty(object_points, image_points0, rvecs0, tvecs0, result.K0,
                       result.D0, result.covariance0, result.correlation0,
                       result.information_condition0);
  intrinsicUncertainty(object_points, image_points1, rvecs1, tvecs1, result.K1,
                       result.D1, result.covariance1, result.correlation1,
                       result.information_condition1);

  result.stereo_rms = cv::stereoCalibrate(
      object_points, image_points0, image_points1, result.K0, result.D0,
      result.K1, result.D1, image_size, result.R, result.T, result.E, result.F,
      cv::CALIB_FIX_INTRINSIC, criteria);

  result.view_rms.assign(views.size(), 0.0);
  double total_squared = 0.0;
  size_t total_points = 0;
  for (size_t i = 0; i < views.size(); ++i) {
    cv::Mat R0;
    cv::Rodrigues(rvecs0[i], R0);
    const cv::Mat R1 = result.R * R0;
    const cv::Mat t1 = result.R * tvecs0[i] + result.T;
    cv::Mat rvec1;
    cv::Rodrigues(R1, rvec1);

    std::vector<cv::Point2f> projected0;
    std::vector<cv::Point2f> projected1;
    cv::projectPoints(views[i].object_points, rvecs0[i], tvecs0[i], result.K0,
                      result.D0, projected0);
    cv::projectPoints(views[i].object_points, rvec1, t1, result.K1, result.D1,
                      projected1);

    double squared = 0.0;
    for (size_t j = 0; j < projected0.size(); ++j) {
      const double dx0 = projected0[j].x - views[i].left[j].x;
      const double dy0 = projected0[j].y - views[i].left[j].y;
      const double dx1 = projected1[j].x - views[i].right[j].x;
      const double dy1 = projected1[j].y - views[i].right[j].y;
      squared += dx0 * dx0 + dy0 * dy0 + dx1 * dx1 + dy1 * dy1;
    }
    result.view_rms[i] =
        std::sqrt(squared / (2.0 * static_cast<double>(projected0.size())));
    total_squared += squared;
    total_points += 2 * projected0.size();
  }
  result.stereo_rms = std::sqrt(total_squared / static_cast<double>(total_points));
  if (!cv::checkRange(result.K0) || !cv::checkRange(result.K1) ||
      !cv::checkRange(result.D0) || !cv::checkRange(result.D1) ||
      !cv::checkRange(result.R) || !cv::checkRange(result.T) ||
      result.K0.at<double>(0,0) <= 0 || result.K0.at<double>(1,1) <= 0 ||
      result.K1.at<double>(0,0) <= 0 || result.K1.at<double>(1,1) <= 0 ||
      !std::isfinite(result.stereo_rms) || cv::norm(result.T) <= 0)
    fail("calibration returned non-finite or physically invalid parameters");
  return result;
}

bool uncertaintyAvailable(const CalibResult& result) {
  if (!std::isfinite(result.information_condition0) || !std::isfinite(result.information_condition1) ||
      result.information_condition0 > 1e12 || result.information_condition1 > 1e12 ||
      !cv::checkRange(result.covariance0) || !cv::checkRange(result.covariance1) ||
      !cv::checkRange(result.correlation0) || !cv::checkRange(result.correlation1)) return false;
  for (int i = 0; i < 8; ++i)
    if (result.covariance0.at<double>(i,i) < 0 || result.covariance1.at<double>(i,i) < 0) return false;
  return true;
}

double parameterStddev(const cv::Mat& covariance, int index) {
  const double variance = covariance.at<double>(index,index);
  return std::isfinite(variance) && variance >= 0 ? std::sqrt(variance) :
      std::numeric_limits<double>::quiet_NaN();
}

RunResult runCalibration(std::vector<View> views, const cv::Size& image_size,
                         const Options& opt) {
  RunResult run;
  if (views.size() < static_cast<size_t>(opt.min_views)) {
    run.failure_reason = "only " + std::to_string(views.size()) +
                         " usable views; need at least --min-views " +
                         std::to_string(opt.min_views);
    run.active = views;
    return run;
  }

  int iteration = 0;
  while (true) {
    CalibResult result;
    try {
      result = calibrateOnce(views, image_size);
    } catch (const std::exception& error) {
      run.failure_reason =
          std::string("OpenCV calibration failed: ") + error.what();
      return run;
    }
    run.have_last = true;
    run.last_calib = result;

    const double max_view_rms =
        *std::max_element(result.view_rms.begin(), result.view_rms.end());
    IterationRecord record;
    record.iteration = iteration;
    record.active_views = static_cast<int>(views.size());
    record.mono_rms0 = result.mono_rms0;
    record.mono_rms1 = result.mono_rms1;
    record.stereo_rms = result.stereo_rms;
    record.max_view_rms = max_view_rms;

    if (result.stereo_rms <= opt.target_rms &&
        max_view_rms <= opt.max_view_rms) {
      run.history.push_back(record);
      run.converged = true;
      run.active = views;
      run.final_calib = result;
      return run;
    }

    if (views.size() <= static_cast<size_t>(opt.min_views)) {
      run.history.push_back(record);
      run.failure_reason =
          "stereo RMS target not reached after pruning to --min-views (" +
          std::to_string(opt.min_views) + ")";
      run.active = views;
      return run;
    }

    const size_t worst = static_cast<size_t>(std::distance(
        result.view_rms.begin(),
        std::max_element(result.view_rms.begin(), result.view_rms.end())));
    record.removed = views[worst].name;
    record.removed_rms = result.view_rms[worst];
    run.history.push_back(record);
    run.pruned.push_back(views[worst].name);
    if (opt.verbose) {
      std::cout << "iteration " << iteration << ": views=" << views.size()
                << " stereo_rms=" << fmt(result.stereo_rms)
                << " max_view_rms=" << fmt(max_view_rms) << " -> prune "
                << views[worst].name << " (" << fmt(result.view_rms[worst])
                << " px)\n";
    }
    views.erase(views.begin() + static_cast<std::ptrdiff_t>(worst));
    ++iteration;
  }
}

Validation validate(const std::vector<View>& views, const CalibResult& calib,
                    const cv::Size& image_size) {
  Validation validation;
  validation.evaluated = true;
  validation.baseline_m = cv::norm(calib.T);

  cv::Mat R1;
  cv::Mat R2;
  cv::Mat P1;
  cv::Mat P2;
  cv::Mat Q;
  cv::stereoRectify(calib.K0, calib.D0, calib.K1, calib.D1, image_size,
                    calib.R, calib.T, R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, 0.0, image_size);

  double squared = 0.0;
  double absolute = 0.0;
  size_t count = 0;
  for (const View& view : views) {
    std::vector<cv::Point2f> rect0;
    std::vector<cv::Point2f> rect1;
    cv::undistortPoints(view.left, rect0, calib.K0, calib.D0, R1, P1);
    cv::undistortPoints(view.right, rect1, calib.K1, calib.D1, R2, P2);
    for (size_t j = 0; j < rect0.size(); ++j) {
      const double dy = rect0[j].y - rect1[j].y;
      squared += dy * dy;
      absolute += std::abs(dy);
      ++count;
    }
  }
  if (count > 0) {
    validation.epipolar_rms_px =
        std::sqrt(squared / static_cast<double>(count));
    validation.epipolar_mean_abs_px =
        absolute / static_cast<double>(count);
  }
  return validation;
}

double imageCoverage(const std::vector<View>& views, const cv::Size& size, bool left) {
  std::vector<bool> occupied(48, false);
  for (const auto& view : views) {
    for (int bin : spr_calib::coverageBins(left ? view.left : view.right, size, 8, 6))
      occupied[static_cast<size_t>(bin)] = true;
  }
  return std::count(occupied.begin(), occupied.end(), true) / 48.0;
}

void evaluateHoldout(const std::vector<View>& held_out, const CalibResult& calib,
                     Validation& validation) {
  double total = 0.0;
  size_t count = 0;
  validation.holdout_names.clear();
  validation.holdout_views = held_out.size();
  validation.holdout_max_view_rms_px = 0.0;
  for (const auto& view : held_out) {
    validation.holdout_names.push_back(view.name);
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(view.object_points, view.left, calib.K0, calib.D0, rvec, tvec))
      fail("holdout pose could not be solved: " + view.name);
    if (tvec.at<double>(2) <= 0) fail("holdout board is behind the camera: " + view.name);
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    cv::Mat right_rotation = calib.R * rotation;
    cv::Mat right_rvec;
    cv::Rodrigues(right_rotation, right_rvec);
    const cv::Mat right_tvec = calib.R * tvec + calib.T;
    std::vector<cv::Point2f> left, right;
    cv::projectPoints(view.object_points, rvec, tvec, calib.K0, calib.D0, left);
    cv::projectPoints(view.object_points, right_rvec, right_tvec, calib.K1, calib.D1, right);
    double squared = 0.0;
    for (size_t j = 0; j < left.size(); ++j) {
      const auto e0 = left[j] - view.left[j];
      const auto e1 = right[j] - view.right[j];
      squared += e0.dot(e0) + e1.dot(e1);
    }
    total += squared;
    count += 2 * left.size();
    validation.holdout_max_view_rms_px = std::max(validation.holdout_max_view_rms_px,
                                                std::sqrt(squared / (2 * left.size())));
  }
  if (count > 0) validation.holdout_rms_px = std::sqrt(total / count);
}

std::string matRow(const cv::Mat& matrix, int row) {
  std::ostringstream os;
  os << std::setprecision(12);
  for (int c = 0; c < matrix.cols; ++c) {
    if (c > 0) {
      os << ", ";
    }
    os << matrix.at<double>(row, c);
  }
  return os.str();
}

void writeCamchain(const fs::path& path, const Meta& meta,
                   const CalibResult& calib) {
  YAML::Node chain;
  const auto write_camera = [&](const char* name, const cv::Mat& K, const cv::Mat& D,
                                const std::string& topic) {
    const cv::Mat d = D.reshape(1,1);
    if (d.total() < 4) fail("distortion vector has fewer than four coefficients");
    auto camera = chain[name];
    camera["camera_model"] = "pinhole";
    camera["intrinsics"] = std::vector<double>{K.at<double>(0,0), K.at<double>(1,1),
                                               K.at<double>(0,2), K.at<double>(1,2)};
    camera["distortion_model"] = "radtan";
    camera["distortion_coeffs"] = std::vector<double>{d.at<double>(0,0), d.at<double>(0,1),
                                                      d.at<double>(0,2), d.at<double>(0,3)};
    camera["resolution"] = std::vector<int>{meta.image_width, meta.image_height};
    camera["rostopic"] = topic;
  };
  write_camera("cam0", calib.K0, calib.D0,
               meta.left_topic.empty() ? "/camera/infra1/image_rect_raw" : meta.left_topic);
  write_camera("cam1", calib.K1, calib.D1,
               meta.right_topic.empty() ? "/camera/infra2/image_rect_raw" : meta.right_topic);
  cv::Mat transform = cv::Mat::eye(4,4,CV_64F);
  calib.R.copyTo(transform(cv::Rect(0,0,3,3)));
  calib.T.copyTo(transform(cv::Rect(3,0,1,3)));
  chain["cam1"]["T_cn_cnm1"] = spr_calib::matrixYaml(transform);
  spr_calib::writeYaml(path, chain);
}

cv::Mat distortion4(const cv::Mat& D5) {
  const cv::Mat as_row = D5.reshape(1, 1);
  return as_row.colRange(0, 4).clone();
}

void writeMonoYaml(const fs::path& path, const cv::Mat& K, const cv::Mat& D5,
                   int width, int height, double rms) {
  YAML::Node node;
  node["camera_matrix"] = spr_calib::matrixYaml(K);
  node["distortion_coefficients"] = spr_calib::matrixYaml(distortion4(D5));
  node["image_width"] = width;
  node["image_height"] = height;
  node["rms"] = rms;
  node["distortion_model"] = "radtan";
  spr_calib::writeYaml(path, node);
}

void writeExcluded(const fs::path& path,
                   const std::vector<std::string>& skipped,
                   const RunResult& run) {
  std::ofstream os(path);
  if (!os) {
    fail("cannot write " + path.string());
  }
  for (const std::string& item : skipped) {
    os << item << "\n";
  }
  for (const IterationRecord& record : run.history) {
    if (!record.removed.empty()) {
      os << record.removed << " (pruned at iteration " << record.iteration
         << ", view RMS " << std::setprecision(12) << record.removed_rms
         << " px)\n";
    }
  }
}

void writeTextReport(const fs::path& path, const Options& opt, const Meta& meta,
                     size_t input_views, const std::vector<std::string>& skipped,
                     const RunResult& run, const Validation& validation) {
  std::ofstream os(path);
  if (!os) {
    fail("cannot write " + path.string());
  }
  os << std::setprecision(12);
  os << "D435i IR stereo calibration report\n"
     << "session: " << fs::absolute(opt.session_dir).string() << "\n"
     << "output: " << fs::absolute(opt.output_dir).string() << "\n"
     << "image size: " << meta.image_width << "x" << meta.image_height << "\n"
     << "board: " << meta.board_cols << "x" << meta.board_rows
     << " inner corners, square " << meta.square_m << " m, auto orientation "
     << (meta.auto_orientation ? "on" : "off") << "\n\n"
     << "Inputs:\n"
     << "  detected views: " << input_views << "\n"
     << "  skipped views: " << skipped.size() << "\n"
     << "  pruned views: " << run.pruned.size() << "\n"
     << "  final views: " << run.active.size() << "\n\n"
     << "Result: " << (opt.diagnostic ? (run.converged ? "DIAGNOSTIC FIT ONLY" : "DIAGNOSTIC FAILURE") :
                            (run.converged ? "SUCCESS" : "FAILURE")) << "\n";
  if (!run.converged) {
    os << "  reason: " << run.failure_reason << "\n";
  }
  os << "  target stereo RMS: " << opt.target_rms << " px\n"
     << "  max per-view RMS: " << opt.max_view_rms << " px\n"
     << "  min views: " << opt.min_views << "\n"
     << "  coverage cam0/cam1: " << validation.coverage_left * 100 << "/"
     << validation.coverage_right * 100 << "% (required " << opt.min_coverage * 100 << "%)\n"
     << "  independent holdout: " << validation.holdout_views << " views, RMS "
     << validation.holdout_rms_px << " px, max " << validation.holdout_max_view_rms_px << " px\n"
     << "  uncertainty/correlation: see report.yaml (local linearized estimate)\n";
  os << "  joint stereo uncertainty: " << (run.stereo_uncertainty.available ? "available" : "unavailable")
     << " (" << run.stereo_uncertainty.reason << ")\n";
  if (run.stereo_uncertainty.available) {
    const auto& sigma = run.stereo_uncertainty.stddev;
    os << "  stereo rotation stddev [deg]: " << sigma[16]*180/CV_PI << ", "
       << sigma[17]*180/CV_PI << ", " << sigma[18]*180/CV_PI << "\n"
       << "  stereo translation stddev [mm]: " << sigma[19]*1000 << ", "
       << sigma[20]*1000 << ", " << sigma[21]*1000 << "\n";
  }

  if (run.have_last) {
    const CalibResult& result = run.converged ? run.final_calib : run.last_calib;
    const double max_view_rms =
        result.view_rms.empty()
            ? 0.0
            : *std::max_element(result.view_rms.begin(), result.view_rms.end());
    os << "\nCalibration metrics:\n"
       << "  stereo RMS: " << result.stereo_rms << " px\n"
       << "  mono RMS cam0: " << result.mono_rms0 << " px\n"
       << "  mono RMS cam1: " << result.mono_rms1 << " px\n"
       << "  max view RMS: " << max_view_rms << " px\n"
       << "  baseline: " << cv::norm(result.T) << " m\n";
    if (run.converged) {
      os << "  epipolar RMS (rectified |dy|): "
         << validation.epipolar_rms_px << " px\n"
         << "  epipolar mean |dy|: " << validation.epipolar_mean_abs_px
         << " px\n";
    }
    os << "\ncam0 intrinsics: [" << matRow(result.K0, 0) << ", "
       << matRow(result.K0, 1) << ", " << matRow(result.K0, 2) << "]\n"
       << "cam0 distortion: [" << matRow(result.D0.reshape(1,1), 0) << "]\n"
       << "cam1 intrinsics: [" << matRow(result.K1, 0) << ", "
       << matRow(result.K1, 1) << ", " << matRow(result.K1, 2) << "]\n"
       << "cam1 distortion: [" << matRow(result.D1.reshape(1,1), 0) << "]\n"
       << "T_cam1_cam0:\n";
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    result.R.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    result.T.copyTo(transform(cv::Rect(3, 0, 1, 3)));
    for (int r = 0; r < 4; ++r) {
      os << "  " << matRow(transform, r) << "\n";
    }

    if (meta.factory_valid && !meta.factory_k_left.empty() &&
        !meta.factory_k_right.empty()) {
      os << "\nFactory intrinsics comparison:\n";
      auto compare = [&](const std::string& label, const cv::Mat& factory,
                         const cv::Mat& estimated) {
        os << "  " << label << ": fx " << estimated.at<double>(0, 0) << " vs "
           << factory.at<double>(0, 0) << " ("
           << (estimated.at<double>(0, 0) - factory.at<double>(0, 0)) << "), "
           << "fy " << estimated.at<double>(1, 1) << " vs "
           << factory.at<double>(1, 1) << " ("
           << (estimated.at<double>(1, 1) - factory.at<double>(1, 1)) << "), "
           << "cx " << estimated.at<double>(0, 2) << " vs "
           << factory.at<double>(0, 2) << " ("
           << (estimated.at<double>(0, 2) - factory.at<double>(0, 2)) << "), "
           << "cy " << estimated.at<double>(1, 2) << " vs "
           << factory.at<double>(1, 2) << " ("
           << (estimated.at<double>(1, 2) - factory.at<double>(1, 2))
           << ")\n";
      };
      compare("cam0", meta.factory_k_left, result.K0);
      compare("cam1", meta.factory_k_right, result.K1);
    }

    os << "\nIterations:\n";
    for (const IterationRecord& record : run.history) {
      os << "  [" << record.iteration << "] views=" << record.active_views
         << " mono=" << record.mono_rms0 << "/" << record.mono_rms1
         << " stereo=" << record.stereo_rms
         << " max_view=" << record.max_view_rms;
      if (!record.removed.empty()) {
        os << " removed=" << record.removed << " (" << record.removed_rms
           << " px)";
      }
      os << "\n";
    }

    os << "\nFinal per-view RMS:\n";
    for (size_t i = 0; i < run.active.size(); ++i) {
      os << "  " << run.active[i].name << " " << result.view_rms[i] << "\n";
    }
  }

  os << "\nSkipped and pruned images:\n";
  for (const std::string& item : skipped) {
    os << "  " << item << "\n";
  }
  for (const IterationRecord& record : run.history) {
    if (!record.removed.empty()) {
      os << "  " << record.removed << " (pruned, " << record.removed_rms
         << " px)\n";
    }
  }
  os << "\n";
}

void writeYamlReport(const fs::path& path, const Options& opt,
                     const Meta& meta, size_t input_views,
                     const std::vector<std::string>& skipped,
                     const RunResult& run, const Validation& validation) {
  YAML::Node report;
  report["schema_version"] = 1;
  report["quality_scope"] = "intrinsic/stereo image calibration only; not camera-IMU or flight acceptance";
  report["opencv_version"] = CV_VERSION;
  report["success"] = run.converged && !opt.diagnostic;
  report["fit_succeeded"] = run.converged;
  report["diagnostic"] = opt.diagnostic;
  report["formal_eligible"] = run.converged && !opt.diagnostic &&
      opt.min_pairs >= 30 && opt.min_views >= 20 && opt.min_coverage >= 0.8 &&
      opt.target_rms <= 0.5 && opt.max_view_rms <= 1.0 &&
      run.have_last && uncertaintyAvailable(run.final_calib) && run.stereo_uncertainty.available;
  report["failure_reason"] = run.failure_reason;
  report["target_rms"] = opt.target_rms;
  report["max_view_rms"] = opt.max_view_rms;
  report["min_views"] = opt.min_views;
  report["min_pairs"] = opt.min_pairs;
  report["min_coverage"] = opt.min_coverage;
  report["image_width"] = meta.image_width;
  report["image_height"] = meta.image_height;
  report["board_cols"] = meta.board_cols;
  report["board_rows"] = meta.board_rows;
  report["square_size_m"] = meta.square_m;
  report["input_views"] = input_views;
  report["skipped_views"] = skipped.size();
  report["pruned_views"] = run.pruned.size();
  report["final_views"] = run.active.size();
  if (run.have_last) report["baseline_m"] = cv::norm(run.last_calib.T);
  else report["baseline_m"] = YAML::Node();
  if (validation.evaluated) {
    report["epipolar_rms_px"] = validation.epipolar_rms_px;
    report["epipolar_mean_abs_px"] = validation.epipolar_mean_abs_px;
  } else {
    report["epipolar_rms_px"] = YAML::Node();
    report["epipolar_mean_abs_px"] = YAML::Node();
  }
  report["coverage_left"] = validation.coverage_left;
  report["coverage_right"] = validation.coverage_right;
  report["coverage_definition"] = "fraction of 8x6 cells containing at least one final training corner";
  report["holdout_views"] = validation.holdout_views;
  if (std::isfinite(validation.holdout_rms_px)) {
    report["holdout_rms_px"] = validation.holdout_rms_px;
    report["holdout_max_view_rms_px"] = validation.holdout_max_view_rms_px;
  } else {
    report["holdout_rms_px"] = YAML::Node();
    report["holdout_max_view_rms_px"] = YAML::Node();
  }
  report["holdout_names"] = validation.holdout_names;
  report["holdout_pose_method"] = "left solvePnP only; right pose predicted using fixed stereo transform";
  report["uncertainty_method"] = "linearized pixel-noise covariance; board poses marginalized by Schur complement";
  report["uncertainty_scope"] = "separate monocular8-parameter and stereo22-parameter local information covariance; conditional on exact target geometry and the pixel-noise model";
  report["bootstrap_performed"] = false;
  const auto& joint = run.stereo_uncertainty;
  report["stereo_uncertainty_available"] = joint.available;
  report["stereo_uncertainty_reason"] = joint.reason;
  report["stereo_uncertainty_method"] = "22 global parameters; per-view6 board poses jointly refined with both images and marginalized using orthogonal projection; column-scaled SVD, no priors/pseudoinverse/eigenvalue clamping";
  report["stereo_uncertainty_parameter_order"] = std::vector<std::string>{
      "cam0_fx","cam0_fy","cam0_cx","cam0_cy","cam0_k1","cam0_k2","cam0_p1","cam0_p2",
      "cam1_fx","cam1_fy","cam1_cx","cam1_cy","cam1_k1","cam1_k2","cam1_p1","cam1_p2",
      "stereo_rx","stereo_ry","stereo_rz","stereo_tx","stereo_ty","stereo_tz"};
  report["stereo_uncertainty_parameter_units"] = std::vector<std::string>{
      "px","px","px","px","dimensionless","dimensionless","dimensionless","dimensionless",
      "px","px","px","px","dimensionless","dimensionless","dimensionless","dimensionless",
      "rad","rad","rad","m","m","m"};
  report["stereo_uncertainty_rotation_convention"] = "R'=Exp(delta_theta)*R_cam1_cam0; T'=T_cam1_cam0+delta_t (translation held fixed for a rotation perturbation)";
  report["stereo_information_rank"] = joint.rank;
  report["stereo_information_rank_relative_tolerance"] = joint.rank_relative_tolerance;
  report["stereo_board_poses_refined"] = joint.fitted_board_poses;
  report["stereo_transform_jacobian_verification"] = "analytic Lie-rotation/translation derivative checked against independent double-precision central differences at every fitted view";
  report["stereo_transform_jacobian_max_relative_error"] = joint.jacobian_max_relative_error;
  report["stereo_nominal_transform_unchanged"] = joint.nominal_transform_unchanged;
  if (std::isfinite(joint.scaled_condition)) report["stereo_information_scaled_condition"] = joint.scaled_condition;
  else report["stereo_information_scaled_condition"] = YAML::Node();
  if (joint.available) {
    report["stereo_global_covariance"] = spr_calib::matrixYaml(joint.covariance);
    report["stereo_global_correlation"] = spr_calib::matrixYaml(joint.correlation);
    report["stereo_global_parameter_stddev"] = joint.stddev;
    report["stereo_extrinsic_covariance"] = spr_calib::matrixYaml(joint.covariance(cv::Rect(16,16,6,6)));
    report["stereo_extrinsic_correlation"] = spr_calib::matrixYaml(joint.correlation(cv::Rect(16,16,6,6)));
    report["stereo_intrinsic_extrinsic_cross_correlation"] = spr_calib::matrixYaml(joint.correlation(cv::Rect(16,0,6,16)));
    report["stereo_residual_variance_px2"] = joint.residual_variance_px2;
    report["stereo_uncertainty_dof"] = joint.degrees_of_freedom;
    report["stereo_joint_board_pose_rms_px"] = joint.fitted_rms_px;
    report["stereo_rotation_stddev_deg"] = std::vector<double>{joint.stddev[16]*180/CV_PI,joint.stddev[17]*180/CV_PI,joint.stddev[18]*180/CV_PI};
    report["stereo_translation_stddev_mm"] = std::vector<double>{joint.stddev[19]*1000,joint.stddev[20]*1000,joint.stddev[21]*1000};
  } else {
    for (const auto* key : {"stereo_global_covariance","stereo_global_correlation","stereo_global_parameter_stddev",
                            "stereo_extrinsic_covariance","stereo_extrinsic_correlation","stereo_intrinsic_extrinsic_cross_correlation",
                            "stereo_residual_variance_px2","stereo_uncertainty_dof","stereo_joint_board_pose_rms_px",
                            "stereo_rotation_stddev_deg","stereo_translation_stddev_mm"}) report[key] = YAML::Node();
  }
  report["uncertainty_limitations"] = std::vector<std::string>{
      "No bootstrap or measured calibration repeatability",
      "Stereo covariance is a local information approximation at the current sequential-calibration estimates, not empirical sampling uncertainty",
      "Target manufacturing, rolling-shutter, exposure, and other systematic errors are not modeled"};
  report["uncertainty_parameter_order"] = std::vector<std::string>{"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"};
  if (run.have_last) {
    const auto& result = run.converged ? run.final_calib : run.last_calib;
    report["stereo_rms"] = result.stereo_rms;
    report["mono_rms_cam0"] = result.mono_rms0;
    report["mono_rms_cam1"] = result.mono_rms1;
    report["cam0_intrinsics"] = spr_calib::matrixYaml(result.K0);
    report["cam0_distortion"] = spr_calib::matrixYaml(distortion4(result.D0));
    report["cam1_intrinsics"] = spr_calib::matrixYaml(result.K1);
    report["cam1_distortion"] = spr_calib::matrixYaml(distortion4(result.D1));
    report["R_cam1_cam0"] = spr_calib::matrixYaml(result.R);
    report["T_cam1_cam0"] = spr_calib::matrixYaml(result.T);
    report["cam0_parameter_covariance"] = spr_calib::matrixYaml(result.covariance0);
    report["cam1_parameter_covariance"] = spr_calib::matrixYaml(result.covariance1);
    report["cam0_parameter_correlation"] = spr_calib::matrixYaml(result.correlation0);
    report["cam1_parameter_correlation"] = spr_calib::matrixYaml(result.correlation1);
    report["cam0_information_condition"] = result.information_condition0;
    report["cam1_information_condition"] = result.information_condition1;
    YAML::Node std0(YAML::NodeType::Sequence), std1(YAML::NodeType::Sequence);
    for (int i = 0; i < 8; ++i) {
      std0.push_back(parameterStddev(result.covariance0, i));
      std1.push_back(parameterStddev(result.covariance1, i));
    }
    report["cam0_parameter_stddev"] = std0;
    report["cam1_parameter_stddev"] = std1;
    const std::vector<std::string> parameter_names{"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"};
    for (int camera = 0; camera < 2; ++camera) {
      const auto& covariance = camera == 0 ? result.covariance0 : result.covariance1;
      const auto& K = camera == 0 ? result.K0 : result.K1;
      const std::vector<double> limits{K.at<double>(0,0)*0.01, K.at<double>(1,1)*0.01,
                                       2.0, 2.0, 0.05, 0.05, 0.005, 0.005};
      for (int i = 0; i < 8; ++i) {
        const auto key = "cam" + std::to_string(camera) + "_parameter_status";
        const double standard_deviation = parameterStddev(covariance, i);
        report[key][parameter_names[static_cast<size_t>(i)]]["stddev_warning_limit"] = limits[static_cast<size_t>(i)];
        report[key][parameter_names[static_cast<size_t>(i)]]["status"] =
            !uncertaintyAvailable(result) || !std::isfinite(standard_deviation) || standard_deviation > limits[static_cast<size_t>(i)]
            ? "RED: high local uncertainty" : "estimated";
      }
    }
    report["uncertainty_status"] = !uncertaintyAvailable(result)
        ? "RED: poorly conditioned intrinsic information; recollect diverse poses" : "estimated";
    YAML::Node per_view;
    for (size_t i = 0; i < run.active.size() && i < result.view_rms.size(); ++i)
      per_view[run.active[i].name] = result.view_rms[i];
    report["final_per_view_rms_px"] = per_view;
  }
  YAML::Node history(YAML::NodeType::Sequence);
  for (const auto& record : run.history) {
    YAML::Node node;
    node["iteration"] = record.iteration;
    node["active_views"] = record.active_views;
    node["stereo_rms"] = record.stereo_rms;
    node["max_view_rms"] = record.max_view_rms;
    node["removed"] = record.removed;
    node["removed_rms"] = record.removed_rms;
    history.push_back(node);
  }
  report["iterations"] = history;
  std::vector<std::string> names;
  for (const auto& view : run.active) names.push_back(view.name);
  report["final_view_names"] = names;
  report["skipped"] = skipped;
  spr_calib::writeYaml(path, report);
}

struct SyntheticResult {
  bool pass = false;
  bool pruned_outliers = false;
  CalibResult calib;
  RunResult run;
  cv::Mat K0_gt;
  cv::Mat K1_gt;
  cv::Mat D0_gt;
  cv::Mat D1_gt;
  cv::Mat R_gt;
  cv::Mat T_gt;
};

SyntheticResult runSyntheticSelfTest() {
  const cv::Size image_size(848, 480);
  SyntheticResult out;
  out.K0_gt = (cv::Mat_<double>(3, 3) << 420.0, 0.0, 424.0, 0.0, 420.0, 240.0,
               0.0, 0.0, 1.0);
  out.K1_gt = (cv::Mat_<double>(3, 3) << 418.0, 0.0, 428.0, 0.0, 418.0, 238.0,
               0.0, 0.0, 1.0);
  out.D0_gt = (cv::Mat_<double>(5, 1) << -0.020, 0.010, 0.0005, -0.0003, 0.0);
  out.D1_gt = (cv::Mat_<double>(5, 1) << -0.025, 0.012, -0.0004, 0.0002, 0.0);
  const cv::Mat rvec_gt =
      (cv::Mat_<double>(3, 1) << 0.005, -0.010, 0.003);
  cv::Rodrigues(rvec_gt, out.R_gt);
  out.T_gt = (cv::Mat_<double>(3, 1) << -0.050, 0.001, 0.001);

  const spr_calib::BoardConfig board;
  const std::vector<cv::Point3f> object_points =
      spr_calib::makeObjectPoints(board.cols, board.rows, board.square_m);

  cv::RNG rng(12345);
  std::vector<View> views;
  int generated = 0;
  int attempts = 0;
  while (generated < 35 && attempts < 20000) {
    ++attempts;
    const cv::Vec3d rvec(rng.uniform(-0.45, 0.45), rng.uniform(-0.45, 0.45),
                         rng.uniform(-0.45, 0.45));
    const cv::Vec3d tvec(rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08),
                         rng.uniform(0.35, 0.75));
    std::vector<cv::Point2f> left;
    std::vector<cv::Point2f> right;
    cv::projectPoints(object_points, rvec, tvec, out.K0_gt, out.D0_gt, left);

    cv::Mat R0;
    cv::Rodrigues(cv::Mat(rvec), R0);
    const cv::Mat t0 = cv::Mat(tvec);
    cv::Mat R1 = out.R_gt * R0;
    const cv::Mat t1 = out.R_gt * t0 + out.T_gt;
    cv::Mat rvec1;
    cv::Rodrigues(R1, rvec1);
    cv::projectPoints(object_points, rvec1, t1, out.K1_gt, out.D1_gt, right);

    bool inside = true;
    for (size_t j = 0; j < left.size(); ++j) {
      if (left[j].x < 5 || left[j].y < 5 || left[j].x > image_size.width - 6 ||
          left[j].y > image_size.height - 6 || right[j].x < 5 ||
          right[j].y < 5 || right[j].x > image_size.width - 6 ||
          right[j].y > image_size.height - 6) {
        inside = false;
        break;
      }
    }
    if (!inside) {
      continue;
    }

    for (size_t j = 0; j < left.size(); ++j) {
      left[j].x += static_cast<float>(rng.gaussian(0.1));
      left[j].y += static_cast<float>(rng.gaussian(0.1));
      right[j].x += static_cast<float>(rng.gaussian(0.1));
      right[j].y += static_cast<float>(rng.gaussian(0.1));
    }

    View view;
    view.name = "synth_" + std::to_string(generated);
    view.object_points = object_points;
    view.left = std::move(left);
    view.right = std::move(right);
    view.pattern = cv::Size(board.cols, board.rows);
    views.push_back(std::move(view));
    ++generated;
  }
  if (generated < 30) {
    return out;
  }

  // Holdout points are never passed to the optimizer or outlier pruning.
  std::vector<View> held_out(views.end() - 7, views.end());
  views.resize(views.size() - 7);

  // Two deliberately corrupted views exercise the pruning path.
  for (int k = 0; k < 2; ++k) {
    View outlier = views[static_cast<size_t>(k)];
    outlier.name = "outlier_" + std::to_string(k);
    for (size_t j = 0; j < outlier.left.size(); ++j) {
      if (j % 4 == 0) {
        outlier.left[j].x += 20.0F;
        outlier.right[j].y -= 20.0F;
      }
    }
    views.push_back(std::move(outlier));
  }

  Options opt;
  opt.session_dir = "synthetic";
  opt.target_rms = 0.5;
  opt.max_view_rms = 1.0;
  opt.min_views = 20;
  out.run = runCalibration(views, image_size, opt);
  out.calib = out.run.converged ? out.run.final_calib : out.run.last_calib;
  if (!out.run.converged) {
    return out;
  }

  const auto relative = [](double estimate, double truth) {
    return std::abs(estimate - truth) / std::max(1e-12, std::abs(truth));
  };
  const bool intrinsics_ok =
      relative(out.calib.K0.at<double>(0, 0), out.K0_gt.at<double>(0, 0)) <
          0.005 &&
      relative(out.calib.K0.at<double>(1, 1), out.K0_gt.at<double>(1, 1)) <
          0.005 &&
      relative(out.calib.K1.at<double>(0, 0), out.K1_gt.at<double>(0, 0)) <
          0.005 &&
      relative(out.calib.K1.at<double>(1, 1), out.K1_gt.at<double>(1, 1)) <
          0.005 &&
      std::abs(out.calib.K0.at<double>(0, 2) - out.K0_gt.at<double>(0, 2)) <
          1.0 &&
      std::abs(out.calib.K0.at<double>(1, 2) - out.K0_gt.at<double>(1, 2)) <
          1.0 &&
      std::abs(out.calib.K1.at<double>(0, 2) - out.K1_gt.at<double>(0, 2)) <
          1.0 &&
      std::abs(out.calib.K1.at<double>(1, 2) - out.K1_gt.at<double>(1, 2)) <
          1.0;

  const cv::Mat d0 = distortion4(out.calib.D0);
  const cv::Mat d1 = distortion4(out.calib.D1);
  const cv::Mat d0_gt = distortion4(out.D0_gt);
  const cv::Mat d1_gt = distortion4(out.D1_gt);
  bool distortion_ok = true;
  for (int i = 0; i < 4; ++i) {
    distortion_ok =
        distortion_ok &&
        std::abs(d0.at<double>(0, i) - d0_gt.at<double>(0, i)) < 0.01 &&
        std::abs(d1.at<double>(0, i) - d1_gt.at<double>(0, i)) < 0.01;
  }

  const double rotation_error =
      spr_calib::rotationAngleDeg(out.calib.R, out.R_gt);
  const double translation_error = cv::norm(out.calib.T - out.T_gt);
  const auto pruned_count = std::count_if(out.run.pruned.begin(), out.run.pruned.end(),
      [](const std::string& name) { return name.rfind("outlier_", 0) == 0; });
  out.pruned_outliers = pruned_count == 2;
  Validation held_validation = validate(out.run.active, out.calib, image_size);
  held_validation.coverage_left = imageCoverage(out.run.active, image_size, true);
  held_validation.coverage_right = imageCoverage(out.run.active, image_size, false);
  evaluateHoldout(held_out, out.calib, held_validation);
  const bool uncertainty_ok = cv::checkRange(out.calib.covariance0) &&
      cv::checkRange(out.calib.covariance1) &&
      out.calib.covariance0.at<double>(0,0) > 0 &&
      out.calib.covariance1.at<double>(0,0) > 0 &&
      std::abs(out.calib.correlation0.at<double>(0,0) - 1.0) < 1e-9;
  bool export_ok = false;
  const auto temp = fs::temp_directory_path() / ("spr_stereo_selftest_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(temp);
  try {
    Meta meta;
    meta.image_width = image_size.width;
    meta.image_height = image_size.height;
    writeCamchain(temp / "camchain.yaml", meta, out.calib);
    writeMonoYaml(temp / "cam0.yaml", out.calib.K0, out.calib.D0,
                  image_size.width, image_size.height, out.calib.mono_rms0);
    writeYamlReport(temp / "report.yaml", opt, meta, 35, {}, out.run, held_validation);
    const auto chain = YAML::LoadFile((temp / "camchain.yaml").string());
    const auto report = YAML::LoadFile((temp / "report.yaml").string());
    const auto mono = YAML::LoadFile((temp / "cam0.yaml").string());
    export_ok = chain["cam0"]["distortion_coeffs"].size() == 4 &&
        chain["cam1"]["T_cn_cnm1"].size() == 4 &&
        chain["cam1"]["T_cn_cnm1"][0][3].as<double>() < 0 &&
        !chain["cam0"]["T_cam_imu"] &&
        report["success"].as<bool>() && report["holdout_views"].as<int>() == 7 &&
        report["cam0_parameter_correlation"].size() == 8 &&
        cv::norm(spr_calib::yamlMatrix(mono["camera_matrix"]) - out.calib.K0) < 1e-9;
  } catch (...) {
    fs::remove_all(temp);
    throw;
  }
  fs::remove_all(temp);
  out.pass = intrinsics_ok && distortion_ok && rotation_error < 0.5 &&
             translation_error < 0.002 && out.pruned_outliers && uncertainty_ok &&
             held_validation.holdout_rms_px < 0.5 && export_ok;

  std::cout << std::fixed << std::setprecision(6)
            << "Synthetic self-test: " << (out.pass ? "PASS" : "FAIL") << "\n"
            << "  final stereo RMS: " << out.calib.stereo_rms << " px\n"
            << "  cam0 fx: " << out.calib.K0.at<double>(0, 0) << " (gt "
            << out.K0_gt.at<double>(0, 0) << ")\n"
            << "  cam0 fy: " << out.calib.K0.at<double>(1, 1) << " (gt "
            << out.K0_gt.at<double>(1, 1) << ")\n"
            << "  cam0 cx/cy: " << out.calib.K0.at<double>(0, 2) << "/"
            << out.calib.K0.at<double>(1, 2) << " (gt "
            << out.K0_gt.at<double>(0, 2) << "/" << out.K0_gt.at<double>(1, 2)
            << ")\n"
            << "  cam1 fx: " << out.calib.K1.at<double>(0, 0) << " (gt "
            << out.K1_gt.at<double>(0, 0) << ")\n"
            << "  cam1 fy: " << out.calib.K1.at<double>(1, 1) << " (gt "
            << out.K1_gt.at<double>(1, 1) << ")\n"
            << "  rotation error: " << rotation_error << " deg\n"
            << "  translation error: " << translation_error << " m\n"
            << "  holdout RMS: " << held_validation.holdout_rms_px << " px (7 independent views)\n"
            << "  covariance/correlation/export: " << (uncertainty_ok && export_ok ? "PASS" : "FAIL") << "\n"
            << "  outliers pruned: " << (out.pruned_outliers ? "yes" : "no")
            << "\n";
  if (!distortion_ok) {
    std::cout << "  distortion check failed\n";
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    cv::setNumThreads(2);
    Options opt = parseArgs(argc, argv);
    if (opt.self_test) {
      const SyntheticResult result = runSyntheticSelfTest();
      return result.pass ? 0 : 1;
    }

    if (opt.output_dir.empty()) {
      const fs::path session(opt.session_dir);
      const fs::path parent = session.parent_path();
      const fs::path workspace =
          parent.filename() == "data" ? parent.parent_path() : parent;
      opt.output_dir = (workspace / "results" / session.filename()).string();
    }
    for (const auto* artifact : {"report.yaml", "camchain.yaml", "cam0.yaml", "cam1.yaml"}) {
      if (fs::exists(fs::path(opt.output_dir) / artifact))
        fail("output already contains calibration artifacts; choose a new --output directory");
    }

    const Meta meta = loadMeta(fs::path(opt.session_dir) / "meta.yaml");
    std::vector<std::string> skipped;
    std::vector<View> views = loadViews(opt, meta, skipped);

    if (opt.verbose) {
      std::cout << "loaded " << views.size() << " usable views, "
                << skipped.size() << " skipped\n";
    }

    const cv::Size image_size(meta.image_width, meta.image_height);
    std::vector<View> training, held_out;
    const size_t holdout_count = views.empty() ? 0 :
        std::max<size_t>(1, static_cast<size_t>(views.size() * opt.holdout_fraction));
    for (size_t i = 0; i < views.size(); ++i) {
      (((i + 1) * holdout_count / views.size()) > (i * holdout_count / views.size())
          ? held_out : training).push_back(views[i]);
    }
    RunResult run = runCalibration(training, image_size, opt);
    Validation validation;
    validation.coverage_left = imageCoverage(run.active, image_size, true);
    validation.coverage_right = imageCoverage(run.active, image_size, false);
    validation.holdout_views = held_out.size();
    for (const auto& view : held_out) validation.holdout_names.push_back(view.name);
    if (run.converged) {
      run.stereo_uncertainty = spr_calib::estimateStereoUncertainty(run.active,
          run.final_calib.K0,run.final_calib.D0,run.final_calib.K1,run.final_calib.D1,
          run.final_calib.R,run.final_calib.T);
      validation = validate(run.active, run.final_calib, image_size);
      validation.coverage_left = imageCoverage(run.active, image_size, true);
      validation.coverage_right = imageCoverage(run.active, image_size, false);
      evaluateHoldout(held_out, run.final_calib, validation);
      if (views.size() < static_cast<size_t>(opt.min_pairs)) {
        run.converged = false;
        run.failure_reason = "insufficient usable input pairs (minimum " + std::to_string(opt.min_pairs) + ")";
      } else if (std::min(validation.coverage_left, validation.coverage_right) < opt.min_coverage) {
        run.converged = false;
        run.failure_reason = "final training corner coverage below threshold in one or both cameras";
      } else if (held_out.empty() || !std::isfinite(validation.holdout_rms_px) ||
                 validation.holdout_rms_px > opt.target_rms ||
                 validation.holdout_max_view_rms_px > opt.max_view_rms) {
        run.converged = false;
        run.failure_reason = "independent holdout reprojection error exceeds threshold";
      } else if (!uncertaintyAvailable(run.final_calib) && !opt.diagnostic) {
        run.converged = false;
        run.failure_reason = "intrinsic information is singular/poorly conditioned; local uncertainty is unavailable";
      } else if (!run.stereo_uncertainty.available && !opt.diagnostic) {
        run.converged = false;
        run.failure_reason = "full stereo uncertainty unavailable: " + run.stereo_uncertainty.reason;
      }
    }

    fs::create_directories(opt.output_dir);
    const fs::path output(opt.output_dir);
    writeTextReport(output / "report.txt", opt, meta, views.size(), skipped,
                    run, validation);
    writeYamlReport(output / "report.yaml", opt, meta, views.size(), skipped,
                    run, validation);
    writeExcluded(output / "excluded.txt", skipped, run);

    if (!run.converged) {
      for (const auto* name : {"camchain.yaml", "cam0.yaml", "cam1.yaml"})
        fs::remove(output / name);
    }
    if (opt.diagnostic) {
      std::cout << "diagnostic fit " << (run.converged ? "succeeded" : "FAILED")
                << ": NOT FORMAL; no calibration exports; reports in " << fs::absolute(output).string() << "\n";
      return run.converged ? 0 : 1;
    }
    if (run.converged) {
      writeCamchain(output / "camchain.yaml", meta, run.final_calib);
      writeMonoYaml(output / "cam0.yaml", run.final_calib.K0,
                    run.final_calib.D0, meta.image_width, meta.image_height,
                    run.final_calib.mono_rms0);
      writeMonoYaml(output / "cam1.yaml", run.final_calib.K1,
                    run.final_calib.D1, meta.image_width, meta.image_height,
                    run.final_calib.mono_rms1);
      std::cout << "calibration succeeded: stereo RMS "
                << run.final_calib.stereo_rms << " px, "
                << run.active.size() << " views\n"
                << "results written to " << fs::absolute(output).string()
                << "\n";
      return 0;
    }

    std::cout << "calibration FAILED: " << run.failure_reason << "\n"
              << "diagnostics written to " << fs::absolute(output).string()
              << "\n";
    return 1;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
