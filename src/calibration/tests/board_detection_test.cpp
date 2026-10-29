#include "board.hpp"
#include "yaml_io.hpp"

#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

cv::Mat renderBoard() {
  // 8x11 INNER corners require 9x12 printed squares. The white margin is
  // intentional: edge-touching boards are not a valid detector fixture.
  cv::Mat image(480, 848, CV_8UC1, cv::Scalar(235));
  constexpr int edge = 30;
  for (int row = 0; row < 12; ++row)
    for (int col = 0; col < 9; ++col)
      cv::rectangle(image, cv::Rect(289 + col * edge, 60 + row * edge, edge, edge),
                    cv::Scalar((row + col) % 2 ? 250 : 5), cv::FILLED);
  return image;
}
}  // namespace

int main() {
  try {
    cv::setNumThreads(1);
    spr_calib::BoardConfig config;
    std::vector<cv::Point2f> corners;
    cv::Size pattern;
    const cv::Mat board = renderBoard();
    require(spr_calib::detectBoard(board, config, corners, pattern),
            "8x11 rendered checkerboard was not detected");
    require(corners.size() == 88, "wrong inner-corner count");
    require(spr_calib::boardInsideImage(corners, board.size(), 15), "board margin");
    require(spr_calib::laplacianVariance(board, corners) > 100, "sharpness gate");

    cv::Mat rotated;
    cv::rotate(board, rotated, cv::ROTATE_90_CLOCKWISE);
    require(spr_calib::detectBoard(rotated, config, corners, pattern),
            "rotated checkerboard was not detected");
    require(corners.size() == 88, "rotated corner count");

    const std::vector<cv::Point2f> source{{0,0}, {847,0}, {847,479}, {0,479}};
    const std::vector<cv::Point2f> destination{{25,18}, {815,42}, {835,455}, {14,470}};
    cv::Mat perspective;
    cv::warpPerspective(board, perspective, cv::getPerspectiveTransform(source, destination),
                        board.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(235));
    require(spr_calib::detectBoard(perspective, config, corners, pattern),
            "perspective checkerboard was not detected");
    cv::GaussianBlur(perspective, perspective, cv::Size(3,3), 0.7);
    require(spr_calib::detectBoard(perspective, config, corners, pattern),
            "slightly blurred checkerboard was not detected");
    cv::Mat blank(board.size(), CV_8UC1, cv::Scalar(127));
    require(!spr_calib::detectBoard(blank, config, corners, pattern),
            "blank image produced a false board");

    const auto points = spr_calib::makeObjectPoints(8, 11, 0.025);
    require(points.size() == 88 && std::abs(points[1].x - 0.025) < 1e-7 &&
            std::abs(points.back().x - 0.175) < 1e-7 &&
            std::abs(points.back().y - 0.250) < 1e-7,
            "25mm board metric coordinates are incorrect");
    const cv::Mat matrix = (cv::Mat_<double>(2,2) << 1.25, -0.5, 3.0, 4.0);
    const auto encoded = YAML::Dump(spr_calib::matrixYaml(matrix));
    require(encoded.find("opencv-matrix") == std::string::npos &&
            cv::norm(matrix - spr_calib::yamlMatrix(YAML::Load(encoded))) < 1e-12,
            "portable YAML matrix roundtrip failed");
    std::cout << "PASS: rendered 8x11 checkerboard, rotations, perspective, blur, negative image, 25mm scale, YAML\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
