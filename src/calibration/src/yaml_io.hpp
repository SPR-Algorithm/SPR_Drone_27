#pragma once

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace spr_calib {

inline YAML::Node matrixYaml(const cv::Mat& input) {
  YAML::Node result(YAML::NodeType::Sequence);
  if (input.empty()) return result;
  cv::Mat matrix;
  input.convertTo(matrix, CV_64F);
  for (int r = 0; r < matrix.rows; ++r) {
    YAML::Node row(YAML::NodeType::Sequence);
    for (int c = 0; c < matrix.cols; ++c) row.push_back(matrix.at<double>(r, c));
    row.SetStyle(YAML::EmitterStyle::Flow);
    result.push_back(row);
  }
  return result;
}

inline cv::Mat yamlMatrix(const YAML::Node& node) {
  if (!node || !node.IsSequence() || node.size() == 0) return {};
  const int rows = static_cast<int>(node.size());
  const bool nested = node[0].IsSequence();
  const int cols = nested ? static_cast<int>(node[0].size()) : rows;
  cv::Mat matrix(nested ? rows : 1, cols, CV_64F);
  for (int r = 0; r < matrix.rows; ++r) {
    if (nested && static_cast<int>(node[r].size()) != cols)
      throw std::runtime_error("ragged YAML matrix");
    for (int c = 0; c < cols; ++c)
      matrix.at<double>(r, c) = (nested ? node[r][c] : node[c]).as<double>();
  }
  return matrix;
}

inline void writeYaml(const std::filesystem::path& path, const YAML::Node& node) {
  const auto temporary = path.string() + ".tmp";
  std::ofstream stream(temporary);
  if (!stream) throw std::runtime_error("cannot write " + temporary);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << node;
  if (!emitter.good()) throw std::runtime_error(emitter.GetLastError());
  stream << emitter.c_str() << '\n';
  stream.close();
  if (!stream) throw std::runtime_error("failed writing " + temporary);
  std::filesystem::rename(temporary, path);
}

}  // namespace spr_calib
