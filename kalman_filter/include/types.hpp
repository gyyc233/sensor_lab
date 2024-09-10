#ifndef OPENKF_TYPES_H
#define OPENKF_TYPES_H

#include <Eigen/Dense>
#include <stdint.h>

namespace kf {
using float32_t = float;

template <int32_t ROW, int32_t COL>
using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

template <int32_t ROW> using Vector = Eigen::Matrix<float32_t, ROW, 1>;
} // namespace kf

#endif // OPENKF_TYPES_H
