#ifndef _TYPRS_HPP
#define _TYPRS_HPP
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ostream>
#include <vector>

using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using quaternion_t = Eigen::Quaternion<scalar_t>;
using vector_array_t = std::vector<vector_t>;
using vector_array2_t = std::vector<vector_array_t>;
#endif // _TYPRS_HPP