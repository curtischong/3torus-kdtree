#include "kdtree_3d.cc"
#include <Eigen/Dense>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

namespace py = pybind11;

/**
 * Convert a 3x3 NumPy array to an Eigen::Matrix3d.
 */
Eigen::Matrix3d convertToEigenMatrix3d(py::array_t<double> lattice) {
  // Get buffer info
  py::buffer_info buf = lattice.request();

  // Check that lattice is 2D, shape = (3,3)
  if (buf.ndim != 2 || buf.shape[0] != 3 || buf.shape[1] != 3) {
    throw std::runtime_error("Lattice must be a 3x3 array.");
  }

  // Pointer to data
  double *ptr = static_cast<double *>(buf.ptr);

  // Copy into Eigen::Matrix3d
  Eigen::Matrix3d mat;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat(i, j) = ptr[i * 3 + j];
    }
  }
  return mat;
}

/**
 * A new function that takes NumPy arrays for coords (Nx3) & lattice (3x3),
 * plus radius, max_neighbors, exclude_self, and returns a dictionary of
 * two 1D NumPy arrays: 'src' and 'dst'.
 */
py::dict create_nn_graph_np(py::array_t<double> coords,
                            py::array_t<double> lattice, double radius,
                            int max_neighbors, bool exclude_self) {
  // -------------------------------
  // 1. Parse the coords array
  // -------------------------------
  auto buf_coords = coords.request();
  if (buf_coords.ndim != 2 || buf_coords.shape[1] != 3) {
    throw std::runtime_error("Coordinates must be of shape (N, 3).");
  }
  int N = static_cast<int>(buf_coords.shape[0]);
  double *ptr_coords = static_cast<double *>(buf_coords.ptr);

  // Convert to std::vector<Eigen::Vector3d>
  std::vector<Eigen::Vector3d> coordsVec;
  coordsVec.reserve(N);

  for (int i = 0; i < N; ++i) {
    double x = ptr_coords[i * 3 + 0];
    double y = ptr_coords[i * 3 + 1];
    double z = ptr_coords[i * 3 + 2];
    coordsVec.push_back(Eigen::Vector3d(x, y, z));
  }

  // -------------------------------
  // 2. Parse the lattice array (3x3)
  // -------------------------------
  Eigen::Matrix3d lat = convertToEigenMatrix3d(lattice);

  // -------------------------------
  // 3. Build the KDTree
  // -------------------------------
  KDTree3D kdtree(coordsVec, lat);

  // -------------------------------
  // 4. Perform radius-search & collect edges
  // -------------------------------
  std::vector<double> src;
  std::vector<double> dst;
  src.reserve(N * max_neighbors);
  dst.reserve(N * max_neighbors);

  for (int i = 0; i < N; ++i) {
    // Find neighbors within the radius
    std::vector<int> neighbors = kdtree.radiusSearch(coordsVec[i], radius);

    // Limit to max_neighbors if needed
    if (static_cast<int>(neighbors.size()) > max_neighbors) {
      neighbors.resize(max_neighbors);
    }

    // Insert edges into src/dst
    for (int nbr : neighbors) {
      if (exclude_self && nbr == i) {
        continue;
      }
      src.push_back(i);
      dst.push_back(nbr);
    }
  }

  // -------------------------------
  // 5. Convert src/dst to py::array_t<double>
  // -------------------------------
  py::array_t<double> py_src(src.size());
  py::array_t<double> py_dst(dst.size());

  // Get pointers to the newly allocated arrays
  auto buf_src = py_src.request();
  auto buf_dst = py_dst.request();

  double *ptr_src = static_cast<double *>(buf_src.ptr);
  double *ptr_dst = static_cast<double *>(buf_dst.ptr);

  // Copy data
  for (size_t i = 0; i < src.size(); ++i) {
    ptr_src[i] = src[i];
    ptr_dst[i] = dst[i];
  }

  // Build a Python dict with the results
  py::dict result;
  result["src"] = py_src;
  result["dst"] = py_dst;
  return result;
}

// -------------------------------
// 6. Pybind11 Module Definition
// -------------------------------
PYBIND11_MODULE(3torus_kdtree, m) {
  m.doc() = "3-Torus KDTree with NumPy-based I/O via pybind11";

  // If you still need your old NNGraph struct, you can keep it/bind it here.
  // For a fully NumPy-based approach, you can skip the struct/class.

  // New function that uses NumPy arrays directly
  m.def("create_nn_graph_np", &create_nn_graph_np, py::arg("coords"),
        py::arg("lattice"), py::arg("radius"), py::arg("max_neighbors"),
        py::arg("exclude_self") = true,
        R"doc(
Create nearest neighbor graph from coords (Nx3 array) and lattice (3x3 array),
returning a dict of NumPy arrays {"src": ..., "dst": ...} that describe edges.
)doc");
}
