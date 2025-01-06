#include "kdtree_3d.cc" // <-- Make sure this defines KDTree3D with the new radiusSearch(...) returning SearchResult
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
 * three NumPy arrays: 'src', 'dst', and 'disp' (the minimal-image
 * displacement for each edge).
 *
 * This version assumes:
 *   KDTree3D::radiusSearch(...) returns std::vector<SearchResult>,
 *   where SearchResult = { int index; double distance; Eigen::Vector3d
 * displacement; }.
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
  // 4. Perform radius-search & collect edges + displacements
  // -------------------------------
  // We'll store src and dst as 1D arrays (one entry per edge),
  // and disp will store the minimal-image displacement vectors as shape (M, 3).
  std::vector<double> src;
  std::vector<double> dst;
  std::vector<double> disp; // for (dx, dy, dz) per edge

  // Reserve approximate size
  src.reserve(N * max_neighbors);
  dst.reserve(N * max_neighbors);
  disp.reserve(N * max_neighbors * 3);

  for (int i = 0; i < N; ++i) {
    // 4a) Find neighbors within the radius, returning SearchResult
    std::vector<SearchResult> neighbors =
        kdtree.radiusSearch(coordsVec[i], radius);

    // 4b) Limit to max_neighbors if needed
    if (static_cast<int>(neighbors.size()) > max_neighbors) {
      neighbors.resize(max_neighbors);
    }

    // 4c) For each neighbor, record (i, nbr.index) and displacement
    for (const auto &nbr : neighbors) {
      if (exclude_self && nbr.index == i) {
        continue;
      }

      // (i -> neighbor)
      src.push_back(i);
      dst.push_back(nbr.index);

      // Minimal-image displacement from coordsVec[i] -> coordsVec[nbr.index]
      // is stored in nbr.displacement (by design).
      Eigen::Vector3d d = nbr.displacement;
      disp.push_back(d[0]);
      disp.push_back(d[1]);
      disp.push_back(d[2]);
    }
  }

  // -------------------------------
  // 5. Convert src, dst, disp -> py::array_t<double>
  // -------------------------------
  size_t M = src.size(); // number of edges

  // Create arrays of length M for src, dst, and shape (M,3) for disp
  py::array_t<double> py_src(M);
  py::array_t<double> py_dst(M);
  py::array_t<double> py_disp(M * 3);

  // Get pointers to these arrays
  auto buf_src = py_src.request();
  auto buf_dst = py_dst.request();
  auto buf_disp = py_disp.request();

  double *ptr_src = static_cast<double *>(buf_src.ptr);
  double *ptr_dst = static_cast<double *>(buf_dst.ptr);
  double *ptr_disp = static_cast<double *>(buf_disp.ptr);

  // Copy data
  for (size_t i = 0; i < M; ++i) {
    ptr_src[i] = src[i];
    ptr_dst[i] = dst[i];

    // disp is stored in triplets: (dx, dy, dz)
    ptr_disp[3 * i + 0] = disp[3 * i + 0];
    ptr_disp[3 * i + 1] = disp[3 * i + 1];
    ptr_disp[3 * i + 2] = disp[3 * i + 2];
  }

  // -------------------------------
  // 6. Build and return a Python dict
  // -------------------------------
  py::dict result;
  result["src"] = py_src;
  result["dst"] = py_dst;
  result["disp"] = py_disp; // shape (M,3)
  return result;
}

// -------------------------------
// Pybind11 Module Definition
// -------------------------------
PYBIND11_MODULE(kdtree_3torus, m) {
  m.doc() = "3-Torus KDTree with NumPy-based I/O via pybind11";

  // New function that uses NumPy arrays directly and returns {src, dst, disp}
  m.def("create_nn_graph_np", &create_nn_graph_np, py::arg("coords"),
        py::arg("lattice"), py::arg("radius"), py::arg("max_neighbors"),
        py::arg("exclude_self") = true,
        R"doc(
Create a nearest-neighbor graph from coords (Nx3) and lattice (3x3),
returning a dict with:
  "src": 1D array of source node indices
  "dst": 1D array of neighbor node indices
  "disp": 2D array (M,3) of minimal-image displacements
)doc");
}
