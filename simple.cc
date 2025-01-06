#include "kdtree_3d.cc"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

namespace py = pybind11;

/**
 * A new function that takes NumPy arrays for coords (Nx3) & lattice (3x3),
 * plus radius, max_neighbors, exclude_self, and returns a dictionary of
 * two 1D NumPy arrays: 'src' and 'dst'.
 */
py::dict create_nn_graph_np(int max_neighbors, bool exclude_self) {
  // Build a Python dict with the results
  py::dict result;
  result["src"] = 0;
  result["dst"] = 2;
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
  m.def("create_nn_graph_np", &create_nn_graph_np, py::arg("max_neighbors"),
        py::arg("exclude_self") = true,
        R"doc(
Create nearest neighbor graph from coords (Nx3 array) and lattice (3x3 array),
returning a dict of NumPy arrays {"src": ..., "dst": ...} that describe edges.
)doc");
}
