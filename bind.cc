#include "kdtree_3d.cc"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

namespace py = pybind11;

struct NNGraph {
  std::vector<double> src;
  std::vector<double> dst;
};

Eigen::Matrix3d
convertToEigenMatrix3d(const std::vector<std::vector<double>> &vec) {
  Eigen::Matrix3d matrix;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix(i, j) = vec[i][j];
    }
  }
  return matrix;
}

NNGraph create_nn_graph(const std::vector<Eigen::Vector3d> &coords,
                        const std::vector<std::vector<double>> &lattice,
                        double radius, int max_neighbors, bool exclude_self) {
  // Build the KDTree
  KDTree3D kdtree(coords, convertToEigenMatrix3d(lattice));
  NNGraph nnGraph;
  nnGraph.src.reserve(coords.size() * max_neighbors);
  nnGraph.dst.reserve(coords.size() * max_neighbors);

  // For each point, find neighbors within the given radius
  for (int i = 0; i < static_cast<int>(coords.size()); ++i) {
    // Find all neighbors within 'radius' of coords[i]
    std::vector<int> neighbors = kdtree.radiusSearch(coords[i], radius);

    // Limit the number of neighbors to max_neighbors (if needed)
    if (static_cast<int>(neighbors.size()) > max_neighbors) {
      neighbors.resize(max_neighbors);
    }

    // Insert edges into the graph
    for (int nbrIdx : neighbors) {
      // Optionally exclude self-edges if desired:
      if (exclude_self && nbrIdx == i)
        continue;
      nnGraph.src.push_back(i);
      nnGraph.dst.push_back(nbrIdx);
    }
  }

  return nnGraph;
}

PYBIND11_MODULE(3torus_kdtree, m) {
  py::class_<NNGraph>(m, "NNGraph")
      .def(py::init<>())
      .def_readwrite("src", &NNGraph::src)
      .def_readwrite("dst", &NNGraph::dst);

  m.def("create_nn_graph", &create_nn_graph,
        "Creates the nearest neighbor graph");
}