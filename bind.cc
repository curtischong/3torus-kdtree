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
                        double radius) {
  KDTree3D kdtree(coords, convertToEigenMatrix3d(lattice));
  NNGraph nnGraph;
  std::vector<int> neighbors =
      kdtree.radiusSearch(Eigen::Vector3d(0, 0, 0), radius);
  for (int i = 0; i < neighbors.size(); ++i) {
    nnGraph.src.push_back(i);
    nnGraph.dst.push_back(neighbors[i]);
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
}