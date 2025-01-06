#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

// A small struct to hold a node in the KD-Tree
struct KDNode {
  Eigen::Vector3d point; // The 3D point
  int index;             // Index in the original input
  KDNode *left;          // Left child
  KDNode *right;         // Right child

  KDNode(const Eigen::Vector3d &pt, int idx)
      : point(pt), index(idx), left(nullptr), right(nullptr) {}
};

/**
 * Helper function:
 *   Returns the minimal-image displacement from b to a under PBC, i.e.
 *   "a - b" adjusted for periodic boundary conditions.  In other words:
 *     fractionalDiff = invLattice * (a - b),
 *     shift fractionalDiff by rounding each component to the nearest integer,
 *     then transform back to Cartesian.
 */
inline Eigen::Vector3d periodicDisplacement(const Eigen::Vector3d &a,
                                            const Eigen::Vector3d &b,
                                            const Eigen::Matrix3d &lattice,
                                            const Eigen::Matrix3d &invLattice) {
  // Convert difference to fractional coordinates
  Eigen::Vector3d diffFrac = invLattice * (a - b);

  // Shift each component to nearest image
  for (int i = 0; i < 3; i++) {
    diffFrac(i) -= std::round(diffFrac(i));
  }

  // Transform back to Cartesian
  return lattice * diffFrac;
}

// For convenience, we still keep the original "distance" helper,
// but we can rely on periodicDisplacement(...) above
inline double periodicDistance(const Eigen::Vector3d &a,
                               const Eigen::Vector3d &b,
                               const Eigen::Matrix3d &lattice,
                               const Eigen::Matrix3d &invLattice) {
  Eigen::Vector3d disp = periodicDisplacement(a, b, lattice, invLattice);
  return disp.norm();
}

/**
 * Structure to hold search results.
 *   index        = index of the neighbor
 *   distance     = minimal-image distance to the query
 *   displacement = minimal-image displacement vector (query -> point)
 */
struct SearchResult {
  int index;
  double distance;
  Eigen::Vector3d displacement;
};

/**
 * A simple 3D KDTree class that:
 *   - Stores the lattice matrix for periodic boundary calculations
 *   - Has a build(...) function that constructs the KD-tree recursively
 *   - Provides a radiusSearch(...) that returns points within radius r
 *     (under periodic boundary conditions), along with distances and
 *     displacements.
 */
class KDTree3D {
public:
  // Constructor: build tree from points and a user-provided lattice matrix
  KDTree3D(const std::vector<Eigen::Vector3d> &points,
           const Eigen::Matrix3d &lattice)
      : root_(nullptr), points_(points), lattice_(lattice) {
    // Precompute inverse lattice for fast minimal-image distance
    invLattice_ = lattice_.inverse();

    // Build the tree using all point indices
    std::vector<int> indices(points_.size());
    for (size_t i = 0; i < points_.size(); i++) {
      indices[i] = static_cast<int>(i);
    }

    root_ = build(indices, 0); // Build with splitting dimension = 0
  }

  ~KDTree3D() {
    // Recursively delete nodes
    deleteSubtree(root_);
  }

  /**
   * Return all points within radius r (periodic distance) of query,
   * along with their minimal-image distances and displacements.
   */
  std::vector<SearchResult> radiusSearch(const Eigen::Vector3d &query,
                                         double r) const {
    std::vector<SearchResult> results;
    radiusSearchRecursive(root_, query, r, 0, results);

    // Sort the results vector by distance
    std::sort(results.begin(), results.end(),
              [](const SearchResult &a, const SearchResult &b) {
                return a.distance < b.distance;
              });

    return results;
  }

private:
  KDNode *root_;
  std::vector<Eigen::Vector3d> points_;
  Eigen::Matrix3d lattice_;
  Eigen::Matrix3d invLattice_;

  // Recursively build the k-d tree
  KDNode *build(std::vector<int> &indices, int depth) {
    if (indices.empty())
      return nullptr;

    // Splitting dimension: 0 -> x, 1 -> y, 2 -> z, repeating
    int axis = depth % 3;

    // Sort indices by the chosen axis coordinate
    std::sort(indices.begin(), indices.end(), [&](int lhs, int rhs) {
      return points_[lhs](axis) < points_[rhs](axis);
    });

    // Pick median as the pivot
    size_t median = indices.size() / 2;
    int midIndex = indices[median];

    KDNode *node = new KDNode(points_[midIndex], midIndex);

    // Split into left and right subsets
    std::vector<int> leftIndices(indices.begin(), indices.begin() + median);
    std::vector<int> rightIndices(indices.begin() + median + 1, indices.end());

    // Recursively build subtrees
    node->left = build(leftIndices, depth + 1);
    node->right = build(rightIndices, depth + 1);

    return node;
  }

  // Recursively free nodes
  void deleteSubtree(KDNode *node) {
    if (!node)
      return;
    deleteSubtree(node->left);
    deleteSubtree(node->right);
    delete node;
  }

  // Recursive radius search under periodic boundary conditions
  void radiusSearchRecursive(KDNode *node, const Eigen::Vector3d &query,
                             double r, int depth,
                             std::vector<SearchResult> &results) const {
    if (!node)
      return;

    // Compute minimal-image displacement from query -> node->point
    Eigen::Vector3d disp =
        periodicDisplacement(node->point, query, lattice_, invLattice_);
    double dist = disp.norm();

    // If it's within radius, add it
    if (dist <= r) {
      SearchResult sr;
      sr.index = node->index;
      sr.distance = dist;
      sr.displacement = disp;
      results.push_back(sr);
    }

    // Determine axis and coordinate difference along that axis (direct in
    // Cartesian).
    int axis = depth % 3;
    double diff = query(axis) - node->point(axis);

    // Decide which subtree to visit first based on diff
    KDNode *first = (diff < 0.0 ? node->left : node->right);
    KDNode *second = (diff < 0.0 ? node->right : node->left);

    // Visit the first subtree
    radiusSearchRecursive(first, query, r, depth + 1, results);

    // Because of periodic boundaries, a simple axis check can be insufficient,
    // but at minimum, if std::abs(diff) < r, we should also visit the second.
    if (std::abs(diff) < r) {
      radiusSearchRecursive(second, query, r, depth + 1, results);
    }
  }
};

// -------------------------------------------------------------
// A small example main() to demonstrate usage.
int main() {
  // Example: Suppose we have a cubic lattice of size 10x10x10 for simplicity.
  // In that case, the lattice matrix is just 10 * Identity.
  Eigen::Matrix3d lattice = 10.0 * Eigen::Matrix3d::Identity();

  // Some random points in [0, 10)^3
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));
  points.push_back(Eigen::Vector3d(9.5, 9.5, 0.1));
  points.push_back(Eigen::Vector3d(5.0, 5.0, 5.0));
  points.push_back(Eigen::Vector3d(0.2, 9.9, 9.8));
  points.push_back(Eigen::Vector3d(8.9, 2.2, 9.9));
  // ... add as many as you like

  // Build KD-tree
  KDTree3D kdtree(points, lattice);

  // Pick a query point
  Eigen::Vector3d query(0.0, 0.0, 0.0);
  double radius = 3.0;

  // Search for neighbors within radius = 3.0 (under PBC)
  std::vector<SearchResult> neighbors = kdtree.radiusSearch(query, radius);

  // Print results
  std::cout << "Neighbors of " << query.transpose() << " within radius "
            << radius << ":\n";
  for (const auto &res : neighbors) {
    const Eigen::Vector3d &p = points[res.index];
    std::cout << "  Index=" << res.index << "  Point=(" << p.transpose() << ")"
              << "  distance=" << res.distance << "  disp=("
              << res.displacement.transpose() << ")\n";
  }

  return 0;
}
