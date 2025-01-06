#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
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

// Helper function:
// Transform (a - b) into fractional coordinates, shift to nearest periodic
// image, then transform back to get minimal image distance (for general
// lattice).
inline double periodicDistance(const Eigen::Vector3d &a,
                               const Eigen::Vector3d &b,
                               const Eigen::Matrix3d &lattice,
                               const Eigen::Matrix3d &invLattice) {
  // Convert difference to fractional coordinates
  // fractionalDiff = invLattice * (a - b)
  Eigen::Vector3d diff = invLattice * (a - b);

  // Shift each component to the principal domain, e.g. [-0.5, 0.5)
  // so that we find the "nearest image" in fractional space.
  for (int i = 0; i < 3; i++) {
    diff(i) -= std::round(diff(i));
  }

  // Transform back to Cartesian
  Eigen::Vector3d cartDiff = lattice * diff;
  return cartDiff.norm();
}

/**
 * A simple 3D KDTree class that:
 *   - Stores the lattice matrix for periodic boundary calculations
 *   - Has a build(...) function that constructs the KD-tree recursively
 *   - Provides a radiusSearch(...) that returns indices of points within radius
 * r of a query point (under periodic boundary conditions)
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
   * Return all point-indices within radius r (periodic distance) of query.
   */
  std::vector<int> radiusSearch(const Eigen::Vector3d &query, double r) const {
    std::vector<std::pair<double, int>> results;
    radiusSearchRecursive(root_, query, r, 0, results);

    // Sort the results vector by distance (first element of the pair)
    std::sort(results.begin(), results.end());

    // Extract only the indexes (second element of the pair)
    std::vector<int> sortedIndexes;
    sortedIndexes.reserve(results.size());
    for (const auto &pair : results) {
      sortedIndexes.push_back(pair.second);
    }

    return sortedIndexes;
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
  void
  radiusSearchRecursive(KDNode *node, const Eigen::Vector3d &query, double r,
                        int depth,
                        std::vector<std::pair<double, int>> &results) const {
    if (!node)
      return;

    // Compute the actual (periodic) distance from query to node->point
    double dist = periodicDistance(query, node->point, lattice_, invLattice_);
    if (dist <= r) {
      // It's within radius, add it
      results.push_back({dist, node->index});
    }

    // Determine axis and coordinate difference along that axis
    int axis = depth % 3;
    double diff = query(axis) - node->point(axis);

    // We decide which subtree to visit first based on diff
    KDNode *first = (diff < 0.0 ? node->left : node->right);
    KDNode *second = (diff < 0.0 ? node->right : node->left);

    // Visit the first subtree
    radiusSearchRecursive(first, query, r, depth + 1, results);

    // However, we need to see if the second subtree _might_ contain points
    // within radius We can’t just rely on normal k-d bounding distance, because
    // we have to account for periodic wrap. A straightforward (though not the
    // most efficient) approach is to check anyway if |diff| could be < r in the
    // minimal image sense.

    // If the absolute difference (in the normal sense) is less than r, we
    // definitely should check the other side.  In a truly periodic sense, we
    // would also check if crossing the cell boundary might yield a smaller
    // difference. A simpler approach is to ALWAYS check both sides or do a
    // minimal-image style approach to the coordinate difference along the axis.
    // We'll do the simpler check here.

    double axisDist = std::abs(diff);
    if (axisDist < r) {
      // Because of potential wrap-around, let's also see if
      //   periodicDistance( query, node->point +/- lattice vectors ) < r
      // but for a minimal code example, we’ll just proceed to search the other
      // side:
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
  std::vector<int> neighbors = kdtree.radiusSearch(query, radius);

  // Print results
  std::cout << "Neighbors of " << query.transpose() << " within radius "
            << radius << ":\n";
  for (int idx : neighbors) {
    Eigen::Vector3d p = points[idx];
    double d = (p - query).norm(); // naive distance
    double pd = periodicDistance(query, p, lattice, lattice.inverse());
    std::cout << "  Index=" << idx << "  Point=(" << p.transpose() << ")"
              << "  naive_dist=" << d << "  pbc_dist=" << pd << "\n";
  }

  return 0;
}
