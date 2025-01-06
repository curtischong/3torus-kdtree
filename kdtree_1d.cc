#include "consts.hh"
#include "quickselect.cc"
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

struct Node {
  Number val;
  Node *left;
  Node *right;

  ~Node() {
    delete left;
    delete right;
  }
};

class KdTree1D {
  Node *root;

public:
  // Build a 1D kd-tree from arr
  KdTree1D(std::vector<Number> &arr) {
    root = createNode(arr, 0, (int)arr.size());
  }

  // Recursively build subtrees
  Node *createNode(std::vector<Number> &arr, int start, int end) {
    int n = end - start;
    if (n <= 0) {
      // no elements in [start, end)
      return nullptr;
    }

    Node *nd = new Node;
    nd->left = nullptr;
    nd->right = nullptr;

    if (n == 1) {
      // exactly one element
      nd->val = arr[start];
      return nd;
    }

    // medianIndex = start + floor(n/2)
    int medianIndex = start + (n / 2);

    // Use quickselect to place the median at index `medianIndex`
    int pivotIdx = quickselect(arr, start, end, medianIndex);

    // The pivot (median) is the node's value
    nd->val = arr[pivotIdx];

    // Left subtree in [start, pivotIdx)
    nd->left = createNode(arr, start, pivotIdx);

    // Right subtree in [pivotIdx+1, end)
    nd->right = createNode(arr, pivotIdx + 1, end);

    return nd;
  }

  // Range query:
  // - If lower <= upper: return values in [lower, upper].
  // - If lower > upper: "wrap around" scenario, meaning
  //   the domain is treated as circular, so we return
  //   values >= lower OR values <= upper.
  vector<Number> rangeSearch(Number lower, Number upper) {
    vector<Number> result;
    if (lower <= upper) {
      // Normal range: [lower, upper]
      rangeSearchHelper(root, lower, upper, result);
    } else {
      // Wrap-around range: values >= lower OR values <= upper
      // We'll treat that as two ranges:
      //   [lower, +∞) ∪ (-∞, upper]
      rangeSearchHelper(root, lower, numeric_limits<Number>::max(), result);
      rangeSearchHelper(root, numeric_limits<Number>::lowest(), upper, result);
    }
    return result;
  }

  ~KdTree1D() { delete root; }

private:
  // Recursive helper that finds values in [lo, hi].
  // Because this is a 1D KD-Tree, node->left contains
  // all values < node->val, and node->right contains
  // all values >= node->val. We can skip entire subtrees
  // if the node->val is outside [lo, hi].
  void rangeSearchHelper(Node *node, Number lo, Number hi,
                         vector<Number> &result) {
    if (!node)
      return;

    if (node->val > hi) {
      // node->val is above hi, so we only search the left side
      rangeSearchHelper(node->left, lo, hi, result);
    } else if (node->val < lo) {
      // node->val is below lo, so we only search the right side
      rangeSearchHelper(node->right, lo, hi, result);
    } else {
      // node->val is in [lo, hi]
      result.push_back(node->val);
      // We still check both sides, because there might be other
      // values in [lo, hi] in either subtree
      rangeSearchHelper(node->left, lo, hi, result);
      rangeSearchHelper(node->right, lo, hi, result);
    }
  }
};

int main() {
  // Example usage
  std::vector<Number> arr = {3.2, 2.1, 1.5, 5.8, 4.6};
  KdTree1D tree(arr);

  std::cout << "1D KD-Tree built successfully!\n";

  // Query #1: Normal range [2.0, 4.0]
  auto res1 = tree.rangeSearch(2.0, 4.0);
  std::cout << "Values in range [2.0, 4.0]:\n";
  for (auto &v : res1) {
    std::cout << "  " << v << "\n";
  }

  // Query #2: Wrap-around range [4.0, 2.0]
  // Interpreted as >= 4.0 or <= 2.0
  auto res2 = tree.rangeSearch(4.0, 2.0);
  std::cout << "Values in wrap-around range [4.0, 2.0]:\n";
  for (auto &v : res2) {
    std::cout << "  " << v << "\n";
  }

  return 0;
}