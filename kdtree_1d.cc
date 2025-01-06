#include "consts.hh"
#include "quickselect.cc"
#include <vector>

using namespace std;

struct Node {
  Number val;
  Node *left;
  Node *right;

  //   Node(const Number &val, Node *left, Node *right)
  //       : val{val}, left{left}, right{right} {}

  // destructor
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
    // We pass the original array by reference
    // to avoid multiple copies.
    root = createNode(arr, 0, (int)arr.size());
  }

  // Recursively build subtrees
  Node *createNode(std::vector<Number> &arr, int start, int end) {
    int n = end - start;
    if (n <= 0) {
      // no elements in [start, end)
      return nullptr;
    }

    // Create the node
    Node *nd = new Node;
    nd->left = nullptr;
    nd->right = nullptr;

    if (n == 1) {
      // exactly one element
      nd->val = arr[start];
      return nd;
    }

    // mid = start + floor(n/2)
    int medianIndex = start + (n / 2);

    // Use quickselect to place the median at index `medianIndex`
    int pivotIdx = quickselect(arr, start, end, medianIndex);

    // The pivot is the actual median
    nd->val = arr[pivotIdx];

    // Build left subtree from [start, pivotIdx)
    nd->left = createNode(arr, start, pivotIdx);

    // Build right subtree from [pivotIdx+1, end)
    nd->right = createNode(arr, pivotIdx + 1, end);

    return nd;
  }

  ~KdTree1D() { delete root; }
};

int main() {
  // Example usage
  std::vector<Number> arr = {3.2, 2.1, 1.5, 5.8, 4.6};

  KdTree1D tree(arr);

  std::cout << "1D KD-Tree built successfully!" << std::endl;
  return 0;
}