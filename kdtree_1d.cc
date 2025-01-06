#include "consts.hh"
#include "quickselect.cc"
#include <vector>

struct Node {
  Number val;
  Node *left;
  Node *right;

  Node(const Number &val, Node *left, Node *right)
      : val{val}, left{left}, right{right} {}

  // destructor
  ~Node() {
    delete left;
    delete right;
  }
};

class KdTree1D {
  Node *root;

public:
  KdTree1D(std::vector<Number> arr) {
    this->root = createNode(arr, 0, arr.size());
  }

  Node *createNode(std::vector<Number> arr, int start, int end) {
    // for leaf nodes, just return a new node with the one value
    Node *nd = new Node(arr[start], nullptr, nullptr);
    if (end - start == 1) {
      return nd;
    }

    int n = end - start;

    // note that this is floor(n/2)
    int pivotIdx = quickselect(arr, 0, n - 1, n / 2);
    Number median = arr[pivotIdx];

    nd->val = median;
    nd->left = createNode(arr, start, pivotIdx);
    nd->right = createNode(arr, pivotIdx + 1, end);

    return nd;
  }

  // destructor
  ~KdTree1D() { delete root; }
};

int main() {
  std::vector<Number> arr = {3.2, 2.1, 1.5, 5.8, 4.6};
  KdTree1D tree(arr);
  std::cout << "1d tree" << std::endl;
  return 0;
}