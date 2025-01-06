#include "consts.hh"
#include "quickselect.cc"
#include <vector>

struct Node {
  Number val;
  Node *left;
  Node *right;

  Node(const Number &val, Node *left, Node *right)
      : val{val}, left{left}, right{right} {}
};

class KdTree1D {
  Node *root;

public:
  KdTree1D(std::vector<Number> arr) {
    this->root = createNode(arr, 0, arr.size());
  }
  Node *createNode(std::vector<Number> arr, int start, int end) {
    // for leaf nodes, just return a new node with the one value
    if (end - start == 1) {
      return new Node{arr[start], nullptr, nullptr};
    }

    int n = end - start;

    Number medianX = quickselect(arr, 0, n - 1, n / 2);

    Node node;
    node.val = median(arr);
    if (start == end) {
      node.left = nullptr;
      node.right = nullptr;
    } else {
      int mid = (start + end) / 2;
      node.left = createNode(arr, start, mid);
      node.right = createNode(arr, mid + 1, end);
    }
    return node;
  }
};

int main() {
  std::vector<Number> arr = {3.2, 2.1, 1.5, 5.8, 4.6};
  KdTree1D tree(arr);
  return 0;
}