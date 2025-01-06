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
  KdTree1D(std::vector<Number> arr) {
    this->root = createNode(arr, 0, arr.size());
  }

  Node *createNode(std::vector<Number> arr, int start, int end) {
    cout << start << " " << end << endl;
    // for leaf nodes, just return a new node with the one value
    Node *nd = new Node;
    int n = end - start;
    if (n == 1) {
      nd->val = arr[start];
      return nd;
    }

    // note that this is floor(n/2)
    int pivotIdx = quickselect(arr, start, end, n / 2);
    Number median = arr[pivotIdx];

    nd->left = createNode(arr, start, pivotIdx);
    nd->right = createNode(arr, pivotIdx, end);

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