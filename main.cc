#include "consts.hh"
#include <array>
#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
  int n;
  std::cin >> n;

  // Use std::array<int, 3> instead of int[3]
  std::vector<std::array<Number, 3>> arr(n);

  for (int i = 0; i < n; i++) {
    std::cin >> arr[i][0] >> arr[i][1] >> arr[i][2];
  }

  for (const auto &item : arr) {
    std::cout << item[0] << " " << item[1] << " " << item[2] << std::endl;
  }
}
