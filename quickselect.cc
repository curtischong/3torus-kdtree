#include "consts.hh"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

int partition(std::vector<Number> &arr, int left, int right) {
  // pivot is the last element in [left, right)
  Number pivot = arr[right - 1];
  int i = left - 1;

  for (int j = left; j < right - 1; ++j) {
    if (arr[j] <= pivot) {
      ++i;
      std::swap(arr[i], arr[j]);
    }
  }

  std::swap(arr[i + 1], arr[right - 1]);
  return i + 1;
}

int quickselect(std::vector<Number> &arr, int left, int right, int k) {
  // Base case: if the range has zero or one element, do not partition again.
  // If it has exactly one element, it's at index 'left'.
  if (right - left <= 1) {
    // If there's exactly one element, check if it's the kth
    return left;
  }

  int pivotIndex = partition(arr, left, right);

  if (pivotIndex == k) {
    return pivotIndex;
  } else if (pivotIndex > k) {
    // Search in the left side [left, pivotIndex)
    return quickselect(arr, left, pivotIndex, k);
  } else {
    // Search in the right side [pivotIndex+1, right)
    return quickselect(arr, pivotIndex + 1, right, k);
  }
}