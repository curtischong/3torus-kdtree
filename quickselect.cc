#include "consts.hh"
#include <iostream>
#include <vector>

// Partition function with exclusive end [left, right)
int partition(std::vector<Number> &arr, int left, int right) {
  // Choose last element as pivot
  Number pivot = arr[right - 1];
  int i = left - 1;

  // Move all <= pivot to the front
  for (int j = left; j < right - 1; ++j) {
    if (arr[j] <= pivot) {
      ++i;
      std::swap(arr[i], arr[j]);
    }
  }

  // Place pivot in the correct position
  std::swap(arr[i + 1], arr[right - 1]);
  return i + 1; // The final pivot index
}

// Quickselect: find the element that should end up at index k
int quickselect(std::vector<Number> &arr, int left, int right, int k) {
  // If the range has 0 or 1 elements, we are done
  if (right - left <= 1) {
    return left; // Only one element => that index is "correct"
  }

  // Partition and get the pivot index
  int pivotIndex = partition(arr, left, right);

  if (pivotIndex == k) {
    return pivotIndex;
  } else if (pivotIndex > k) {
    // Search in [left, pivotIndex)
    return quickselect(arr, left, pivotIndex, k);
  } else {
    // Search in [pivotIndex+1, right)
    return quickselect(arr, pivotIndex + 1, right, k);
  }
}