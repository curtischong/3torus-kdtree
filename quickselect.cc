#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

// Define a constant to switch between float and double
#define USE_DOUBLE true

#if USE_DOUBLE
using Number = double;
#else
using Number = float;
#endif

// Partition function
int partition(std::vector<Number>& arr, int left, int right) {
    Number pivot = arr[right]; // Choose the last element as the pivot
    int i = left - 1;          // Index for the smaller element

    for (int j = left; j < right; ++j) {
        if (arr[j] <= pivot) {
            ++i;
            std::swap(arr[i], arr[j]); // Swap smaller element with arr[i]
        }
    }
    std::swap(arr[i + 1], arr[right]); // Place pivot in the correct position
    return i + 1;
}

// Quickselect function
Number quickselect(std::vector<Number>& arr, int left, int right, int k) {
    if (left <= right) {
        int pivotIndex = partition(arr, left, right);

        if (pivotIndex == k) {
            return arr[pivotIndex];
        } else if (pivotIndex > k) {
            return quickselect(arr, left, pivotIndex - 1, k);
        } else {
            return quickselect(arr, pivotIndex + 1, right, k);
        }
    }
    return -1.0; // This case should never occur
}

Number median(std::vector<Number>& arr) {
    int n = arr.size();
    return quickselect(arr, 0, n - 1, n / 2);
}

int main() {
    std::srand(std::time(nullptr));
    std::vector<Number> arr = {3.2, 2.1, 1.5, 5.8, 4.6};

    int k = 2; // Find the 2nd smallest element (0-based index)
    int n = arr.size();
    if (k < 0 || k >= n) {
        std::cout << "Invalid value of k\n";
        return 1;
    }

    Number result = quickselect(arr, 0, n - 1, k);
    std::cout << "The " << k + 1 << "th smallest element is " << result << "\n";

    return 0;
}