void swap(int* a, int* b) {
    int t = *a;
    *a = *b;
    *b = t;
}

int partition(int arr[], int low, int high) {
    int pivot = arr[high];
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++) {
        if (arr[j] < pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

void quicksort(int arr[], int low, int high) {
    if (low < high) {
        int pi = partition(arr, low, high);
        quicksort(arr, low, pi - 1);
        quicksort(arr, pi + 1, high);
    }
}

// Global array ensures it is mapped to the .data section
int data[10] = {87, 34, 12, 56, 98, 23, 45, 67, 9, 77};

int main() {
    // Execute the sorting benchmark
    quicksort(data, 0, 9);
    
    // Verify the sort: The lowest value (9) + the highest value (98) = 107
    int sum_check = data[0] + data[9];
    
    // Write to the MMIO address to signal the Vivado testbench to stop
    volatile int* mmio_done = (int*)0x40000000;
    *mmio_done = sum_check; 
    
    return sum_check; 
}