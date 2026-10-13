int data[10] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

int main() {
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += data[i];
    }
    
    // Send the actual sum out to the hardware!
    volatile int* mmio_done = (int*)0x40000000;
    *mmio_done = sum; 
    
    return sum; 
}