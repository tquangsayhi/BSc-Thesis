// 4x4 Matrices mapped to the .data section
int matA[4][4] = {
    {1, 2, 3, 4},
    {1, 2, 3, 4},
    {1, 2, 3, 4},
    {1, 2, 3, 4}
};

int matB[4][4] = {
    {9, 8, 7, 6},
    {9, 8, 7, 6},
    {9, 8, 7, 6},
    {9, 8, 7, 6}
};

int matC[4][4]; // Result matrix

int main() {
    int sum_check = 0;

    // Matrix Addition: matC = matA + matB
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            matC[i][j] = matA[i][j] + matB[i][j];
            sum_check += matC[i][j]; 
        }
    }
    
    // Write the final verification sum to the MMIO address to stop Vivado
    volatile int* mmio_done = (int*)0x40000000;
    *mmio_done = sum_check; 
    
    return sum_check; 
}