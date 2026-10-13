typedef unsigned int uint32_t;

// 1. MMIO Hardware Pointers
#define MMIO_LOWER (*(volatile uint32_t*)0x40000000)
#define MMIO_UPPER (*(volatile uint32_t*)0x40000004)

// 2. Bypass GCC's Linker! Hardcode the grids into known-safe Data Memory.
#define GRID      ((volatile uint32_t (*)[8])0x00001000)
#define NEXT_GRID ((volatile uint32_t (*)[8])0x00001100)

// 3. Force inline execution: This completely disables Stack Memory usage.
static inline void __attribute__((always_inline)) delay(uint32_t count) {
    for (volatile uint32_t i = 0; i < count; i++);
}

static inline void __attribute__((always_inline)) init_board() {
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            GRID[r][c] = 0;
        }
    }
    // Glider pattern
    GRID[1][2] = 1;
    GRID[2][3] = 1;
    GRID[3][1] = 1;
    GRID[3][2] = 1;
    GRID[3][3] = 1;
}

int main() {
    uint32_t neighbours;
    
    init_board();

    while (1) {
        
        // STEP A: Push to MMIO
        uint32_t lower_data = 0;
        uint32_t upper_data = 0;

        for (int r = 0; r < 4; r++) {
            uint32_t row_byte = 0;
            for (int c = 0; c < 8; c++) {
                row_byte |= (GRID[r][c] << c);
            }
            lower_data |= (row_byte << (r << 3)); 
        }

        for (int r = 4; r < 8; r++) {
            uint32_t row_byte = 0; 
            for (int c = 0; c < 8; c++) {
                row_byte |= (GRID[r][c] << c);
            }
            upper_data |= (row_byte << ((r - 4) << 3)); 
        }

        MMIO_LOWER = lower_data;
        MMIO_UPPER = upper_data;

        // STEP B: Calculate the Next Generation
        for (int r = 0; r < 8; r++) {
            for (int c = 0; c < 8; c++) {
                
                neighbours = 
                    GRID[(r+7)&7][(c+7)&7] + GRID[(r+7)&7][ c ] + GRID[(r+7)&7][(c+1)&7] +
                    GRID[ r ][(c+7)&7]     +                      GRID[ r ][(c+1)&7] +
                    GRID[(r+1)&7][(c+7)&7] + GRID[(r+1)&7][ c ] + GRID[(r+1)&7][(c+1)&7];

                if (neighbours == 2) {
                    NEXT_GRID[r][c] = GRID[r][c];
                } else if (neighbours == 3) {
                    NEXT_GRID[r][c] = 1;         
                } else {
                    NEXT_GRID[r][c] = 0;         
                }
            }
        }

        // STEP C: Commit the Generation
        for (int r = 0; r < 8; r++) {
            for (int c = 0; c < 8; c++) {
                GRID[r][c] = NEXT_GRID[r][c];
            }
        }

        delay(1500000); 
    }
    
    return 0;
}
