#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "image.h"
#include "const.h"

u_int8_t IMAGE[IMG_HEIGHT][IMG_WIDTH];

/* For debugging purposes only */
void print_image() {
    int i, j;
    printf("\n");
    for (i = 0; i < IMG_HEIGHT; i++) {
        for (j = 0; j < IMG_WIDTH; j++) {
            printf("%d  ", IMAGE[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void init_image() {
    int i, j;
    for (i = 0; i < IMG_HEIGHT; i++) {
        for (j = 0; j < IMG_WIDTH; j++) {
            IMAGE[i][j] = 0;
        }
    }
}

u_int8_t get_cell(int32_t i, int32_t j) {
    if (i >= IMG_HEIGHT || j >= IMG_WIDTH) {
        return ERR_OUT_OF_BOUNDS;
    }
    return IMAGE[i][j];
}

int set_cell(int32_t i, int32_t j, u_int8_t value) {
    if (i >= IMG_HEIGHT || j >= IMG_WIDTH) {
        return ERR_OUT_OF_BOUNDS;
    }
    IMAGE[i][j] = value;
    return 0;
}

int main() {
    init_image();
    print_image();
    set_cell(8, 12, 1);
    print_image();
    printf("La case (8, 12) a pour valeur %d.\n", get_cell(8, 12));
    return 0;
}
