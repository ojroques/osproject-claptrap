#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "image.h"
#include "const.h"
#include "client.h"

uint8_t IMAGE[IMG_HEIGHT][IMG_WIDTH];
const char *ERR_OUT_OF_BOUNDS = "Index out of bounds exception";

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

uint8_t get_cell(uint16_t i, uint16_t j) {
    if (i >= IMG_HEIGHT || j >= IMG_WIDTH) {
        printf("%s\n", ERR_OUT_OF_BOUNDS);
    }
    return IMAGE[i][j];
}

void set_cell(uint16_t i, uint16_t j, uint8_t value) {
    if (i >= IMG_HEIGHT || j >= IMG_WIDTH) {
        printf("%s\n", ERR_OUT_OF_BOUNDS);
    }
    IMAGE[i][j] = value;
}

int main() {
    init_image();
    print_image();
    set_cell(8, 12, 1);
    print_image();
    printf("La case (8, 12) a pour valeur %d.\n", get_cell(8, 12));
    return 0;
}
