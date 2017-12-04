/* Written by Olivier Roques for the OS project.
Eurecom, 2017 - 2018. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "image.h"
#include "const.h"
#include "client.h"

int IMAGE[IMG_HEIGHT][IMG_WIDTH];
const char *STR_OUT_OF_BOUNDS = "Index out of bounds exception";

const color_t WHITE = {255, 255, 255};
const color_t BLACK = {0, 0, 0};
const color_t RED = {255, 0, 0};
const color_t GREEN = {0, 255, 0};
const color_t BLUE = {0, 0, 255};

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

int get_cell(uint16_t i, uint16_t j) {
    if (is_out_of_bounds(i, j)) {
        printf("%s\n", STR_OUT_OF_BOUNDS);
        return ERR_OUT_OF_BOUNDS;
    }
    return IMAGE[i][j];
}

int set_cell(uint16_t i, uint16_t j, int value) {
    if (is_out_of_bounds(i, j)) {
        printf("%s\n", STR_OUT_OF_BOUNDS);
        return ERR_OUT_OF_BOUNDS;
    }
    IMAGE[i][j] = value;
    return 0;
}

uint16_t coord_to_index(int16_t c) {
    uint16_t i = (uint16_t) c;
    return (i - (i % 5)) / 5;
}

int is_out_of_bounds(uint16_t i, uint16_t j) {
    return (i >= IMG_HEIGHT || j >= IMG_WIDTH);
}

void place_obstacle(int16_t x, int16_t y) {
    uint16_t i, j;
    i = coord_to_index(y);
    j = coord_to_index(x);
    int k, l, value;
    for (k = i; k < i + SIZE_OBSTACLE; k++) {
        for (l = j; l < j + SIZE_OBSTACLE; l++) {
            if (!is_out_of_bounds(k, l)) {
                value = get_cell(k, l);
                if (value < 2) {
                    set_cell(k, l, 2);
                }
                else {
                    set_cell(k, l, value + 1);
                }
            }
        }
    }
}

/* TODO: Offer a larger choice of colors */
color_t val_to_color(int value) {
    switch (value) {
        case 0:
            return WHITE;
        case 1:
            return WHITE;
        default:
            return BLACK;
    }
}

void send_image() {
    color_t col;
    int16_t i, j;
    for (i = 0; i < IMG_HEIGHT; i++) {
        for (j = 0; j < IMG_WIDTH; j++) {
            col = val_to_color(get_cell(i, j));
            send_mapdata(i, j, col.red, col.green, col.blue);
        }
    }
    send_mapdone();
}

/* For test purposes
int main() {
    init_image();
    set_cell(20, 12, 2);
    place_obstacle(12, 12);
    place_obstacle(12, 12);
    print_image();
    printf("La case (20, 12) a pour valeur %d.\n", get_cell(20, 12));
    return 0;
} */
