/* Written by Olivier Roques for the OS project.
Eurecom, 2017 - 2018. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "image.h"
#include "const.h"
#include "client.h"

int img_width, img_height;       // The map dimensions
int **image;                     // The matrix representing the map
const char *STR_OUT_OF_BOUNDS = "Index out of bounds exception";

// Declare colors using the RGB syntax
const color_t WHITE = {255, 255, 255};
const color_t BLACK = {0, 0, 0};
const color_t RED   = {255, 0, 0};
const color_t GREEN = {0, 255, 0};
const color_t BLUE  = {0, 0, 255};

/* For debugging purposes only */
void print_image() {
    int i, j;
    printf("\n");
    for (i = 0; i < img_height; i++) {
        for (j = 0; j < img_width; j++) {
            printf("%d  ", image[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

/* Create a global matrix of given height and width by dynamic allocation
   then fill this matrix with 0s. */
void init_image(int width, int height) {
    int i, j;

    img_width  = width;
    img_height = height;
    image      = (int **)malloc(height * sizeof(int *)); // Allocate the rows
    for (i = 0; i < img_height; i++) {
        image[i] = (int *)malloc(width * sizeof(int));   // Allocate the columns
    }

    // Initialize the matrix with 0s
    for (i = 0; i < img_height; i++) {
        for (j = 0; j < img_width; j++) {
            image[i][j] = 0;
        }
    }
}

/* Return the value of the cell at given coordinates */
int get_cell(uint16_t i, uint16_t j) {
    if (is_out_of_bounds(i, j)) {
        printf("%s\n", STR_OUT_OF_BOUNDS);
        return ERR_OUT_OF_BOUNDS;
    }
    return image[i][j];
}

/* Set the cell at given coordinates with given value*/
int set_cell(uint16_t i, uint16_t j, int value) {
    if (is_out_of_bounds(i, j)) {
        printf("%s\n", STR_OUT_OF_BOUNDS);
        return ERR_OUT_OF_BOUNDS;
    }
    image[i][j] = value;
    return 0;
}

/* Check if given indexes are not out of bounds */
int is_out_of_bounds(uint16_t i, uint16_t j) {
    return (i >= img_height || j >= img_width);
}

/* Place an obstacle at given coordinates. To do so, a 2 x 2 block is
incremented within the image.
0 means that the area has not been explored yet.
1 means that the area has been explored and is clean from obstacles.
2 and more means that there is probably an obstacle in the area. The higher the
value is, the higher the probability of an obstacle. */
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

/* Convert the given value to a color based on the probability of presence of
   an obstacle. A larger value means a higher probability.
   TODO: Offer a larger choice of colors */
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

/* Send the resulting map to the server */
void send_image() {
    color_t col;
    int16_t i, j;
    for (i = 0; i < img_height; i++) {
        for (j = 0; j < img_width; j++) {
            col = val_to_color(get_cell(i, j));
            send_mapdata(i, j, col.red, col.green, col.blue);
        }
    }
    send_mapdone();
}

/* For test purposes only
int main() {
    init_image(24, 40);
    set_cell(20, 12, 2);
    place_obstacle(12, 12);
    place_obstacle(12, 12);
    print_image();
    printf("La case (20, 12) a pour valeur %d.\n", get_cell(20, 12));
    return 0;
}
*/
