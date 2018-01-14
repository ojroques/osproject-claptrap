/* Written by Olivier Roques for the OS project.
Eurecom, 2017 - 2018. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

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
        printf("%s: (%d, %d)\n", STR_OUT_OF_BOUNDS, i, j);
        return ERR_OUT_OF_BOUNDS;
    }
    return image[i][j];
}

/* Set the cell at given coordinates with given value*/
int set_cell(uint16_t i, uint16_t j, int value) {
    if (is_out_of_bounds(i, j)) {
        printf("%s: (%d, %d)\n", STR_OUT_OF_BOUNDS, i, j);
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

/* UTILITY FUNCTIONS */
/* Function to get minimum of three values */
int min(int a, int b, int c) {
    int m = a;
    if (m > b) m = b;
    if (m > c) m = c;
    return m;
}

/* Place in x, y the center of the largest square
 * of unexplored area fitting inside the image */
void unexplored_area(int16_t *x, int16_t *y) {
    int i, j;
    int S[img_height][img_width];
    int max_of_s, max_i, max_j;

    /* Set first column of S[][]*/
    for (i = 0; i < img_height; i++)
        S[i][0] = image[i][0];

    /* Set first row of S[][]*/
    for (j = 0; j < img_width; j++)
        S[0][j] = image[0][j];

    /* Construct other entries of S[][]*/
    for (i = 1; i < img_height; i++) {
        for (j = 1; j < img_width; j++) {
            if (image[i][j] == 0)
                S[i][j] = min(S[i][j-1], S[i-1][j], S[i-1][j-1]) + 1;
            else
                S[i][j] = 0;
        }
    }

    /* Find the maximum entry, and indexes of maximum entry in S[][] */
    max_of_s = S[0][0];
    max_i = 0;
    max_j = 0;
    for (i = 0; i < img_height; i++) {
        for (j = 0; j < img_width; j++) {
            if (max_of_s < S[i][j]) {
                max_of_s = S[i][j];
                max_i = i;
                max_j = j;
            }
        }
    }

    *x = index_to_coord(max_i - (max_of_s / 2));
    *y = index_to_coord(max_j - (max_of_s / 2));
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
    int value;
    color_t col;
    int16_t i, j;
    for (i = 0; i < img_height; i++) {
        for (j = 0; j < img_width; j++) {
            value = get_cell(i, j);
            if (value > 1) {
                col = val_to_color(value);
                send_mapdata(i, j, col.red, col.green, col.blue);
            }
        }
    }
    send_mapdone();
}

/* Erwan
Update matrix after the robot moved */
void explored_line(int16_t x_start, int16_t x_finish, int16_t y_start, int16_t y_finish){
    int16_t delta_x = x_finish-x_start;
    int16_t delta_y = y_finish-y_start;
    if( delta_x != 0) {
        float slope = ((float)delta_y)/((float)delta_x);
        float intercept = -slope*x_start + y_start;
        int16_t x_previous_update = x_start;
        int16_t y_previous_update = y_start;

        if (!is_out_of_bounds(coord_to_index(y_previous_update), coord_to_index(x_previous_update))) {
            set_cell(coord_to_index(y_previous_update), coord_to_index(x_previous_update), 1);
        }

        int precision = floor(abs(slope*(delta_x)) + 1);
        float current_x;
        float current_y;
        int i;
        for (i = 0; i <= precision; i++){
            current_x = x_start + i*(delta_x)/precision;
            current_y = slope*current_x + intercept;
            if(coord_to_index((round)(current_x + 0.5)) != coord_to_index(x_previous_update) || coord_to_index((round)(current_y + 0.5)) != coord_to_index(y_previous_update)){
                x_previous_update = (round)(current_x + 0.5);
                y_previous_update = (round)(current_y + 0.5);
                if (!is_out_of_bounds(coord_to_index(y_previous_update), coord_to_index(x_previous_update))) {
                    set_cell(coord_to_index(y_previous_update), coord_to_index(x_previous_update), 1);
                }
            }
        }
    }
    else {
        int16_t y_previous_update = y_start;
        int16_t x_previous_update = x_start;
        if (!is_out_of_bounds(coord_to_index(y_previous_update), coord_to_index(x_previous_update))) {
            set_cell(coord_to_index(y_previous_update), coord_to_index(x_previous_update), 1);
        }
        int precision = floor(abs(delta_y)+1);
        float current_y;
        int i;
        for (i = 0; i <= precision; i++){
            current_y = y_start + i*(delta_y)/precision;
            if(coord_to_index((round)(current_y + 0.5)) != coord_to_index(y_previous_update)){
                y_previous_update = (round)(current_y + 0.5);
                if (!is_out_of_bounds(coord_to_index(y_previous_update), coord_to_index(x_previous_update))) {
                    set_cell(coord_to_index(y_previous_update), coord_to_index(x_previous_update), 1);
                }
            }
        }
    }
}

/* For test purposes
int main() {
    int16_t x_free, y_free;
    init_image(24, 40);
    set_cell(20, 12, 2);
    place_obstacle(12, 12);
    place_obstacle(12, 12);
    place_obstacle(5, 5);
    explored_line(0,600,400,0);
    print_image();
    // printf("La case (20, 12) a pour valeur %d\n", get_cell(20, 12));
    // unexplored_area(&x_free, &y_free);
    // printf("Centre de la zone inexploree: (%d, %d)\n", x_free, y_free);
    // open_connection();
    // printf("Envoi de la map... ");
    // send_image();
    // printf("Done.\n");
    return 0;
} */
