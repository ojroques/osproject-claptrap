#ifndef __IMAGE_H
#define __IMAGE_H

void print_image();
void init_image();
int get_cell(uint16_t i, uint16_t j);
int set_cell(uint16_t i, uint16_t j, int value);
uint16_t coord_to_index(int16_t c);
int is_out_of_bounds(uint16_t i, uint16_t j);
void place_obstacle(int16_t x, int16_t y);

#endif
