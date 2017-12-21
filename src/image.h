#ifndef IMAGE_H
#define IMAGE_H

#define SIZE_OBSTACLE       2   /* The size of an obstacle's side */
#define IMG_WIDTH          24
#define IMG_HEIGHT         40
#define ERR_OUT_OF_BOUNDS  -1

typedef struct color_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_t;

void print_image();
void init_image();
int get_cell(uint16_t i, uint16_t j);
int set_cell(uint16_t i, uint16_t j, int value);
int is_out_of_bounds(uint16_t i, uint16_t j);
void place_obstacle(int16_t x, int16_t y);
color_t val_to_color(int value);
void send_image();

#endif
