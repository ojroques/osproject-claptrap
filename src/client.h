#ifndef __CLIENT_H
#define __CLIENT_H

int read_from_server(char *buffer, size_t maxSize);
int sent_to_server(char *buffer, size_t maxSize);
void print_message(char *message, size_t message_size);
void send_position(int16_t x, int16_t y);
void send_mapdata(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue);
void send_mapdone();
void send_obstacle(int16_t x, int16_t y, uint8_t act);
int open_connection();
void close_connection();

#endif
