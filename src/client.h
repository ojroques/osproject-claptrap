#ifndef CLIENT_H
#define CLIENT_H

#define SERV_ADDR          "00:28:f8:51:71:4b"   /* The address of the server */
#define TEAM_ID           7    /* The team ID */
#define MSG_ACK           0
#define MSG_START         1
#define MSG_STOP          2
#define MSG_KICK          3
#define MSG_POSITION      4
#define MSG_MAPDATA       5
#define MSG_MAPDONE       6
#define MSG_OBSTACLE      7
#define CONNECTION_ERROR  0
#define WRONG_MESSAGE     1
#define START_MESSAGE     2

uint16_t coord_to_index(int16_t c);
int16_t index_to_coord(uint16_t x);
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
