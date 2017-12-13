/* Written by Olivier Roques for the OS project.
Eurecom, 2017 - 2018. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "client.h"
#include "const.h"

#define DEBUG 0

int s;
uint16_t msgId = 1;

int read_from_server(char *buffer, size_t maxSize) {
    int bytes_read = recv(s, buffer, maxSize, 0);

    if (bytes_read <= 0) {
        printf("Server unexpectedly closed connection...\n");
        close_connection(s);
        return CONNECTION_ERROR;
    }
    if (DEBUG) {
        printf ("[DEBUG] received %d bytes\n", bytes_read);
    }
    return bytes_read;
}

int sent_to_server(char *buffer, size_t maxSize) {
    int bytes_sent = send(s, buffer, maxSize, 0);

    if (bytes_sent <= 0) {
        printf("The message could not be sent\n");
        close_connection(s);
        return CONNECTION_ERROR;
    }
    Sleep( 500 );
    if (DEBUG) {
        printf("[DEBUG] sent %d bytes\n", bytes_sent);
    }
    return bytes_sent;
}

/* For debugging purposes */
void print_message(char *message, size_t message_size) {
    unsigned int i;
    printf("[DEBUG] message sent:\n");
    for(i = 0; i < message_size; i++) {
        // Prints signed char by default. For unsigned char, use %08hhx.
        printf("%02hhx ", message[i]);
    }
    printf("\n");
}

void send_position(int16_t x, int16_t y) {
    char message[9];
    message[0] = msgId & 0x00ff;
    message[1] = (msgId & 0xff00) >> 8;
    msgId++;
    message[2] = TEAM_ID;
    message[3] = 0xFF;
    message[4] = MSG_POSITION;
    message[5] = x & 0x00ff;
    message[6] = (x & 0xff00) >> 8;
    message[7] = y & 0x00ff;
    message[8] = (y & 0xff00) >> 8;
    if (DEBUG) {
        printf("[DEBUG] SENDING POSITION\n");
        print_message(message, 9);
    }
    sent_to_server(message, 9);
}

void send_mapdata(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue) {
    char message[12];
    message[0] = msgId & 0x00ff;
    message[1] = (msgId & 0xff00) >> 8;
    msgId++;
    message[2] = TEAM_ID;
    message[3] = 0xFF;
    message[4] = MSG_MAPDATA;
    message[5] = x & 0x00ff;
    message[6] = (x & 0xff00) >> 8;
    message[7] = y & 0x00ff;
    message[8] = (y & 0xff00) >> 8;
    message[9] = red;
    message[10] = green;
    message[11] = blue;
    if (DEBUG) {
        printf("[DEBUG] SENDING MAPDATA\n");
        print_message(message, 12);
    }
    sent_to_server(message, 12);
}

void send_mapdone() {
    char message[5];
    message[0] = msgId & 0x00ff;
    message[1] = (msgId & 0xff00) >> 8;
    msgId++;
    message[2] = TEAM_ID;
    message[3] = 0xFF;
    message[4] = MSG_MAPDONE;
    if (DEBUG) {
        printf("[DEBUG] SENDING MAPDONE\n");
        print_message(message, 5);
    }
    sent_to_server(message, 5);
}

void send_obstacle(int16_t x, int16_t y, uint8_t act) {
    char message[10];
    message[0] = msgId & 0x00ff;
    message[1] = (msgId & 0xff00) >> 8;
    msgId++;
    message[2] = TEAM_ID;
    message[3] = 0xFF;
    message[4] = MSG_OBSTACLE;
    message[5] = act;
    message[6] = x & 0x00ff;
    message[7] = (x & 0xff00) >> 8;
    message[8] = y & 0x00ff;
    message[9] = (y & 0xff00) >> 8;
    if (DEBUG) {
        printf("[DEBUG] SENDING OBSTACLE\n");
        print_message(message,10);
    }
    sent_to_server(message, 10);
}

int open_connection() {
    struct sockaddr_rc addr = {0};
    int status;

    /* allocate a socket */
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    /* set the connection parameters (who to connect to) */
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(SERV_ADDR, &addr.rc_bdaddr);

    /* connect to server */
    printf("Connecting to the server... ");
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    /* if connected */
    if(status == 0) {
        char string[58];

        /* Wait for START message */
        read_from_server(string, 9);
        if (string[4] == MSG_START) {
            printf("Start message received!\n");
            Sleep(2000);
            return START_MESSAGE;
        }
        printf("Connected, but wrong start message.\n");
        close_connection();
        return WRONG_MESSAGE;
    }
    printf("Failed to connect to the server.\n");
    return CONNECTION_ERROR;
}

void close_connection() {
    printf("Closing connection\n");
    close(s);
}

/*
int main() {
    int16_t x     = 4;
    int16_t y     = 6;
    uint8_t red   = 204;
    uint8_t green = 85;
    uint8_t blue  = 0;
    uint8_t act   = 1;
    open_connection();
    send_position(x, y);
    send_obstacle(x, y, act);
    send_mapdata(x, y, red, green, blue);
    send_mapdone();
    return 0;
}
*/
