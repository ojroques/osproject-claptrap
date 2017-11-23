#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "client.h"
#include "const.h"

int s;

int read_from_server(int sock, char *buffer, size_t maxSize) {
    int bytes_read = read (sock, buffer, maxSize);

    if (bytes_read <= 0) {
        printf("Server unexpectedly closed connection...\n");
        closeConnection(s);
        return CONNECTION_ERROR;
    }

    printf ("[DEBUG] received %d bytes\n", bytes_read);
    return bytes_read;
}

int openConnection() {
    struct sockaddr_rc addr = {0};
    int status;

    /* allocate a socket */
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    /* set the connection parameters (who to connect to) */
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba (SERV_ADDR, &addr.rc_bdaddr);

    /* connect to server */
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    /* if connected */
    if(status == 0) {
        char string[58];

        /* Wait for START message */
        read_from_server(s, string, 9);
        if (string[4] == MSG_START) {
            printf ("Received start message!\n");
            Sleep(2000);
            return START_MESSAGE;
        }
        printf("Connected, but wrong start message\n");
        closeConnection();
        return WRONG_MESSAGE;
    }
    printf("Failed to connect to the server\n");
    return CONNECTION_ERROR;
}

void closeConnection() {
    printf("Closing connection\n");
    close(s);
}
