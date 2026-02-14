#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>


//192.168.1.100

int main() {
    int sock = 0;
    struct sockaddr_in serv_addr;

    // 1. Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    // 2. Server info
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8080);  // Remote server port

    if (inet_pton(AF_INET, "192.168.1.100", &serv_addr.sin_addr) <= 0) {
        perror("inet_pton");
        return 1;
    }

    // 3. Connect
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("connect");
        return 1;
    }

    printf("Connected to server!\n");

    // 4. Send and receive
    const char *msg = "Hello from Raspberry Pi 5!\n";
    send(sock, msg, strlen(msg), 0);

    char buf[1024] = {0};
    int n = recv(sock, buf, sizeof(buf)-1, 0);
    if (n > 0) {
        printf("Received: %s\n", buf);
    }

    close(sock);
    return 0;
}
