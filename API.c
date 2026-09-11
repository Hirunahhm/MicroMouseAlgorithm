#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 32

// The simulator drives the mouse by request and reply over stdin/stdout, so
// every read below blocks until it answers. A closed stdin means the
// simulator is gone and there is nothing left to do, which is worth saying
// out loud rather than carrying on over an uninitialised buffer.
static void readLine(char* response, int size) {
    if (fgets(response, size, stdin) == NULL) {
        fprintf(stderr, "simulator closed the connection\n");
        fflush(stderr);
        exit(EXIT_FAILURE);
    }
}

int getInteger(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    readLine(response, BUFFER_SIZE);
    int value = atoi(response);
    return value;
}

int getBoolean(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    readLine(response, BUFFER_SIZE);
    int value = (strcmp(response, "true\n") == 0);
    return value;
}

int getAck(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    readLine(response, BUFFER_SIZE);
    int success = (strcmp(response, "ack\n") == 0);
    return success;
}

int API_mazeWidth() {
    return getInteger("mazeWidth");
}

int API_mazeHeight() {
    return getInteger("mazeHeight");
}

int API_wallFront() {
    return getBoolean("wallFront");
}

int API_wallRight() {
    return getBoolean("wallRight");
}

int API_wallLeft() {
    return getBoolean("wallLeft");
}

int API_wallBack() {
    return getBoolean("wallBack");
}

int API_moveForward() {
    return getAck("moveForward");
}

void API_turnRight() {
    getAck("turnRight");
}

void API_turnLeft() {
    getAck("turnLeft");
}

void API_setWall(int x, int y, char direction) {
    printf("setWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_clearWall(int x, int y, char direction) {
    printf("clearWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_setColor(int x, int y, char color) {
    printf("setColor %d %d %c\n", x, y, color);
    fflush(stdout);
}

void API_clearColor(int x, int y) {
    printf("clearColor %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllColor() {
    printf("clearAllColor\n");
    fflush(stdout);
}

void API_setText(int x, int y, const char* text) {
    printf("setText %d %d %s\n", x, y, text);
    fflush(stdout);
}

void API_clearText(int x, int y) {
    printf("clearText %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllText() {
    printf("clearAllText\n");
    fflush(stdout);
}

int API_wasReset() {
    return getBoolean("wasReset");
}

void API_ackReset() {
    getAck("ackReset");
}

double API_getStat(const char* stat) {
    printf("getStat %s\n", stat);
    fflush(stdout);
    char response[BUFFER_SIZE];
    readLine(response, BUFFER_SIZE);
    if (response[0] == '\n' || response[0] == '\0') return -1.0;
    return atof(response);
}
