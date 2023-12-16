#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 32

int getInteger(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int value = atoi(response);
    return value;
}

// False has value of negative 1
int getBoolean(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int value = (strcmp(response, "true\n") == 0);
    // if (value = 0)
    //     value = -1;
    return value;
}

int getAck(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int success = (strcmp(response, "ack\n") == 0);
    return success;
}

int mazeWidth() {
    return getInteger("mazeWidth");
}

int mazeHeight() {
    return getInteger("mazeHeight");
}

int wallFront() {
    return getBoolean("wallFront");
}

int wallRight() {
    return getBoolean("wallRight");
}

int wallLeft() {
    return getBoolean("wallLeft");
}

int moveForward() {
    return getAck("moveForward");
}

void turnRight() {
    getAck("turnRight");
}

void turnLeft() {
    getAck("turnLeft");
}

void setWall(int x, int y, char direction) {
    printf("setWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void clearWall(int x, int y, char direction) {
    printf("clearWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void setColor(int x, int y, char color) {
    printf("setColor %d %d %c\n", x, y, color);
    fflush(stdout);
}

void clearColor(int x, int y) {
    printf("clearColor %d %d\n", x, y);
    fflush(stdout);
}

void clearAllColor() {
    printf("clearAllColor\n");
    fflush(stdout);
}

void setText(int x, int y, int text) {
    printf("setText %d %d %d\n", x, y, text);
    fflush(stdout);
}

void clearText(int x, int y) {
    printf("clearText %d %d\n", x, y);
    fflush(stdout);
}

void clearAllText() {
    printf("clearAllText\n");
    fflush(stdout);
}

int wasReset() {
    return getBoolean("wasReset");
}

void ackReset() {
    getAck("ackReset");
}
