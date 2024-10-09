#include "API.c"
#include "API.h"

bool getWallLeft() {
    return API_wallLeft();
}

bool getWallFront() {
    return API_wallFront();
}

bool getWallRight() {
    return API_wallRight();
}

void setText(int x, int y, char text[]) {
    API_setText(x, y, text);
}

void setWall(int x, int y, char direction) {
    API_setWall(x, y, direction);
}

void turnRight() {
    API_turnRight();
}

void turnLeft() {
    API_turnLeft();
}

void move() {
    API_moveForward();
}