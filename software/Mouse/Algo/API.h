#pragma once

int mazeWidth();
int mazeHeight();

int wallFront();
int wallRight();
int wallLeft();

int moveForward();  // Returns 0 if crash, else returns 1
void turnRight();
void turnLeft();

void setWall(int x, int y, char direction);
void clearWall(int x, int y, char direction);

void setColor(int x, int y, char color);
void clearColor(int x, int y);
void clearAllColor();

void setText(int x, int y, char* str);
void clearText(int x, int y);
void clearAllText();

int wasReset();
void ackReset();