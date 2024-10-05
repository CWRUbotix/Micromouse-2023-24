/**
 * @file Converted.c
 * @author kaleb
 * @brief converting micromouse.ino to a c file calling mms AP
 ****** CURRENTLY BROKEN (WIP)
 * @date 2023-11-10
 * 
 * this entire description was auto-generated haha
 */

#define MAZE_SIZE 10

#include <stdio.h>
#include <stdbool.h>

#include "API.h"

const int8_t NORTH = 1;
const int8_t EAST = 2;
const int8_t SOUTH = 4;
const int8_t WEST = 8;

// Single node of maze
typedef struct Node {
  // X-coordinate
  int8_t x;
  // Y-coordinate
  int8_t y;
  // Projected distance to goal
  int8_t dist;
  // Next node to flood
  Node *next;
} Node;

// 2D array of maze nodes
Node mazeNodes[MAZE_SIZE][MAZE_SIZE];
// 3D array of maze walls (X, Y, V/H)
bool mazeWalls[MAZE_SIZE + 1][MAZE_SIZE + 1][2] = false;

// Current robot facing (start facing clockwise around maze)
int8_t facing = EAST;
int8_t xPos = 0;
int8_t yPos = 0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Set up maze by assigning starting distances for each node
void setupMaze() {
  // Iterate through all nodes, setting initial values
  for(int x = 0; x < MAZE_SIZE; x++) {
    for(int y = 0; y < MAZE_SIZE; y++) {
      mazeNodes[x][y].x = x;
      mazeNodes[x][y].y = y;
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
      mazeWalls[x][y][0] = false;
      mazeWalls[x][y][1] = false;
    }
  }
  // Calculate distances from center using flood fill
  floodFill();
}

// Recalculate maze distances for each node
void recalcMaze() {
  // Iterate through all nodes, resetting distances
  for(int x = 0; x < MAZE_SIZE; x++) {
    for(int y = 0; y < MAZE_SIZE; y++) {
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
    }
  }
  // Recalculate distances from center using flood fill
  floodFill();
}

void floodFill() {
  // Next node to update
  Node *curr;
  // Last node to update
  Node *last;

  // Add four goal nodes to queue
  curr = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2];
  last = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2];
  mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2].dist = 0;
  last->next = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2];
  last = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2];
  mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2].dist = 0;
  last->next = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1];
  last = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1];
  mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1].dist = 0;
  last->next = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1];
  last = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1];
  mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1].dist = 0;

  // Add unchecked nodes to queue and update projected distance from goal until all nodes have been updated
  while(curr) {
    // If north side of node is open, add north node to queue
    if(curr->y >= 1 && !mazeWalls[curr->x][curr->y][1] && mazeNodes[curr->x][curr->y - 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If east side of node is open, add east node to queue
    if(curr->x + 1 < MAZE_SIZE && !mazeWalls[curr->x + 1][curr->y][0] && mazeNodes[curr->x + 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If south side of node is open, add south node to queue
    if(curr->y + 1 < MAZE_SIZE && !mazeWalls[curr->x][curr->y + 1][1] && mazeNodes[curr->x][curr->y + 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If west side of node is open, add west node to queue
    if(curr->x >= 1 && !mazeWalls[curr->x][curr->y][0] && mazeNodes[curr->x - 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    curr = curr->next;
  }
}

// Update walls from sensor readings
void updateWalls() {
  bool wallLeft = checkWall(WEST);
  bool wallFront = checkWall(NORTH);
  bool wallRight = checkWall(EAST);
  switch(facing) {
    // Facing North
    case NORTH:
      mazeWalls[xPos][yPos][0] = wallLeft;
      mazeWalls[xPos][yPos][1] = wallFront;
      mazeWalls[xPos + 1][yPos][0] = wallRight;
      break;
    // Facing East
    case EAST:
      mazeWalls[xPos][yPos][1] = wallLeft;
      mazeWalls[xPos + 1][yPos][0] = wallFront;
      mazeWalls[xPos][yPos + 1][1] = wallRight;
      break;
    // Facing South
    case SOUTH:
      mazeWalls[xPos + 1][yPos][0] = wallLeft;
      mazeWalls[xPos][yPos + 1][1] = wallFront;
      mazeWalls[xPos][yPos][0] = wallRight;
      break;
    // Facing West
    case WEST:
      mazeWalls[xPos][yPos + 1][1] = wallLeft;
      mazeWalls[xPos][yPos][0] = wallFront;
      mazeWalls[xPos][yPos][1] = wallRight;
      break;
  }
}

// Determines direction to move, rotates to that direction, and moves forward (recalculates projected distances if no reasonable move found)
// Returns true if the move was successful, or false if not
bool rotateMove() {
  int8_t dist = mazeNodes[xPos][yPos].dist;
  if(!mazeWalls[xPos][yPos + 1][1] && yPos + 1 < MAZE_SIZE && mazeNodes[xPos][yPos + 1].dist < dist) {
    rotate(SOUTH);
    move();
    return true;
  }
  else if(!mazeWalls[xPos + 1][yPos][0] && xPos + 1 < MAZE_SIZE && mazeNodes[xPos + 1][yPos].dist < dist) {
    rotate(EAST);
    move();
    return true;
  }
  else if(!mazeWalls[xPos][yPos][0] && curr->x >= 1 && mazeNodes[xPos - 1][yPos].dist < dist) {
    rotate(WEST);
    move();
    return true;
  }
  else if(!mazeWalls[xPos][yPos][1] && curr->y >= 1 && mazeNodes[xPos][yPos - 1].dist < dist) {
    rotate(NORTH);
    move();
    return true;
  }
  recalcMaze();
  return false;
}