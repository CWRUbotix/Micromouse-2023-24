/**
 * @file Converted.c
 * @author kaleb
 * @brief converting micromouse.ino to a c file calling mms AP
 ****** CURRENTLY BROKEN (WIP)
 * @date 2023-11-10
 * 
 * this entire description was auto-generated haha
 */
#include "Micro.c"

#include <stdio.h>
#include <stdbool.h>

// Single node of maze

/* ---- Variables ---- */
// 2D array of maze nodes
// Node mazeNodes[MAZE_SIZE][MAZE_SIZE];

// The maze as a 2D array
// maze[Y][X][1: vertical; 0: horizontal]
// And the number represents the confidence
// So 0 is we have no idea, and positive is wall, and negative is no wall
int maze[MAZE_SIZE + 1][MAZE_SIZE + 1][2];

/* ---- METHODS ---- */
// (Not yet implemented)
void createBackPath(){}
void createPath(){}
void interimLogic(){}
void closeNode(){}

void floodFill();
// void moveRobot();
// int isGoal();
// Initialize maze by assigning starting distances for each node
void init_maze() {
  // Iterate through all nodes, setting initial values
  for(int x = 0; x < MAZE_SIZE; x++) {
    for(int y = 0; y < MAZE_SIZE; y++) {
      mazeNodes[x][y].x = x;
      mazeNodes[x][y].y = y;
      mazeNodes[x][y].guess = 100;
      mazeNodes[x][y].next = NULL;
      maze[x][y][0] = false;
      maze[x][y][1] = false;
    }
    maze[MAZE_SIZE][MAZE_SIZE][0] = false;
    maze[MAZE_SIZE][MAZE_SIZE][1] = false;
  }
  // Calculate distances from center using flood fill
  floodFill();
}

// Recalculate maze distances for each node
void recalcMaze() {
  // Iterate through all nodes, resetting distances
  for(int x = 0; x < MAZE_SIZE; x++) {
    for(int y = 0; y < MAZE_SIZE; y++) {
      mazeNodes[x][y].guess = 100;
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
  mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2].guess = 0;

  last->next = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2];
  last = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2];
  mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2].guess = 0;

  last->next = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1];
  last = &mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1];
  mazeNodes[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1].guess = 0;

  last->next = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1];
  last = &mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1];
  mazeNodes[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1].guess = 0;

  // Add unchecked nodes to queue and update projected distance from goal until all nodes have been updated
  while(curr) {
    // If north side of node is open, add north node to queue
    if(curr->y >= 1 && !maze[curr->x][curr->y][1] && mazeNodes[curr->x][curr->y - 1].guess > curr->guess + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->guess = curr->guess + 1;
    }
    // If east side of node is open, add east node to queue
    if(curr->x + 1 < MAZE_SIZE && !maze[curr->x + 1][curr->y][0] && mazeNodes[curr->x + 1][curr->y].guess > curr->guess + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->guess = curr->guess + 1;
    }
    // If south side of node is open, add south node to queue
    if(curr->y + 1 < MAZE_SIZE && !maze[curr->x][curr->y + 1][1] && mazeNodes[curr->x][curr->y + 1].guess > curr->guess + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->guess = curr->guess + 1;
    }
    // If west side of node is open, add west node to queue
    if(curr->x >= 1 && !maze[curr->x][curr->y][0] && mazeNodes[curr->x - 1][curr->y].guess > curr->guess + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->guess = curr->guess + 1;
    }
    curr = curr->next;
  }
}

void updateGoal(){
  int guess = mazeNodes[robot.x][robot.y].guess;
  // North
  if(robot.y + 1 < MAZE_SIZE && !maze[robot.x][robot.y + 1][1] && mazeNodes[robot.x][robot.y + 1].guess < guess) {
    goal = &mazeNodes[robot.x][robot.y + 1];
  }
  // East
  else if(robot.x + 1 < MAZE_SIZE && !maze[robot.x + 1][robot.y][1] && mazeNodes[robot.x + 1][robot.y].guess < guess) {
    goal = &mazeNodes[robot.x + 1][robot.y];
  }
  // South
  else if(robot.y - 1 < MAZE_SIZE && !maze[robot.x][robot.y][0] && mazeNodes[robot.x][robot.y - 1].guess < guess) {
    goal = &mazeNodes[robot.x][robot.y - 1];
  }
  // West
  else if(robot.x - 1 < MAZE_SIZE && !maze[robot.x][robot.y][1] && mazeNodes[robot.x - 1][robot.y].guess < guess) {
    goal = &mazeNodes[robot.x - 1][robot.y];
  }

}
void moveToGoal() {
  moveRobot(goal);
}