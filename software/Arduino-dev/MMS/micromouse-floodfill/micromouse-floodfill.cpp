#include "sim-interface.cpp"
#include <cstdint>

#define MAZE_SIZE 16
const int MAZE_HEIGHT = MAZE_SIZE;
const int MAZE_WIDTH = MAZE_SIZE;

const uint8_t NORTH = 1;
const uint8_t EAST = 2;
const uint8_t SOUTH = 4;
const uint8_t WEST = 8;

// Single node of maze
typedef struct Node {
  // X-coordinate
  uint8_t x;
  // Y-coordinate
  uint8_t y;
  // Projected distance to goal
  uint8_t dist;
  // Next node to flood
  Node *next;
} Node;

// 2D array of maze nodes
Node mazeNodes[MAZE_WIDTH][MAZE_HEIGHT];
// 3D array of maze walls (X, Y, V/H)
bool mazeWalls[MAZE_WIDTH + 1][MAZE_HEIGHT + 1][2] = {false};

// Current robot facing
uint8_t facing = NORTH;

uint8_t xPos = 0;
uint8_t yPos = 0;

bool backtracking = false;

bool useSim = true;

void floodFill(int goal[][2], int size) {
  API_clearAllText();
  // Next node to update
  Node *curr;
  // Last node to update
  Node *last;

  // Add goal nodes to queue
  for(int i = 0; i < size; i++) {
    if(i == 0) {
      curr = &mazeNodes[goal[i][0]][goal[i][1]];
    }
    else {
      last->next = &mazeNodes[goal[i][0]][goal[i][1]];
    }
    last = &mazeNodes[goal[i][0]][goal[i][1]];
    mazeNodes[goal[i][0]][goal[i][1]].dist = 0;
  }

  // Add unchecked nodes to queue and update projected distance from goal until all nodes have been updated
  while(curr) {
    if(useSim) {
      char distance_string[3];
      sprintf(distance_string, "%d", curr->dist);
      setText(curr->x, curr->y, distance_string);
    }
    // If north side of node is open, add north node to queue
    if(curr->y + 1 < MAZE_WIDTH && !mazeWalls[curr->x][curr->y + 1][1] && mazeNodes[curr->x][curr->y + 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y + 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If east side of node is open, add east node to queue
    if(curr->x + 1 < MAZE_HEIGHT && !mazeWalls[curr->x + 1][curr->y][0] && mazeNodes[curr->x + 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x + 1][curr->y];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If south side of node is open, add south node to queue
    if(curr->y >= 1 && !mazeWalls[curr->x][curr->y][1] && mazeNodes[curr->x][curr->y - 1].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x][curr->y - 1];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    // If west side of node is open, add west node to queue
    if(curr->x >= 1 && !mazeWalls[curr->x][curr->y][0] && mazeNodes[curr->x - 1][curr->y].dist > curr->dist + 1) {
      last->next = &mazeNodes[curr->x - 1][curr->y];
      last = last->next;
      last->dist = curr->dist + 1;
    }
    curr = curr->next;
  }
}

// Set up maze by assigning starting distances for each node
void setupMaze() {
  // Iterate through all nodes, setting initial values
  for(int x = 0; x < MAZE_WIDTH; x++) {
    for(int y = 0; y < MAZE_HEIGHT; y++) {
      mazeNodes[x][y].x = x;
      mazeNodes[x][y].y = y;
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
    }
  }
  int goal[4][2] = {{MAZE_WIDTH / 2, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2, MAZE_HEIGHT / 2 - 1}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2 - 1}};
  // Calculate distances from center using flood fill
  floodFill(goal, 4);
}

// Recalculate maze distances for each node
void recalcMaze(int goal[][2], int size) {
  // Iterate through all nodes, resetting distances
  for(int x = 0; x < MAZE_WIDTH; x++) {
    for(int y = 0; y < MAZE_HEIGHT; y++) {
      mazeNodes[x][y].dist = 100;
      mazeNodes[x][y].next = NULL;
    }
  }
  // Recalculate distances from center using flood fill
  floodFill(goal, size);
}

// Update walls from sensor readings
void updateWalls() {
  bool wallLeft = getWallLeft();
  bool wallFront = getWallFront();
  bool wallRight = getWallRight();
  switch(facing) {
    // Facing North
    case NORTH:
      mazeWalls[xPos][yPos][0] = wallLeft;
      mazeWalls[xPos][yPos + 1][1] = wallFront;
      mazeWalls[xPos + 1][yPos][0] = wallRight;
      if(useSim && wallLeft) {
        setWall(xPos, yPos, 'w');
      }
      if(useSim && wallFront) {
        setWall(xPos, yPos, 'n');
      }
      if(useSim && wallRight) {
        setWall(xPos, yPos, 'e');
      }
      break;
    // Facing East
    case EAST:
      mazeWalls[xPos][yPos + 1][1] = wallLeft;
      mazeWalls[xPos + 1][yPos][0] = wallFront;
      mazeWalls[xPos][yPos][1] = wallRight;
      if(useSim && wallLeft) {
        setWall(xPos, yPos, 'n');
      }
      if(useSim && wallFront) {
        setWall(xPos, yPos, 'e');
      }
      if(useSim && wallRight) {
        setWall(xPos, yPos, 's');
      }
      break;
    // Facing South
    case SOUTH:
      mazeWalls[xPos + 1][yPos][0] = wallLeft;
      mazeWalls[xPos][yPos][1] = wallFront;
      mazeWalls[xPos][yPos][0] = wallRight;
      if(useSim && wallLeft) {
        setWall(xPos, yPos, 'e');
      }
      if(useSim && wallFront) {
        setWall(xPos, yPos, 's');
      }
      if(useSim && wallRight) {
        setWall(xPos, yPos, 'w');
      }
      break;
    // Facing West
    case WEST:
      mazeWalls[xPos][yPos][1] = wallLeft;
      mazeWalls[xPos][yPos][0] = wallFront;
      mazeWalls[xPos][yPos + 1][1] = wallRight;
      if(useSim && wallLeft) {
        setWall(xPos, yPos, 's');
      }
      if(useSim && wallFront) {
        setWall(xPos, yPos, 'w');
      }
      if(useSim && wallRight) {
        setWall(xPos, yPos, 'n');
      }
      break;
  }
}

// Rotates to a particular facing
void rotate(uint8_t dir) {
  if((facing == NORTH && dir == EAST) ||
     (facing == EAST && dir == SOUTH) ||
     (facing == SOUTH && dir == WEST) ||
     (facing == WEST && dir == NORTH)) {
      turnRight();
     }
  else if((facing == NORTH && dir == SOUTH) ||
     (facing == EAST && dir == WEST) ||
     (facing == SOUTH && dir == NORTH) ||
     (facing == WEST && dir == EAST)) {
      turnRight();
      turnRight();
     }
  else if((facing == NORTH && dir == WEST) ||
     (facing == EAST && dir == NORTH) ||
     (facing == SOUTH && dir == EAST) ||
     (facing == WEST && dir == SOUTH)) {
      turnLeft();
     }
}

// Determines direction to move, rotates to that direction, and moves forward (recalculates projected distances if no reasonable move found)
// Returns true if the move was successful, or false if not
bool rotateMove() {
  uint8_t dist = mazeNodes[xPos][yPos].dist;
  // If moving North lowers distance to goal, move North
  if(!mazeWalls[xPos][yPos + 1][1] && yPos + 1 < MAZE_HEIGHT && mazeNodes[xPos][yPos + 1].dist < dist) {
    rotate(NORTH);
    facing = NORTH;
    move();
    yPos = yPos + 1;
    return true;
  }
  // If moving East lowers distance to goal, move East
  else if(!mazeWalls[xPos + 1][yPos][0] && xPos + 1 < MAZE_WIDTH && mazeNodes[xPos + 1][yPos].dist < dist) {
    rotate(EAST);
    facing = EAST;
    move();
    xPos = xPos + 1;
    return true;
  }
  // If moving West lowers distance to goal, move West
  else if(!mazeWalls[xPos][yPos][0] && mazeNodes[xPos][yPos].x >= 1 && mazeNodes[xPos - 1][yPos].dist < dist) {
    rotate(WEST);
    facing = WEST;
    move();
    xPos = xPos - 1;
    return true;
  }
  // If moving South lowers distance to goal, move South
  if(!mazeWalls[xPos][yPos][1] && mazeNodes[xPos][yPos].y >= 1 && mazeNodes[xPos][yPos - 1].dist < dist) {
    rotate(SOUTH);
    facing = SOUTH;
    move();
    yPos = yPos - 1;
    return true;
  }
  if(mazeNodes[xPos][yPos].dist == 0) {
    backtracking = !backtracking;
  }
  if(!backtracking) {
    int goal[4][2] = {{MAZE_WIDTH / 2, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2}, {MAZE_WIDTH / 2, MAZE_HEIGHT / 2 - 1}, {MAZE_WIDTH / 2 - 1, MAZE_HEIGHT / 2 - 1}};
    recalcMaze(goal, 4);
    return false;
  }
  int goal[1][2] = {{0, 0}};
  recalcMaze(goal, 1);
  return false;
}

void setup() {
  // put your setup code here, to run once:
  setupMaze();
}

void loop() {
  // put your main code here, to run repeatedly:
  updateWalls();
  rotateMove();
}

void main() {
  setup();
  if(useSim) {
    setWall(0, 0, 's');
  }
  while(true) {
    loop();
  }
}