/* --- Includes --- */
// Boolean definition necessary for C
#include <stdbool.h>
// Defines abs() function
#include <stdlib.h>
/* ---- Defines ---- */
// #define DEBUG

#ifdef DEBUG
  #define log(...) fprintf(stderr, __VA_ARGS__); fflush(stderr)
  #define logf(...) fprintf(stderr, __VA_ARGS__); fflush(stderr)
  #define logln(...) fprintf(stderr, __VA_ARGS__); fflush(stderr)
  #define LOGGING 1
#else
  #define log(...)
  #define logf(...)
  #define logln(...)
  #define LOGGING 0
#endif

/* ---- User Variables ---- */
 
typedef struct Node {
  // X-coordinate
  int x;
  // Y-coordinate
  int y;
  // Projected distance to goal
  int guess;
  // Next node to flood
  struct Node *next;
} Node;

// A node used in the Flood Fill sub-algorithm
typedef struct FFNode {
  bool closed;
  int x;
  int y;
} FFNode; 

// Null pointer
#define NULL ((void *)0)

#define MAZE_SIZE          16
// 0 - Basic A*
// 1 - Hallway following - if there is only a single open node adjacent to the robot, we go there, regardless of score
// 2 - Greedy heuristic - when calculating score, we value getting close to the center 5x more than staying close to
//                        the start (like a string pulled taunt from two sides, the shortest path values both equally)
// 3 - Greedy heuristic and hallway following
#define MAPPING_MODE       0

// This is the midpoint of the maze
// Okay. The heuristic that A* uses is taxi-cab distance to the pin in the middle of
//  the 4x4 goal in the middle of the maze.
//  This pin is located at (4.5, 4.5) (since we've 0-indexed node locations)
//  However, the distance from the center of any node to 4.5, 4.5, by taxi-cab distance is a whole number
//  To avoid floating point math (I don't like floating point), we multiply by 10
//  (Everywhere else, distances, scores, and node locations are absolute, we only multiple by 10 in h())
#define END_X              75
#define END_Y              75

// Used as parameters to the motor functions
enum turning_direction_t {LEFT = -1, RIGHT = 1};

enum cardinal_t {NORTH, EAST, SOUTH, WEST};

/* ---- User Variables ---- */
// The maze as a 2D array
// maze[Y][X][1: vertical; 0: horizontal]
// And the number represents the confidence
// So 0 is we have no idea, and positive is wall, and negative is no wall
int maze[MAZE_SIZE + 1][MAZE_SIZE + 1][2];

// The list of nodes
// Sorted such that closed nodes are 0..closedNodes-1
// And nodes after that increase in score
// This is the canonical list of nodes, order doesn't matter
// All other nodes are pointers into nodesStatic
Node nodesStatic[MAZE_SIZE * MAZE_SIZE];
// A*'s working list of nodes. 0..closedNodes-1 are closed, and closedNodes..numNodes-1 are open
Node *mazeNodes[MAZE_SIZE * MAZE_SIZE];
int closedNodes = 0;
int numNodes = 0;

//The node the robot is currently at (or that A* thinks we're at)
Node *current = NULL;
//The current goal node
Node *goal;

Node *backPath[MAZE_SIZE * MAZE_SIZE]; // Used to store the path along which we backtrack to get to goal
int backPathLength = 0;

// The main path, once we've solved the maze, the route we use
Node *mainPath[MAZE_SIZE * MAZE_SIZE];
int pathLength = 0;

// Create a robot
// Only stores x, y, and facing direction
struct RobotStruct {
  int x;
  int y;
  enum cardinal_t facing;
} robot = {
  0, 0, NORTH
};

bool shouldFloodFill = false;
int mappingMode = MAPPING_MODE;

/* ---- User Functions ---- */
// Method used to update the maze
void updateMaze();
int moveForward();
void turnRight();
void turnLeft();

//Causes the robot to do a point-turn to spin to the desired location
void turnTo(enum cardinal_t direction) {
  if (robot.facing == direction) {
    // return, we're done!
  }else if (
    (robot.facing == NORTH && direction == EAST) ||
    (robot.facing == EAST && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == WEST) ||
    (robot.facing == WEST && direction == NORTH)
  ) {
    //Rotate 90º Right
    turnRight();
  }else if (
    (robot.facing == NORTH && direction == WEST) ||
    (robot.facing == WEST && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == EAST) ||
    (robot.facing == EAST && direction == NORTH)
  ) {
    //Rotate 90º Left
    turnLeft();
  }else if (
    (robot.facing == NORTH && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == NORTH) ||
    (robot.facing == EAST && direction == WEST) ||
    (robot.facing == WEST && direction == EAST)
  ) {
    // Rotate 180º
    turnRight();
    turnRight();
  }

  robot.facing = direction;
}

// Moves the robot to a node which is adjacent to the current node
// adjNode is goal, is x: 1, y: 0
void moveRobot(Node *adjNode) {
  // Figure out what direction the node is in
  enum cardinal_t direction;
  if (adjNode->x + 1 == robot.x) {
    direction = WEST;
  }else if (adjNode->x - 1 == robot.x) {
    direction = EAST;
  }else if (adjNode->y + 1 == robot.y) {
    direction = SOUTH;
  }else if (adjNode->y - 1 == robot.y) {
    direction = NORTH;
  }else if (adjNode->x == robot.x && adjNode->y == robot.y) {
    // log("WARN: moveRobot called with current location\n");
    return;
  }else {
    log("AAAAahaaahhhah\n");
    direction = NORTH;
  }

  // logf("Spinning to direction #%d\n", direction);
  turnTo(direction);

  moveForward();

  robot.x = adjNode->x;
  robot.y = adjNode->y;
}
// Check if we are in any of the four finish squares
int isGoal(int x, int y) {
  return (
    (x == END_X/10 && y == END_Y/10) ||
    (x == END_X/10 && y == END_Y/10 + 1) ||
    (x == END_X/10 + 1 && y == END_Y/10) ||
    (x == END_X/10 + 1 && y == END_Y/10 + 1)
  );
}

