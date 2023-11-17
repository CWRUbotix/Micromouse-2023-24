/* --- Includes --- */
// Boolean definition necessary for C
#include <stdbool.h>

/* ---- Defines ---- */
typedef struct Node {
  int   x;
  int   y;
  bool  closed;
  int   distance; // The length of the path up to this point, this.last.distance + 1
  int   guess;    // The heuristic, usually Manhattan to goal with no walls
  int   score;    //The guess + distance (depending on algo) (lower is better)
  Node *last;     //The parent node, the node from which we discovered this node
} Node;

// A node used in the Flood Fill sub-algorithm
typedef struct FFNode {
  bool closed;
  int x;
  int y;
} FFNode;

// Null pointer
#define NULL ((void *)0)

#define MAZE_SIZE          10
// 0 - Basic A*
// 1 - Hallway following - if there is only a single open node adjacent to the robot, we go there, regardless of score
// 2 - Greedy heuristic - when calculating score, we value getting close to the center 5x more than staying close to
//                        the start (like a string pulled taunt from two sides, the shortest path values both equally)
// 3 - Greedy heuristic and hallway following
#define MAPPING_MODE       3

// This is the midpoint of the maze
// Okay. The heuristic that A* uses is taxi-cab distance to the pin in the middle of
//  the 4x4 goal in the middle of the maze.
//  This pin is located at (4.5, 4.5) (since we've 0-indexed node locations)
//  However, the distance from the center of any node to 4.5, 4.5, by taxi-cab distance is a whole number
//  To avoid floating point math (I don't like floating point), we multiply by 10
//  (Everywhere else, distances, scores, and node locations are absolute, we only multiple by 10 in h())
#define END_X              45
#define END_Y              45

// Used as parameters to the motor functions
enum turning_direction_t {LEFT = -1, RIGHT = 1};

enum cardinal_t {NORTH, EAST, SOUTH, WEST};

/* ---- User Variables ---- */
// The maze as a 2D array
// maze[Y][X][1: vertical; 0: horizontal]
// And the number represents the confidence
// So 0 is we have no idea, and positive is wall, and negative is no wall
int maze[11][11][2];

// The list of nodes
// Sorted such that closed nodes are 0..closedNodes-1
// And nodes after that increase in score
// This is the canonical list of nodes, order doesn't matter
// All other nodes are pointers into nodesStatic
Node nodesStatic[MAZE_SIZE * MAZE_SIZE];
// A*'s working list of nodes. 0..closedNodes-1 are closed, and closedNodes..numNodes-1 are open
Node *nodes[MAZE_SIZE * MAZE_SIZE];
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
    direction = NORTH;
  }else if (adjNode->y - 1 == robot.y) {
    direction = SOUTH;
  }else if (adjNode->x == robot.x && adjNode->y == robot.y) {
    log("WARN: moveRobot called with current location");
    return;
  }else {
    log("AAAAahaaahhhah");
    direction = NORTH;
  }

  logf("Spinning to direction #%d\n", direction);
  turnTo(direction);

  moveForwardOneSquare();

  robot.x = adjNode->x;
  robot.y = adjNode->y;
}

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

/** A* algorithm **/

//Heuristic function
int h(int x, int y) {
  // See comment above the definition of END_X and END_Y
  // Since this is taxi-cab distance, the distance is always a whole number
  // To avoid introducing floating-point math, we multiply by 10 first and divide by 10 later
  return (abs(END_X - (x * 10)) + abs(END_Y - (y * 10))) / 10;
}

void insertAt(Node **arr, int len, int i, Node *n) {
  for (int j = len; j > i; j--) {
    arr[j] = arr[j - 1];
  }
  arr[i] = n;
}

void removeAt(Node **arr, int len, int i) {
  for (int j = i; j < len - 1; j++) {
    arr[j] = arr[j + 1];
  }
}

// Run Flood Fill at a node to see if it's possible to get from that node to the goal
// Returns true if it's possible, false otherwise
bool ff (Node *testingNode) {
  FFNode ffnodes[MAZE_SIZE * MAZE_SIZE];
  int ffnodeCount = 0;
  int openFFNodeCount = 0;

  FFNode* currentFFNode = &ffnodes[ffnodeCount];
  currentFFNode->x = testingNode->x;
  currentFFNode->y = testingNode->y;
  currentFFNode->closed = false;
  ffnodeCount++;
  openFFNodeCount++;

  // while we have open ffnodes
  while (openFFNodeCount > 0) {
    // loop over all open ffnodes
    for (int i = ffnodeCount - 1; i >= 0; i--) {
      if (!ffnodes[i].closed) {
        currentFFNode = &ffnodes[i];

        // If this node is at the goal, then return true
        if (isGoal(currentFFNode->x, currentFFNode->y)) {
          return true;
        }

        bool isBlockedNorth = false;
        bool isBlockedSouth = false;
        bool isBlockedEast = false;
        bool isBlockedWest = false;

        // Loop over all pre-existing ffn nodes, we don't want or need to cover them
        for (int j = 0; j < ffnodeCount; j++) {
          // North
          if (currentFFNode->x == ffnodes[j].x && currentFFNode->y - 1 == ffnodes[j].y) {
            isBlockedNorth = true;
          }
          // East
          if (currentFFNode->x + 1 == ffnodes[j].x && currentFFNode->y == ffnodes[j].y) {
            isBlockedEast = true;
          }
          // South
          if (currentFFNode->x == ffnodes[j].x && currentFFNode->y + 1 == ffnodes[j].y) {
            isBlockedSouth = true;
          }
          // West
          if (currentFFNode->x - 1 == ffnodes[j].x && currentFFNode->y == ffnodes[j].y) {
            isBlockedWest = true;
          }
        }

        // Loop over all closed A* nodes
        for (int j = 0; j < closedNodes; j++) {
          // North
          if (currentFFNode->x == nodes[j]->x && currentFFNode->y - 1 == nodes[j]->y) {
            isBlockedNorth = true;
          }
          // East
          if (currentFFNode->x + 1 == nodes[j]->x && currentFFNode->y == nodes[j]->y) {
            isBlockedEast = true;
          }
          // South
          if (currentFFNode->x == nodes[j]->x && currentFFNode->y + 1 == nodes[j]->y) {
            isBlockedSouth = true;
          }
          // West
          if (currentFFNode->x - 1 == nodes[j]->x && currentFFNode->y == nodes[j]->y) {
            isBlockedWest = true;
          }
        }

        // We can't go outside the board
        if (currentFFNode->x == 0) {
          isBlockedWest = true;
        }
        if (currentFFNode->x == MAZE_SIZE - 1) {
          isBlockedEast = true;
        }
        if (currentFFNode->y == 0) {
          isBlockedNorth = true;
        }
        if (currentFFNode->y == MAZE_SIZE - 1) {
          isBlockedSouth = true;
        }

        // And we can't go through walls
        // Remember, walls are 0 if we haven't seen them before, and we want to treat un-seen walls as open
        // So we're blocked only if know there's a wall there; > 0
        if (maze[currentFFNode->y][currentFFNode->x][0] > 0) {
          isBlockedNorth = true;
        }
        if (maze[currentFFNode->y][currentFFNode->x + 1][1] > 0) {
          isBlockedEast = true;
        }
        if (maze[currentFFNode->y + 1][currentFFNode->x][0] > 0) {
          isBlockedSouth = true;
        }
        if (maze[currentFFNode->y][currentFFNode->x][1] > 0) {
          isBlockedWest = true;
        }

        // If we can, create an ffnode in each direction
        if (!isBlockedNorth) {
          ffnodes[ffnodeCount].closed = false;
          ffnodes[ffnodeCount].x = currentFFNode->x;
          ffnodes[ffnodeCount].y = currentFFNode->y - 1;
          ffnodeCount++;
          openFFNodeCount++;
        }
        if (!isBlockedEast) {
          ffnodes[ffnodeCount].closed = false;
          ffnodes[ffnodeCount].x = currentFFNode->x + 1;
          ffnodes[ffnodeCount].y = currentFFNode->y;
          ffnodeCount++;
          openFFNodeCount++;
        }
        if (!isBlockedSouth) {
          ffnodes[ffnodeCount].closed = false;
          ffnodes[ffnodeCount].x = currentFFNode->x;
          ffnodes[ffnodeCount].y = currentFFNode->y + 1;
          ffnodeCount++;
          openFFNodeCount++;
        }
        if (!isBlockedWest) {
          ffnodes[ffnodeCount].closed = false;
          ffnodes[ffnodeCount].x = currentFFNode->x - 1;
          ffnodes[ffnodeCount].y = currentFFNode->y;
          ffnodeCount++;
          openFFNodeCount++;
        }

        //Close the current ffn node
        currentFFNode->closed = true;
        openFFNodeCount--;
      }
    }
  }

  // If we finished the flood fill without finding the goal, return false
  return false;
}

void addNodeIfNotExists(int x, int y) {
  //Check if we've seen the node before
  for (int i = 0; i < numNodes; i++) {
    if (nodes[i]->x == x && nodes[i]->y == y) {
      //We don't have to ever update the score on a node
      //Because our heuristic is nice (just distance)
      return;
    }
  }

  Node *n = &nodesStatic[numNodes];

  n->x = x;
  n->y = y;
  n->last  = current;
  n->guess = h(x, y);
  if (current == NULL) {
    n->distance = 0;
  }else {
    n->distance = current->distance + 1;
  }
  if (mappingMode == 0 || mappingMode == 1) {
    n->score = n->distance + n->guess;
  } else if (mappingMode == 2 || mappingMode == 3) {
    n->score = n->distance + n->guess * 10;
  }

  //Sort the new node in
  for (int i = closedNodes; i < numNodes; i++) {
    if (nodes[i]->score > n->score) {
      //nodes.splice(i, 0, n);
      //arr, length, index to insert, item
      insertAt(nodes, numNodes, i, n);
      numNodes++;
      return;
    }
  }
  //Otherwise, add the node to the end
  //  nodes.splice(nodes.length, 0, n);
  //  insertAt(nodes, numNodes, numNodes, n);
  nodes[numNodes] = n;
  numNodes++;
}

// Check if we are in any of the four finish squares
int isGoal(int x, int y) {
  return (
    (x == 4 && y == 4) ||
    (x == 4 && y == 5) ||
    (x == 5 && y == 4) ||
    (x == 5 && y == 5)
  );
}

// Once we've solved the maze, converts node tree into `mainPath` (a list)
void createPath() {
  mainPath[0] = current;
  Node *previous = current;

  pathLength = 1;
  while (previous->distance > 0) {
    insertAt(mainPath, pathLength, 0, previous->last);
    pathLength++;
    previous = previous->last;
  }
}

// Most of the time, goal is just nodes[closedNodes]
void updateGoal() {
  logf("Updating goal; closedNodes: %d, numNodes: %d,\n", closedNodes, numNodes);
  if (closedNodes >= numNodes) {
    return;
  }

  if (mappingMode == 0 || mappingMode == 2) {
    goal = nodes[closedNodes];
  } else {
    // Hallway following exception logic
    // If there's one adjacent node, go there, otherwise go to A*
    Node *adjacentNode = NULL;

    for (int i = 0; i < numNodes; i++) {
      // If it's open and adjacent
      if (nodes[i]->last == current && !nodes[i]->closed) {
        // If we already have one
        if (adjacentNode != NULL) {
          // Then we want to ignore it
          adjacentNode = NULL;
          break;
        } else {
          adjacentNode = nodes[i];
        }
      }
    }

    if (adjacentNode == NULL) {
      goal = nodes[closedNodes];
    } else {
      goal = adjacentNode;
    }
  }
}

// Creates a path from the current node to the goal node
// By backtracking through the nodes that we've seen
void createBackPath() {
  log("Creating backpath\n");

  //finding backPath only needs to run once
  // Create a path.
  //      Find a common parent of current and goal
  backPath[0] = current;
  backPath[1] = goal;

  // memcpy(&backPath[0], current, sizeof(Node));
  // memcpy(&backPath[1], goal, sizeof(Node));
  int numMid = 1; //The number of nodes that we go up before going back down.

  backPathLength = 2;

  // 1
  const int genDiff = abs(current->distance - goal->distance);

  for (int i = 0; i < genDiff; i++) {
    if (current->distance > goal->distance) {
      // parentCur.push(parentCur[parentCur.length-1].last);
      // backPath.splice(numMid, 0, backPath[numMid - 1].last);
      insertAt(backPath, backPathLength, numMid, backPath[numMid - 1]->last);
      numMid++;
      backPathLength++;
    } else {
      // parentGoal.unshift(parentGoal[0].last);
      // backPath.splice(numMid, 0, backPath[numMid].last);
      insertAt(backPath, backPathLength, numMid, backPath[numMid]->last);
      backPathLength++;
    }
  }
  //Walk back up the tree until they're the same
  while (backPath[numMid - 1] != backPath[numMid]) {
    // parentCur.push(parentCur[parentCur.length-1].last);
    // backPath.splice(numMid, 0, backPath[numMid-1].last);
    insertAt(backPath, backPathLength, numMid, backPath[numMid - 1]->last);
    numMid++;
    backPathLength++;

    // parentGoal.unshift(parentGoal[0].last);
    // backPath.splice(numMid, 0, backPath[numMid].last);
    insertAt(backPath, backPathLength, numMid, backPath[numMid]->last);
    backPathLength++;
  }

  //Remove the duplicated shared parent, that they both pushed
  // backPath.slice(numMid, 0, backPath[numMid].last);
  removeAt(backPath, backPathLength, numMid);
  backPathLength--;
}

// Moves us from current to goal, along a calculated backPath
// Blocks until we're there
void moveToGoal() {
  // Move back along path
  // backPath is a list of contiguous nodes. First is current, last is goal

  logf("Moving to goal, backPath.length is %d\n", backPathLength);
  while (backPathLength > 0) {
    moveRobot(backPath[0]);
    removeAt(backPath, backPathLength, 0);
    backPathLength--;
  }

  // Then we're actually done
  current = backPath[0];
  backPathLength = 0;
}

void closeNode (Node* node) {
  //Close the current node
  node->closed = true;
  //If this wasn't the next node scheduled to be closed, we need to move it to the closed section of the list
  if (nodes[closedNodes] != node) {
    Node *lastValue = nodes[closedNodes];
    for (int i = closedNodes + 1; i < numNodes; i++) {
      Node *tmp = nodes[i];
      nodes[i]  = lastValue;
      lastValue = tmp;
      if (tmp == node) {
        //Then we're done
        break;
      }
    }
    nodes[closedNodes] = node;
  }
  closedNodes++;
}

/* ---- Init ---- */
// Initializes maze state and variables
void init_maze() {
  // Initialize current
  addNodeIfNotExists(0, 0);
  current = nodes[0];

  // We need to make sure that we've checked the square behind us before starting
  // (Normally, this isn't needed since the square
  //  behind us is the square we just came from)
  updateMaze(); // Update the maze while we're pointed North,
  turnTo(SOUTH); // Spin South
  updateMaze(); // Update the maze again
}