/* --- Includes --- */
// Wire.h for I2C communication with TCA chip and LiDAR
#include <Wire.h>
// Arduino standard encoder library
#include <Encoder.h>
// Adafruit library for VL6180X LiDAR sensor
#include "Adafruit_VL6180X.h"
// Pin definitions
#include "micromouse_pins_2023.h"

/* ---- Defines ---- */
typedef enum motor_t {
    LEFT_MOTOR = 0,
    RIGHT_MOTOR
} motor_t;

#define POWER_DEADBAND 6

#define LIDAR_COUNT 4
#define LIDAR_ADDR_BASE 0x50

// The physical distance between the sensors
#define LIDAR_SEPERATION 123 // 123 mm between sensors

#define ANGLE_TOLERANCE 5

const double wheelSeparation = 9.5;
const double wheelRadius = 3;
const double turnRatio = (wheelSeparation / 2.0) / wheelRadius / 360 * 380 * 12;

// The LiDAR sensors return a running average of readings,
//  so when we move past a wall, the LiDAR returns a value greater than the previous value but less than an overflow.
// (After a certain amount of time, the running average overflows the maximum and only then does the LiDAR throw a read error)
// If the LiDAR is greater than this value, we assume that it's not sensing the wall.
#define SENSOR_RANGE_MAX 110

// Squares are 10in by 10in, but we work in mm. 10in = 254mm
#define SQUARE_SIZE 254



typedef struct Node {
  int   x;
  int   y;
  bool  closed;
  int   distance; // The length of the path up to this point, this.last.distance + 1
  int   guess;    // The heuristic, usually Manhattan to goal with no walls
  int   score;    //The guess + distance (depending on algo) (lower is better)
  Node *last;     //The parent node, the node from which we discovered this node
} Node;

#define MAZE_SIZE          10
// 0 - Basic A*; 1 - A*, but tries to go center; 2 - A²*, 3 - Augment A* and extra huristic
#define MAPPING_MODE       3

// TODO: Maze goal need to be a 4x4
#define END_X              5
#define END_Y              5

// Used as parameters to the motor functions
enum turning_direction_t { LEFT, RIGHT };

enum cardinal_t {NORTH, EAST, SOUTH, WEST};

/* ---- User Variables ---- */

// GPIO pin numbers for the CS line on each LiDAR sensor
// We have another 2 pins on the board, but nothing's plugged into them
const int lidar_cs_pins[LIDAR_COUNT] = {LIDAR_CS1, LIDAR_CS2, LIDAR_CS3, LIDAR_CS4};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];

uint8_t back_right;
uint8_t front_right;

uint8_t back_left;
uint8_t front_left;

bool back_right_errored, front_right_errored, back_left_errored, front_left_errored;

Encoder rightEncoder (ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder (ENCODER_LEFT_1, ENCODER_LEFT_2);

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
  cardinal_t facing;
} robot = {
  0, 0, NORTH
};

/* ---- User Functions ---- */

// Moves the robot to a node which is adjacent to the current node
// adjNode is goal, is x: 1, y: 0
void moveRobot(Node *adjNode) {
  // Figure out what direction the node is in
  cardinal_t direction;
  if (adjNode->x + 1 == robot.x) {
    direction = WEST;
  }else if (adjNode->x - 1 == robot.x) {
    direction = EAST;
  }else if (adjNode->y + 1 == robot.y) {
    direction = NORTH;
  }else if (adjNode->y - 1 == robot.y) {
    direction = SOUTH;
  }else if (adjNode->x == robot.x && adjNode->y == robot.y) {
    Serial.println("WARN: moveRobot called with current location");
    return;
  }else {
    Serial.println("AAAAahaaahhhah");
    direction = NORTH;
    abort();
  }

  Serial.printf("Spinning to direction #%d\n", direction);
  spinTo(direction);

  moveForwardOneSquare();

  robot.x = adjNode->x;
  robot.y = adjNode->y;
}

//Causes the robot to do a point-turn to spin to the desired location
void spinTo(cardinal_t direction) {
  if (robot.facing == direction) {
    // return, we're done!
  }else if (
    (robot.facing == NORTH && direction == EAST) ||
    (robot.facing == EAST && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == WEST) ||
    (robot.facing == WEST && direction == NORTH)
  ) {
    //Rotate 90º Right
    rotate90(RIGHT);
  }else if (
    (robot.facing == NORTH && direction == WEST) ||
    (robot.facing == WEST && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == EAST) ||
    (robot.facing == EAST && direction == NORTH)
  ) {
    //Rotate 90º Left
    rotate90(LEFT);
  }else if (
    (robot.facing == NORTH && direction == SOUTH) ||
    (robot.facing == SOUTH && direction == NORTH) ||
    (robot.facing == EAST && direction == WEST) ||
    (robot.facing == WEST && direction == EAST)
  ) {
    // Rotate 180º
    rotate90(RIGHT);
    rotate90(RIGHT);
  }

  robot.facing = direction;
}

/**
 * Convert a value in range [-128..127] to a motor power value
 *
 * @param p The input power [-128..127]
 * @return Output power [0..255]
 */
uint8_t convertPower(int8_t p) {
  if (p == 0) {
    return 255;
  }
  if (p < 0) {
    if (p == -128) {
      p = -127;
    }
    p = -p;
  }
  return 255 - (((uint8_t)p) * 2);
}

/**
 * Set motor power for a specified motor
 *
 * @param m The motor to modify
 * @param power The power and direction of the motor
 *              (range: [-128..127])
 *              Positive is "forward"
 *              Negative is "backward"
 */
void setMotor (motor_t m, int power) {
  power = -power;
  if (power < -128) {
    power = -128;
  }else if (power > 127) {
    power = 127;
  }
  int m1, m2;

  // Determine motor
  if (m == LEFT_MOTOR) {
    m1 = MOTORLEFT_1;
    m2 = MOTORLEFT_2;
  }else if (m == RIGHT_MOTOR) {
    m1 = MOTORRIGHT_1;
    m2 = MOTORRIGHT_2;
  }else {
    return;
  }

  // Set power
  if (power < POWER_DEADBAND && power > -POWER_DEADBAND) {
      analogWrite(m1, 255);
      analogWrite(m2, 255);
  } else if (power > 0) {
      analogWrite(m1, 255);
      analogWrite(m2, convertPower(power));
  } else {
      analogWrite(m1, convertPower(power));
      analogWrite(m2, 255);
  }
}

void updateSensors () {
  back_right = lidar_sensors[1].readRange();
  back_right_errored = lidar_sensors[1].readRangeStatus() != VL6180X_ERROR_NONE || back_right > SENSOR_RANGE_MAX;
  front_right = lidar_sensors[2].readRange();
  front_right_errored = lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX;

  back_left = lidar_sensors[0].readRange();
  back_left_errored = lidar_sensors[0].readRangeStatus() != VL6180X_ERROR_NONE || back_left > SENSOR_RANGE_MAX;
  front_left = lidar_sensors[3].readRange();
  front_left_errored = lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX;

  Serial.printf("back_left (%d): %d, front_right (%d): %d, back_left (%d): %d, front_left (%d): %d\n", back_right_errored, back_right, front_right_errored, front_right, back_left_errored, back_left, front_left_errored, front_left);
}

// p_controller(80.0, currentAngle, 0, -127.0, 127.0);
double p_controller(double p, double current, double goal, double min, double max) {
  double out = (goal - current) * p;
  if (out > max) {
    out = max;
  }else if (out < min) {
    out = min;
  }
  return out;
}

/**
 * Get angle of robot from center line of maze path
 *
 * @return The angle of the robot from the center line of the maze path
 *         Positive is CCW
 *         Negative is CW
 */
// double getAngle() {
//   updateSensors();
//   // If left side is in range, use left measurements
//   if (!front_left_errored && !back_left_errored) {
//     return atan2(back_left - front_left, LIDAR_SEPERATION);
//   }
//   // If right side is in range, use right measurements
//   else if (!front_right_errored && !back_right_errored) {
//     return atan2(front_right - back_right, LIDAR_SEPERATION);
//   }
//   // If both sides are out of range, return 0
//   else
//     return 0;
// }

/**
 * Turn robot right (CW) by a given positive angle (in degrees) using PID
 *
 * @param angle The angle to turn (in degrees)
 */
// void turnRight(double angle) {
//   leftEncoder.write(0);

//   // Set up PID
//   double current = 0;
//   double target = angle * turnRatio;

//   // Wait until necessary angle is reached
//   while(leftEncoder.read() < target - ANGLE_TOLERANCE) {
//     current = leftEncoder.read();

//     // Update PID
//     double output = p_controller(0.6, current, target, 0.0, 255.0);

//     // Turn right wheel backwards
//     setMotor(RIGHT_MOTOR, output / -2);

//     // Turn left wheel forwards
//     setMotor(LEFT_MOTOR, output / 2);
//   }

//   // Stop both motors
//   setMotor(RIGHT_MOTOR, 0);
//   setMotor(LEFT_MOTOR, 0);
// }

void turnRight(double angle) {
  leftEncoder.write(0);

  double target = angle * turnRatio;

  // TODO: this is bad
  double output = 0.05 * target;

  // Turn right wheel backwards
  setMotor(RIGHT_MOTOR, output / -2);
  // Turn left wheel forwards
  setMotor(LEFT_MOTOR, output / 2);

  while(leftEncoder.read() < target - ANGLE_TOLERANCE);

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

void turnLeft(double angle) {
  rightEncoder.write(0);

  double target = angle * turnRatio;

  // TODO: this is bad
  double output = 0.05 * target;

  // Turn right wheel backwards
  setMotor(RIGHT_MOTOR, output / 2);
  // Turn left wheel forwards
  setMotor(LEFT_MOTOR, output / -2);

  while(rightEncoder.read() < target - ANGLE_TOLERANCE);

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

/**
 * Turn robot left (CCW) by a given positive angle (in degrees) using PID
 *
 * @param angle The angle to turn (in degrees)
 */
// void turnLeft(double angle) {
//   rightEncoder.write(0);

//   // Set up PID
//   double target = angle * turnRatio;
//   double current = 0;

//   // Wait until necessary angle is reached
//   while(rightEncoder.read() < target - ANGLE_TOLERANCE) {
//     current = rightEncoder.read();

//     // Update PID
//     double output = p_controller(1, current, target, 0.0, 255.0);

//     // Turn left wheel backwards
//     setMotor(LEFT_MOTOR, output / -2);

//     // Turn right wheel forwards
//     setMotor(RIGHT_MOTOR, output / 2);
//   }

//   // Stop both motors
//   setMotor(LEFT_MOTOR, 0);
//   setMotor(RIGHT_MOTOR, 0);
// }

/**
 * Turn robot by a given angle (in degrees)
 *
 * @param angle The angle to turn (in degrees)
 *              Positive is CCW
 *              Negative is CW
 */
void turn(double angle) {
  // If angle is negative (CW), turn right
  if(angle < 0) {
    turnRight(abs(angle));
  }
  // If angle is positive (CCW), turn left
  else {
    turnLeft(angle);
  }
  // // Get error from side lidar sensors
  // double error = getAngle();
  // // If current angle is not within tolerance, perform an adjustment turn to compensate
  // if(abs(error) > angleTolerance) {
  //   turn(error * -0.8);
  // }
}

// Rotate 90 degrees
// Takes a direction, either LEFT or RIGHT
void rotate90(turning_direction_t direction) {
  if (direction == LEFT) {
    turn(90);
  }else if (direction == RIGHT) {
    turn(-90);
  }
}

void rotate90updateRobot (turning_direction_t direction) {
  if (direction == LEFT) {
    turn(90);
  } else if (direction == RIGHT) {
    turn(-90);
  }

  if (direction == LEFT) {
    if (robot.facing == NORTH) {
      robot.facing = WEST;
    }else if (robot.facing == EAST) {
      robot.facing = NORTH;
    }else if (robot.facing == SOUTH) {
      robot.facing = EAST;
    }else if (robot.facing == WEST) {
      robot.facing = SOUTH;
    }
  } else if (direction == RIGHT) {
    if (robot.facing == NORTH) {
      robot.facing = EAST;
    }else if (robot.facing == EAST) {
      robot.facing = SOUTH;
    }else if (robot.facing == SOUTH) {
      robot.facing = WEST;
    }else if (robot.facing == WEST) {
      robot.facing = NORTH;
    }
  }
}

// Moves the robot forward 1 square in the direction the robot is currently facing
void moveForwardOneSquare() {
  // Reset encoders
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Create angle variables
  double currentAngle, angularVelocity;

  // Create speed variables
  // (currentDistance is the distance inside the current square.)
  double currentDistance = 0;
  double goalDistance = SQUARE_SIZE;
  double velocity;

  // Center variables
  double centerVelocity, centerOffset;

  // loop quickly
  while (1) {
    // update currentDistance and currentAngle
    {
      // read LiDAR
      updateSensors();

      // arctan((lidarDistanceBL - lidarDistanceFL) / lidarSeparation);
      // Average left and right sensors
      double leftAngle = -atan2(front_left - back_left, LIDAR_SEPERATION);
      double rightAngle = atan2(front_right - back_right, LIDAR_SEPERATION);
      Serial.printf("left angle: %f\tright angle: %f; ", leftAngle * 180.0 / PI, rightAngle * 180.0 / PI);

      if ((back_left_errored || front_left_errored) && (back_right_errored || front_right_errored)) {
        // If we have no good data, assume we're going straight
        Serial.printf("Using 0 as angle\n");
        currentAngle = 0;
      } else if (back_left_errored || front_left_errored) {
        Serial.printf("Using right angle\n");
        currentAngle = rightAngle;
      } else if (back_right_errored || front_right_errored) {
        Serial.printf("Using left angle\n");
        currentAngle = leftAngle;
      } else {
        Serial.printf("Averaging angles\n");
        currentAngle = (leftAngle + rightAngle) / 2;
      }

      // Update current distance
      // rev / 4560 is num revolutions (380:1 gearbox * 12 ticks per rev normally)
      // num revolutions * pi * diameter (Zach says 60mm)
      long leftRevs = leftEncoder.read();
      long rightRevs = rightEncoder.read();
      Serial.printf("Encoder left: %d\tright: %d; ", leftRevs, rightRevs);
      currentDistance = (leftRevs + rightRevs) / 2.0 / 4560 * PI * 60.0;
      Serial.printf("currentDistance: %f\n", currentDistance);
    }

    // check if currentDistance and currentAngle are within tolerance
    if (currentDistance >= goalDistance) {
      setMotor(LEFT_MOTOR, 0);
      setMotor(RIGHT_MOTOR, 0);
      break;
    }

    // How far away from the center we are
    // Right is positive
    // (ASCII art by Zach)
    // |              | <--  MAZE    |
    // |              |   CENTERLINE |
    // |              |              |
    // |      ERROR   |              |
    // |           \  |              |
    // |           |<>|              |
    // |       +-------+ <--- X ---> |
    // |       | FRONT |             |
    // |       |L  |  R|             |
    // |       |   |   |             |
    // |       | ROBOT |             |
    // |       +-------+             |
    // |       |<- R ->|             |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // | <------ MAZE WIDTH -------> |
    //
    //              MAZE WIDTH - R
    // ERROR = X - ----------------
    //                    2
    centerOffset = (double)front_right - (double)front_left;
    if (front_left_errored && front_right_errored) {
        Serial.printf("both errorered, setting offset to 0\n");
        centerOffset = 0;
    }else if (front_left_errored) {
        // If we don't have a left value
        // (We're targeting to an offset of 0)
        // sensors are 84 mm apart, maze is 240mm wide
        centerOffset = (double)front_right - (240 - 84) / 2.0;
    }else if (front_right_errored) {
        centerOffset = (240 - 84) / 2.0 - (double)front_left;
    }

    Serial.printf("left %d; right %d: center offset: %f\n", front_left, front_right, centerOffset);

    // p, current, goal, min, max
    // When the angle is 0.1, we need to bump left power by like 5, so P of 20
    // (A positive angle means that we're turned left)
    angularVelocity = p_controller(80.0, currentAngle, 0, -127.0, 127.0);

    // With a distance of 254 (one square), we've chose a P of 12.25
    //  so it saturates velocity for the majority of the distance
    velocity = p_controller(12.25, currentDistance, goalDistance, -64, 64);

    // With a center off set of 10mm, that's a velocity of 5
    centerVelocity = p_controller(0.5, centerOffset, 0, -50, 50);

    angularVelocity += centerVelocity;

    // NOTE: We're assuming that at angles close to 0, angularVelocity has a linear relationship with velocity.

    // update motor values
    int velocityLeft = (int)(-angularVelocity / 2.0) + velocity;
    int velocityRight = (int)(angularVelocity / 2.0) + velocity;

    setMotor(LEFT_MOTOR, velocityLeft);
    setMotor(RIGHT_MOTOR, velocityRight);

    // Delay?
    delay(10);
  }
}

/** A* algorithm **/

//Heuristic function
int h(int x, int y) {
  return(abs(END_X - x) + abs(END_Y - y));
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
  if (MAPPING_MODE == 0) {
    n->score = n->distance + n->guess;
  } else if (MAPPING_MODE == 1 || MAPPING_MODE == 3) {
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

int checkDone() {
  return(current->x == END_X && current->y == END_Y);
}

// Once we've solved the maze, converts node tree into `mainPath` (a list)
void createPath() {
  mainPath[0] = current;
  Node *previous = current;

  pathLength = 1;
  while (previous->last->distance >= 0) {
    // path.unshift(backtrack.last);
    insertAt(mainPath, pathLength, 0, previous->last);
    pathLength++;
    previous = previous->last;
  }
}

//Update the known information about the maze with what we know
// Needs to be called when we're in the middle of a tile
// Only updates the two sides, so we either need to call it once,
//  rotate the robot and call it again, or update the forward case based on the ultrasonic data
void updateMaze() {
  updateSensors();

  // One annoying edge case is when we get a value from one sensor on one side, but not from the other one
  // In this case, we increment and decrement, leaving the wall data for that wall unchanged

  if (robot.facing == NORTH) {
    // Update the wall to our right, that is, East of us.
    // If the sensor errored, that probably means there's no wall there
    maze[current->y][current->x + 1][1] += back_right_errored ? -1 : 1;
    maze[current->y][current->x + 1][1] += front_right_errored ? -1 : 1;
    // Left is west
    maze[current->y][current->x][1] += back_left_errored ? -1 : 1;
    maze[current->y][current->x][1] += front_left_errored ? -1 : 1;
  }else if (robot.facing == EAST) {
    // Right is south
    maze[current->y + 1][current->x][0] += back_right_errored ? -1 : 1;
    maze[current->y + 1][current->x][0] += front_right_errored ? -1 : 1;
    // Left is north
    maze[current->y][current->x][0] += back_left_errored ? -1 : 1;
    maze[current->y][current->x][0] += front_left_errored ? -1 : 1;
  }else if (robot.facing == SOUTH) {
    // Right is west
    maze[current->y][current->x][1] += back_right_errored ? -1 : 1;
    maze[current->y][current->x][1] += front_right_errored ? -1 : 1;
    // Left is east
    maze[current->y][current->x + 1][1] += back_left_errored ? -1 : 1;
    maze[current->y][current->x + 1][1] += front_left_errored ? -1 : 1;
  }else if (robot.facing == WEST) {
    // Right is north
    maze[current->y][current->x][0] += back_right_errored ? -1 : 1;
    maze[current->y][current->x][0] += front_right_errored ? -1 : 1;
    // Left is south
    maze[current->y + 1][current->x][0] += back_left_errored ? -1 : 1;
    maze[current->y + 1][current->x][0] += front_left_errored ? -1 : 1;
  }

  Serial.println("Attempting to add nodes");

  //If there's not a wall to each of our sides, add a new node there
  //North
  if (maze[current->y][current->x][0] < 0) {
    addNodeIfNotExists(current->x, current->y - 1);
  }
  //West
  if (maze[current->y][current->x][1] < 0) {
    addNodeIfNotExists(current->x - 1, current->y);
  }
  //South
  if (maze[current->y + 1][current->x][0] < 0) {
    addNodeIfNotExists(current->x, current->y + 1);
  }
  //East
  if (maze[current->y][current->x + 1][1] < 0) {
    addNodeIfNotExists(current->x + 1, current->y);
  }
}

// Most of the time, goal is just nodes[closedNodes]
void updateGoal() {
  Serial.printf("Updating goal; closedNodes: %d, numNodes: %d,\n", closedNodes, numNodes);
  if (closedNodes >= numNodes) {
    return;
  }

  if (MAPPING_MODE == 1 || MAPPING_MODE == 2) {
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
  Serial.printf("Creating backpath\n");

  //finding backPath only needs to run once
  // Create a path.
  //      Find a common parent of current and goal
  backPath[0] = current;
  backPath[1] = goal;
  Serial.printf("init backPath\n");
  // memcpy(&backPath[0], current, sizeof(Node));
  // memcpy(&backPath[1], goal, sizeof(Node));
  int numMid = 1; //The number of nodes that we go up before going back down.

  backPathLength = 2;

  const int genDiff = abs(current->distance - goal->distance);

  Serial.printf("genDiff: %d\n", genDiff);

  for (int i = 0; i < genDiff; i++) {
    Serial.printf("current distance: %d, goal distance: %d\n", current->distance, goal->distance);
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
  while (backPath[numMid - 1]->last != backPath[numMid]->last) {
    Serial.printf("Walking up tree, numMid: %d, backPathLength: %d\n", numMid, backPathLength);

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

  Serial.printf("Final remove\b");
  //Remove the duplicated shared parent, that they both pushed
  // backPath.slice(numMid, 0, backPath[numMid].last);
  removeAt(backPath, backPathLength, numMid);
  backPathLength--;
  Serial.printf("Done\n");
}

// Moves us from current to goal, along a calculated backPath
// Blocks until we're there
void moveToGoal() {
  // Move back along path
  // backPath is a list of contiguous nodes. First is current, last is goal

  Serial.printf("Moving to goal, backPath.length is %d\n", backPathLength);
  while (backPathLength > 0) {
    moveRobot(backPath[0]);
    removeAt(backPath, backPathLength, 0);
    backPathLength--;
  }

  // Then we're actually done
  current = backPath[0];
  backPathLength = 0;
}

void closeCurrentNode () {
  //Close the current node
  current->closed = true;
  //If this wasn't the next node scheduled to be closed, we need to move it to the closed section of the list
  if (nodes[closedNodes] != current) {
    Node *lastValue = nodes[closedNodes];
    for (int i = closedNodes + 1; i < numNodes; i++) {
      Node *tmp = nodes[i];
      nodes[i]  = lastValue;
      lastValue = tmp;
      if (tmp == current) {
        //Then we're done
        break;
      }
    }
    nodes[closedNodes] = current;
  }
  closedNodes++;
}

/* ---- SETUP ---- */
void setup(void) {
  // Debug led on the board itself
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Start serial
  Serial.begin(115200);
  Serial.println("Serial ready!");

  // Starts I2C on the default pins (18 (SDA), 19 (SCL))
  // (I think, I can't find docs on it)
  I2C_LIDAR.begin();
  Serial.println("I2C ready!");

  // Setup LiDARs
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    pinMode(lidar_cs_pins[i], OUTPUT);
  }
  // Disable all sensors except the first
  for (size_t i = 1; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }

  // Set address for each sensor
  // Write the CS line high (turning it on)
  // Set the address
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
    // Pass pointer to the Wire2 object since we're running on I2C bus 2
    if (!lidar_sensors[i].begin(&I2C_LIDAR)) {
      Serial.print("Failed init on sensor ");
      Serial.println(i);
    } else {
      lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
    }
    delay(10);
  }
  Serial.println("LiDAR sensors ready!");

  // Setup motors
  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_2, OUTPUT);

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
  Serial.println("Motors ready!");

  // Initialize current
  addNodeIfNotExists(0, 0);
  current = nodes[0];
}

/* ---- MAIN ---- */
void loop(void) {
  // updateSensors();

  // Reads sensor data and updates the maze
  Serial.println("Updating maze");
  updateMaze();
  Serial.println("Turning left");
  rotate90updateRobot(LEFT);
  Serial.println("Updating maze");
  updateMaze();

  Serial.println("Closing current node");
  closeCurrentNode(); // Marks the current node as closed
  Serial.println("Updating goal");
  updateGoal(); // Figures out what node we're moving to
  Serial.printf("Create back path to goal (x: %d, y: %d)\n", goal->x, goal->y);
  createBackPath(); // Calculates a path from current to goal
  Serial.printf("Moving to goal\n");
  moveToGoal(); // Moves along that path to goal

  Serial.println("Checking done");
  if (checkDone()) {
    Serial.println("Solving maze");
    // Then we've solved the maze
    // Create a maze
    createPath();
    // Run the maze in reverse, then forward
    // Move along path
    // TODO: Verify this makes sense
    // TODO: pull into a function so that this loop method is super clean
    while (true) {
      delay(10);
      //TODO: Run the maze in reverse, then forwards
      for (int i = pathLength - 1; i >= 0; i--) {
          moveRobot(mainPath[i]);
      }
      for (int i = 0; i < pathLength; i++) {
          moveRobot(mainPath[i]);
      }
    }
  }
}
