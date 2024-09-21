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

// When centered, there should be 60mm in front of the ultrasonic
#define ULTRASONIC_FRONT 60

// Squares are 10in by 10in, but we work in mm. 10in = 254mm
#define SQUARE_SIZE 254

// Uncommenting this causes everything to break :/
// void abortAndFlash (void) {
//   setMotor(LEFT_MOTOR, 0);
//   setMotor(RIGHT_MOTOR, 0);

//   while (true) {
//     digitalWrite(BLUE_LED, HIGH);
//     delay(100);
//     digitalWrite(BLUE_LED, LOW);
//     delay(100);
//   }
// }

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

double ultrasonic;
double ultrasonic_running_average;
const double ultrasonic_ave_factor = 0.1;

// Magic constant that converts the time the ultrasonic takes to read the sensors into millimeters
// Based on the speed of sound
double ultrasonic_distance_factor = 0.17;

bool ultrasonic_errored;

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

bool shouldFloodFill = false;
int mappingMode = MAPPING_MODE;

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
    // abortAndFlash();
  }

  // Serial.printf("Spinning to direction #%d\n", direction);
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
  // Read the right LIDAR sensors and update their values
  back_right = lidar_sensors[1].readRange();
  back_right_errored = lidar_sensors[1].readRangeStatus() != VL6180X_ERROR_NONE || back_right > SENSOR_RANGE_MAX;
  front_right = lidar_sensors[2].readRange();
  front_right_errored = lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX;

  // Read the left LIDAR sensors and update their values
  back_left = lidar_sensors[0].readRange();
  back_left_errored = lidar_sensors[0].readRangeStatus() != VL6180X_ERROR_NONE || back_left > SENSOR_RANGE_MAX;
  front_left = lidar_sensors[3].readRange();
  front_left_errored = lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX;

  // Read front ultrasonic sensor
  digitalWrite(SONIC_TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG1, LOW);
  ultrasonic = pulseIn(SONIC_ECHO1, HIGH) * ultrasonic_distance_factor;
  ultrasonic_errored = ultrasonic > SENSOR_RANGE_MAX;
  // And keep a running average
  if (!ultrasonic_errored) {
    ultrasonic_running_average = ultrasonic * ultrasonic_ave_factor + ultrasonic_running_average * (1 - ultrasonic_ave_factor);
  }

  Serial.printf("Ultrasonic: %f (avg: %f); back_left (%d): %d, front_right (%d): %d, back_left (%d): %d, front_left (%d): %d\n", ultrasonic, ultrasonic_running_average, back_right_errored, back_right, front_right_errored, front_right, back_left_errored, back_left, front_left_errored, front_left);
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
 * @return angle given by lidar sensors
 */
double getAngle()
{
  // arctan((lidarDistanceBL - lidarDistanceFL) / lidarSeparation);
  // Average left and right sensors
  double leftAngle = -atan2(front_left - back_left, LIDAR_SEPERATION);
  double rightAngle = atan2(front_right - back_right, LIDAR_SEPERATION);
  // Serial.printf("left angle: %f\tright angle: %f; ", leftAngle * 180.0 / PI, rightAngle * 180.0 / PI);

  if ((back_left_errored || front_left_errored) && (back_right_errored || front_right_errored)) {
    // If we have no good data, assume we're going straight
    // Serial.printf("Using 0 as angle\n");
    return 0;
  } else if (back_left_errored || front_left_errored) {
    // Serial.printf("Using right angle\n");
    return rightAngle;
  } else if (back_right_errored || front_right_errored) {
    // Serial.printf("Using left angle\n");
    return leftAngle;
  } else {
    // Serial.printf("Averaging angles\n");
    return (leftAngle + rightAngle) / 2;
  }
}

/**
 * Turn robot by a given angle (in degrees)
 *
 * @param angle The angle to turn (in degrees)
 *              Positive is CCW
 *              Negative is CW
 */
void turn(double angle, turning_direction_t direction) {
  // Encoder to turn
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  // target point
  double target = angle * turnRatio;

  // Direction constant
  int dir = 1;

  if (direction == LEFT)
  {
    // Turn left
    turnEncoder = &rightEncoder;
    otherTurnEncoder = &leftEncoder;
  } else {
    // Turn right
    turnEncoder = &leftEncoder;
    otherTurnEncoder = &rightEncoder;
    dir = -1;
  }

  turnEncoder->write(0);
  otherTurnEncoder->write(0);

  // Turn right wheel backwards if left, forwards if right
  // Scale by 0.7 to compensate for over-volted motors
  setMotor(RIGHT_MOTOR, 45.125 * dir * 0.7);
  // Turn left wheel forwards if left, backwards if right
  setMotor(LEFT_MOTOR, -45.125 * dir * 0.7);

  // Turn until within margin of error

  int encoderAverage;
  do {
    encoderAverage = (turnEncoder->read() - otherTurnEncoder->read()) / 2;
  } while (encoderAverage < target - ANGLE_TOLERANCE);

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

// Rotate 90 degrees
// Takes a direction, either LEFT or RIGHT
void rotate90(turning_direction_t direction) {
  if (direction == RIGHT) {
    turn(90.0 + getAngle() * 180.0 / PI, direction);
  } else {
    turn(90.0 - getAngle() * 180.0 / PI, direction);
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

      // Update angle
      currentAngle = getAngle();

      // Update current distance
      // rev / 4560 is num revolutions (380:1 gearbox * 12 ticks per rev normally)
      // num revolutions * pi * diameter (Zach says 60mm)
      long leftRevs = leftEncoder.read();
      long rightRevs = rightEncoder.read();
      // Serial.printf("Encoder left: %d\tright: %d; ", leftRevs, rightRevs);
      currentDistance = (leftRevs + rightRevs) / 2.0 / 4560 * PI * 60.0;
      // Serial.printf("currentDistance: %f\n", currentDistance);
    }

    Serial.printf("Moving forward. current: %d, ultrasonic: %f, cond: %d, %d, %d\n", currentDistance, ultrasonic, currentDistance >= goalDistance, ultrasonic < 150, ultrasonic > 95);

    // We're also not allowed to break out of the loop (stop going forward), if we're more than 95 mm in front
    //  If we have a wall in front of us, but it's more than 95mm from the ultrasonic, then we're too far back and we need to go forward still
    // If we think we're there, but we're not, go farther
    if (currentDistance >= goalDistance && ultrasonic < 150 && ultrasonic > 95) {
      Serial.printf("Moving goalDistance forward.\n");
      // Increase goal distance such that the ultrasonic ends up ULTRASONIC_FRONT (60mm) away from the wall in front of us
      goalDistance += ultrasonic - ULTRASONIC_FRONT;
    }

    // check if currentDistance and currentAngle are within tolerance
    // For the ultrasonic, 60 is 60 mm from the wall. This is about
    // the distance when the robot is centered in the tile
    if (currentDistance >= goalDistance || (!ultrasonic_errored && ultrasonic < ULTRASONIC_FRONT)) {
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
        // Serial.printf("both errored, setting offset to 0\n");
        centerOffset = 0;
    }else if (front_left_errored) {
        // If we don't have a left value
        // (We're targeting to an offset of 0)
        // sensors are 84 mm apart, maze is 240mm wide
        centerOffset = (double)front_right - (240 - 84) / 2.0;
    }else if (front_right_errored) {
        centerOffset = (240 - 84) / 2.0 - (double)front_left;
    }

    // Serial.printf("left %d; right %d: center offset: %f\n", front_left, front_right, centerOffset);

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
    // Scale by 0.7 to compensate for over-volted motors
    int velocityLeft = (int)(-angularVelocity / 2.0 * 0.7) + velocity;
    int velocityRight = (int)(angularVelocity / 2.0 * 0.7) + velocity;

    setMotor(LEFT_MOTOR, velocityLeft);
    setMotor(RIGHT_MOTOR, velocityRight);
  }
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

// Length is the total length of the array, not the number of elements to print
void printArray(Node **arr, int startIndex, int length) {
  Serial.printf("[");
  for (int i = startIndex; i < length; i++) {
    Serial.printf("(x: %d, y: %d, dist: %d, score: %d, closed: %c)", arr[i]->x, arr[i]->y, arr[i]->distance, arr[i]->score, arr[i]->closed ? 'y' : 'n');
    if (i != length - 1) {
      Serial.printf(", ");
    }
  }
  Serial.printf("]\n");
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

    // In front of us, north
    maze[current->y][current->x][0] += ultrasonic_errored ? -1 : 1;
  }else if (robot.facing == EAST) {
    // Right is south
    maze[current->y + 1][current->x][0] += back_right_errored ? -1 : 1;
    maze[current->y + 1][current->x][0] += front_right_errored ? -1 : 1;
    // Left is north
    maze[current->y][current->x][0] += back_left_errored ? -1 : 1;
    maze[current->y][current->x][0] += front_left_errored ? -1 : 1;

    // In front of us, east
    maze[current->y][current->x + 1][1] += ultrasonic_errored ? -1 : 1;
  }else if (robot.facing == SOUTH) {
    // Right is west
    maze[current->y][current->x][1] += back_right_errored ? -1 : 1;
    maze[current->y][current->x][1] += front_right_errored ? -1 : 1;
    // Left is east
    maze[current->y][current->x + 1][1] += back_left_errored ? -1 : 1;
    maze[current->y][current->x + 1][1] += front_left_errored ? -1 : 1;

    // In front of us, south
    maze[current->y + 1][current->x][0] += ultrasonic_errored ? -1 : 1;
  }else if (robot.facing == WEST) {
    // Right is north
    maze[current->y][current->x][0] += back_right_errored ? -1 : 1;
    maze[current->y][current->x][0] += front_right_errored ? -1 : 1;
    // Left is south
    maze[current->y + 1][current->x][0] += back_left_errored ? -1 : 1;
    maze[current->y + 1][current->x][0] += front_left_errored ? -1 : 1;

    // In front of us, west
    maze[current->y][current->x][1] += ultrasonic_errored ? -1 : 1;
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
  Serial.printf("Creating backpath\n");

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

/* ---- SETUP ---- */
void setup(void) {
  // Debug led on the board itself
  pinMode(DEBUG_LED, OUTPUT);

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(DEBUG_LED, HIGH);

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
  }
  Serial.println("LiDAR sensors ready!");

  pinMode(SONIC_TRIG1, OUTPUT);
  pinMode(SONIC_ECHO1, INPUT);
  Serial.println("Ultrasonic sensor ready!");

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

  pinMode(START_BUTTON, INPUT_PULLUP);
  int t = 0;
  // Spin until start button is pressed
  // t is ms
  // On for 300 (0-300) off for 500 (300-800)
  while(!digitalRead(START_BUTTON)) {
    if (t == 0) {
      digitalWrite(YELLOW_LED, HIGH);
    }else if (t == 300) {
      digitalWrite(YELLOW_LED, LOW);
    }

    t = (t + 10) % 800;
    delay(10);
  }
  digitalWrite(YELLOW_LED, LOW);

  delay(20); // Switch "debounce"
  
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  long pressTime = millis();
  long endPressTime = pressTime;
  while (digitalRead(START_BUTTON)) {
    /* spin, waiting for button release */
    endPressTime = millis();
    if (endPressTime - pressTime > 1000) {
      digitalWrite(LED2, LOW);
    }
    if (endPressTime - pressTime > 3000) {
      digitalWrite(LED1, LOW);
    }
  }
  if (endPressTime - pressTime < 1000) {
    shouldFloodFill = true;
    mappingMode = 3;
  }else if (endPressTime - pressTime < 3000) {
    shouldFloodFill = false;
    mappingMode = 3;
  }else {
    shouldFloodFill = false;
    mappingMode = 0;
  }
  delay(500);
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // We need to make sure that we've checked the square behind us before starting
  // (Normally, this isn't needed since the square
  //  behind us is the square we just came from)
  updateMaze(); // Update the maze while we're pointed North,
  spinTo(SOUTH); // Spin South
  updateMaze(); // Update the maze again
}

/* ---- MAIN ---- */
void loop(void) {
  Serial.println("");
  Serial.printf("Robot at (x: %d, y: %d)\n", current->x, current->y);

  // Reads sensor data and updates the maze
  Serial.println("Updating maze");
  updateMaze();

  printArray(nodes, 0, numNodes);
  Serial.println("Closing current node");
  closeNode(current); // Marks the current node as closed
  printArray(nodes, 0, numNodes);

  if (shouldFloodFill) {
    Serial.println("Running flood fill on all open nodes.");
    for (int i = closedNodes; i < numNodes; i++) {
      if (!ff(nodes[i])) {
        Serial.printf("FF closing node x: %d, y: %d", nodes[i]->x, nodes[i]->y);
        closeNode(nodes[i]);
      }
    }
  }

  Serial.println("Updating goal");
  updateGoal(); // Figures out what node we're moving to
  Serial.printf("Creating back path to goal (x: %d, y: %d)\n", goal->x, goal->y);
  createBackPath(); // Calculates a path from current to goal
  printArray(backPath, 0, backPathLength);
  Serial.printf("Moving to goal\n");
  moveToGoal(); // Moves along that path to goal


  Serial.println("Checking done");
  if (isGoal(current->x, current->y)) {
    Serial.println("Solving maze");
    digitalWrite(GREEN_LED, HIGH);

    // Then we've solved the maze
    // Create a maze
    createPath();

    // Wait a second for cosmetics
    delay(1000);

    // Run the maze in reverse, then forward
    // Move along path
    // TODO: pull into a function so that this loop method is super clean
    while (true) {
      for (int i = pathLength - 1; i >= 0; i--) {
        moveRobot(mainPath[i]);
      }
      delay(700);
      for (int i = 0; i < pathLength; i++) {
        moveRobot(mainPath[i]);
      }
      delay(700);
    }
  }

  if (numNodes == closedNodes) {
    while (true) {
      digitalWrite(RED_LED, LOW);
      delay(600);
      digitalWrite(RED_LED, HIGH);
      delay(100);
    }
  }
}
