// Uh so the current plan is that the sensor values are read at random times
// But then we also implement blocking functions for moving in a direction, or
// checking if there's a wall in a given direction. That can do what they want

// TODO: Function to move 1 square in any direction
// TODO: Function to check if there's a wall in a given direction
// TODO: Check DIP switches for maze algo
// TODO: Once we've solved the maze, go back to the beginning, and run it
// TODO: Any number of bugs arising from the fact that I don't know C

#include <Wire.h>
#include <Encoder.h>
// Include the VL6180X_WE sensor library
#include <VL6180X_WE.h>

/* ---- Defines ---- */
#define MAZE_SIZE          10
// 0 - Basic A*; 1 - A*, but tries to go center; 2 - A²*, 3 - Augment A* and extra huristic
#define MAPPING_MODE       3

#define END_X              5
#define END_Y              5

// Address of the MUX
#define TCA_ADDR           0x70

#define VL6180X_ADDRESS    0x29

// Motor Pins
// Each motor has 2 inputs, and turns according to a difference between them
#define MOTOR_RIGHT_IN1    35
#define MOTOR_RIGHT_IN2    36
#define MOTOR_LEFT_IN1     37
#define MOTOR_LEFT_IN2     38

// Encoder pins
#define ENCODER_RIGHT_1 33
#define ENCODER_RIGHT_2 34
#define ENCODER_LEFT_1 23
#define ENCODER_LEFT_2 22

// Used as parameters to the motor functions

enum direction {LEFT, RIGHT, FORWARD, BACKWARD, OFF};

enum cardinal {NORTH, EAST, SOUTH, WEST};

/* ---- User Variables ---- */
VL6180x sensor(VL6180X_ADDRESS);

Encoder rightEncoder(ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder(ENCODER_LEFT_1, ENCODER_LEFT_2);

// A maze
// Uh
// maze[Y][X][1: vertical; 0: horizontal]
// And the number represents the confidence
// So 0 is we have no idea, and positive is wall, and negative is no wall
int maze[11][11][2];

// Aahhhah
typedef struct Node Node;

struct Node {
    int   x;
    int   y;
    int   closed;   //Boolean but this is C :(
    int   distance; // The length of the path up to this point, this.last.distance + 1
    int   guess;    // The heuristic, usually Manhattan to goal with no walls
    int   score;    //The guess + distance (depending on algo) (lower is better)
    Node *last;     //The parent node, the node from which we discovered this node
};

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

//let robot (Needs to keep track of inter-tile location, e.g. encoder counts)
Node *current;                         //The node the robot is currently at (or that A* thinks we're at)
Node *goal;                            //The current goal node
//let state (Might be needed to track what we're doing. Might be fine a while true loop and blocking functions)
Node *backPath[MAZE_SIZE * MAZE_SIZE]; // Used to store the path along which we backtrack to get to goal
int backPathLength = 0;

// TODO: init current, create a node, put it in nodesStatic and nodes and current and make numNodes = 1

// The main path, once we've solved the maze, the route we use
Node *mainPath[MAZE_SIZE * MAZE_SIZE];
int pathLength = 0;

struct RobotStruct {
    int   x;
    int   y;
    // Node *currentNode; //The node the robot is currently in (physically ±1 square)
    // int side; //What side of the current node are we
		cardinal facing;
};                     // Create a robot with an
struct RobotStruct theRobot = {
    0, 0, NORTH
};
struct RobotStruct *robot = &theRobot;


// Knows robot position
// Consumes the backpath
// Blocks until it's done
// Starts and ends rounded to the nearest node
// void moveAlongBackpath();

/* ---- User Functions ---- */

// TODO: Adjust for what sensor is being used
// Function to select a channel on the I2C MUX
void select_sensor(uint8_t i) {
    if (i > 7) {
        return;
    }
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

// The number of the sensor we want to read
// Return a distance in uh millimeters
// Between ~20 (about one inch away) and 180mm ~7inches
// For the short range sensors
// 0 -- right back
// 1 -- right front
// 2 -- left back
// 3 -- left front
int read_sensor(uint8_t i) {
    select_sensor(i);

    return((int)sensor.getDistance());
}

// Moves the robot to a node which is adjacent to the current node
void moveRobot(Node *adjNode) {
	// Figure out what direction the node is in
	cardinal direction;
	if (adjNode->x + 1 == robot->x) {
		direction = EAST;
	}else if (adjNode->x - 1 == robot->x) {
		direction = WEST;
	}else if (adjNode->y + 1 == robot->y) {
		direction = NORTH;
	}else if (adjNode->y - 1 == robot->y) {
		direction = SOUTH;
	}else {
		Serial.println("AAAAahaaahhhah");
        direction = NORTH;
	}

	spinTo(direction);

    moveForwardOneSquare();

    robot->x = adjNode->x;
    robot->y = adjNode->y;
}

//Causes the robot to do a point-turn to spin to the desired location
void spinTo(cardinal direction) {
	if (robot->facing == direction) {
		// return, we're done!
	}else if (
		(robot->facing == NORTH && direction == EAST) ||
		(robot->facing == EAST && direction == SOUTH) ||
		(robot->facing == SOUTH && direction == WEST) ||
		(robot->facing == WEST && direction == NORTH)
	) {
		//Rotate 90º Right
        rotate90(RIGHT);
	}else if (
		(robot->facing == NORTH && direction == WEST) ||
		(robot->facing == WEST && direction == SOUTH) ||
		(robot->facing == SOUTH && direction == EAST) ||
		(robot->facing == EAST && direction == NORTH)
	) {
		//Rotate 90º Left
        rotate90(LEFT);
	}else if (
		(robot->facing == NORTH && direction == SOUTH) ||
		(robot->facing == SOUTH && direction == NORTH) ||
		(robot->facing == EAST && direction == WEST) ||
		(robot->facing == WEST && direction == SOUTH)
	) {
		// Rotate 180º
        rotate90(RIGHT);
        rotate90(RIGHT);
	}

	robot->facing = direction;
}

// method that tells the robot to move or stop
void setMotors(direction direction, int power) {
	if (direction == RIGHT) {
		analogWrite(MOTOR_RIGHT_IN1, power);
		analogWrite(MOTOR_RIGHT_IN2, 0);
		analogWrite(MOTOR_LEFT_IN1, 0);
		analogWrite(MOTOR_LEFT_IN2, power);
	}else if (direction == LEFT) {
		analogWrite(MOTOR_RIGHT_IN1, 0);
		analogWrite(MOTOR_RIGHT_IN2, power);
		analogWrite(MOTOR_LEFT_IN1, power);
		analogWrite(MOTOR_LEFT_IN2, 0);
    }else if (direction == FORWARD) {
		analogWrite(MOTOR_RIGHT_IN1, power);
		analogWrite(MOTOR_RIGHT_IN2, 0);
		analogWrite(MOTOR_LEFT_IN1, power);
		analogWrite(MOTOR_LEFT_IN2, 0);
	}else if (direction == BACKWARD) {
		analogWrite(MOTOR_RIGHT_IN1, 0);
		analogWrite(MOTOR_RIGHT_IN2, power);
		analogWrite(MOTOR_LEFT_IN1, 0);
		analogWrite(MOTOR_LEFT_IN2, power);
	}else {
		analogWrite(MOTOR_RIGHT_IN1, 0);
		analogWrite(MOTOR_RIGHT_IN2, 0);
		analogWrite(MOTOR_LEFT_IN1, 0);
		analogWrite(MOTOR_LEFT_IN2, 0);
	}
}

// TODO
// Rotate 90 degrees
// Takes a direction, either LEFT or RIGHT
void rotate90(direction direction) {
    
}

// TODO
// Moves the robot forward 1 square in the direction the robot is currently facing
void moveForwardOneSquare()
{

}

// TODO
// "rounds" the current robot position to be perpendicular with the wall, by doing a point turn
// Doesn't do anything if we're within tolerance
// NOTE: this is a blocking function. It will not return until the robot is parallel with the walls!
void alignWithWall() {

}

/** A* algorithm **/
// TODO: Maybe rewrite???

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
    n->distance = current->distance + 1;
    if (MAPPING_MODE == 0) {
        n->score = n->distance + n->guess;
    }
    else if (MAPPING_MODE == 1 || MAPPING_MODE == 3) {
        n->score = n->distance + n->guess * 10;
    }

    /*//Just assertions
     * assertEq(nodes[closedNodes].closed, false, "AHHHH");
     * if (closedNodes < nodes.length - 1) {
     *  assertEq(nodes[closedNodes + 1].closed, false, "B");
     * }else {
     *  console.assert(false, "testing");
     * }
     * if (closedNodes >= 1) {
     *  assertEq(nodes[closedNodes - 1].closed, true, "B");
     * }*/

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
    //	nodes.splice(nodes.length, 0, n);
    //	insertAt(nodes, numNodes, numNodes, n);
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
void updateMaze() {
    // Scan the maze

    // Check right
    sensorRightA = read_sensor(0); // right back
    // sensorRightB = read_sensor(1); // right front

    // Check Left
    sensorLeftA = read_sensor(2); // left back
    // sensorLeftB = read_sensor(3); // left front

    // Then there's no wall on our right
    if (robot.facing == NORTH) {
        // Update the wall to our right, that is, East of us.
        maze[current->y][current->x + 1][1] += sensorRightA < 255 ? 1 : -1;
        // Left is west
        maze[current->y][current->x][1] += sensorLeftA < 255 ? 1 : -1;
    }else if (robot.facing == EAST) {
        // Right is south
        maze[current->y + 1][current->x][0] += sensorRightA < 255 ? 1 : -1;
        // Left is north
        maze[current->y][current->x][0] += sensorLeftA < 255 ? 1 : -1;
    }else if (robot.facing == SOUTH) {
        // Right is west
        maze[current->y][current->x][1] += sensorRightA < 255 ? 1 : -1;
        // Left is east
        maze[current->y][current->x + 1][1] += sensorLeftA < 255 ? 1 : -1;
    }else if (robot.facing == WEST) {
        // Right is north
        maze[current->y][current->x][0] += sensorRightA < 255 ? 1 : -1;
        // Left is south
        maze[current->y + 1][current->x][0] += sensorLeftA < 255 ? 1 : -1;
    }

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

void updateGoal() {
    if (closedNodes >= numNodes) {
        return;
    }

    if (MAPPING_MODE == 1 || MAPPING_MODE == 2) {
        goal = nodes[closedNodes];
    }
    else {
        //If there's one adjacent node, go there, otherwise go to A*
        Node *adjacentNode = NULL;

        for (int i = 0; i < numNodes; i++) {
            //If it's open and adjacent
            if (nodes[i]->last == current && !nodes[i]->closed) {
                // If we already have one
                if (adjacentNode != NULL) {
                    //Then we want to ignore it
                    adjacentNode = NULL;
                }
                else {
                    adjacentNode = nodes[i];
                }
            }
        }

        if (adjacentNode == NULL) {
            goal = nodes[closedNodes];
        }
        else {
            goal = adjacentNode;
        }
    }
}

void createBackPath() {
    //finding backPath only needs to run once
    // Create a path.
    //      Find a common parent of current and goal
    //	backPath[0] = current;
    //	backPath[1] = goal;
    memcpy(&backPath[0], current, sizeof(Node));
    memcpy(&backPath[1], goal, sizeof(Node));
    int numMid = 1; //The number of nodes that we go up before going back down.

    backPathLength = 2;

    const int genDiff = abs(current->distance - goal->distance);

    for (int i = 0; i < genDiff; i++) {
        if (current->distance > goal->distance) {
            // parentCur.push(parentCur[parentCur.length-1].last);
            // backPath.splice(numMid, 0, backPath[numMid - 1].last);
            insertAt(backPath, backPathLength, numMid, backPath[numMid - 1]->last);
            numMid++;
            backPathLength++;
        }
        else {
            // parentGoal.unshift(parentGoal[0].last);
            // backPath.splice(numMid, 0, backPath[numMid].last);
            insertAt(backPath, backPathLength, numMid, backPath[numMid]->last);
            backPathLength++;
        }
    }
    //Walk back up the tree until they're the same
    while (backPath[numMid - 1]->last != backPath[numMid]->last) {
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

void moveToGoal() {
    // Move back along path
    //      To move one node, move ? per tile
    // backPath is a list of contiguous nodes. First is current, last is goal

    //TODO: Uh spin some motors or something
    // Probably refactor this to call a blocking move function,
    // And continue once we're there, instead of checking if we're there

//    if (backPath[1].x == (robot.x - SQUARE_SIZE/2)/SQUARE_SIZE &&
//        backPath[1].y == (robot.y - SQUARE_SIZE/2)/SQUARE_SIZE) {
//        //Then we've made it
//        backPath.shift(); //Remove the first
//    }
    // if (1 == 1 /*We're there, by some metric*/) {
    //     //Then we've made it
    //     removeAt(backPath, backPathLength, 0);
    //     backPathLength--;
    // }
    //
    // if (backPathLength == 1) {
    //     //Then we're actually done,
    //
    //     current = backPath[0];
    //     backPathLength = 0;
    // }

    while (backPathLength > 1) {
        moveRobot(backPath[0]);
        removeAt(backPath, backPathLength, 0);
        backPathLength--;
    }
    // Then we're actually done,
    current = backPath[0];
    backPathLength = 0;
}

// TODO: Every subgroup must set up the sensors they need to use in here
/* ---- SETUP ---- */
void setup(void) {
    // Read DIP values
    // const OPT_A = HAL_GPIO_ReadPin(OPT_A_GPIO_Port, OPT_A_Pin);
    // const OPT_B = HAL_GPIO_ReadPin(OPT_B_GPIO_Port, OPT_B_Pin);
    // const OPT_C = HAL_GPIO_ReadPin(OPT_C_GPIO_Port, OPT_C_Pin);
    // const OPT_D = HAL_GPIO_ReadPin(OPT_D_GPIO_Port, OPT_D_Pin);

    // put your setup code here, to run once:
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);

    Serial.begin(115200);

    // Setup I2C
    Wire.begin();
    // We use a single `sensor` to communicate with all sensors
    for (uint8_t i = 0; i < 8; i++) {
        select_sensor(i);

        if (sensor.VL6180xInit() != 0) {
            Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
        }
    }

    // Write 0s to all motors
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, 0);
    analogWrite(MOTOR_LEFT_IN1, 0);
    analogWrite(MOTOR_LEFT_IN2, 0);

    updateMaze();
    updateGoal();

    // rotate90(RIGHT);
    //
    // delay(100);
    //
    // moveForwardOneSquare();
    //
    // delay(100);
    //
    // alignWithWall();
}

/* ---- MAIN ---- */
void loop(void) {
    // program to test moving forward on:
    moveForwardOneSquare();
    delay(100);
    moveForwardOneSquare();
    moveForwardOneSquare();
    delay(999999999);

    // Ignore everything behind here for now

    //Then we've solved the maze
    /*if (pathLength > 0) {
        delay(10);
        //TODO: Run the maze in reverse, then forwards
        for (int i = pathLength - 1; i >= 0; i--) {
            moveRobot(mainPath[i]);
        }
        for (int i = 0; i < pathLength; i++) {
            moveRobot(mainPath[i]);
        }
    }
    else {
        createBackPath();
        moveToGoal();

            //Update where we are
            current = goal;

            //Check if we're done
            if (checkDone()) {
                createPath();
            }
            else {
                //Update our knowledge of the maze, from the current tile
                // i.e. scan around us
                updateMaze();

                //Close the current node
                current->closed = 1;
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

                // Update goal
                //  get a new goal
                updateGoal();
            }
        // }
            //Move to previously discovered `goal` tile
            // if (backPathLength > 0) {
            createBackPath();
            // }
            moveToGoal();
    }*/
		// alignWithWall();

    // rotate90(RIGHT);

    // delay(100);

    // moveForwardOneSquare();

    // delay(100);

    // alignWithWall();
}
