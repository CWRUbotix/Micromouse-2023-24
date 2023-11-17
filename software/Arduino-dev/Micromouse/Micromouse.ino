/* --- Includes --- */
// A* Algorithm
#include "micro.c"
#include "mouse.ino"
/* ---- Defines ---- */
// Comment out this line to hide logs
// #define DEBUG

#ifdef DEBUG
  #define log(...) Serial.print(__VA_ARGS__)
  #define logln(...) Serial.println(__VA_ARGS__)
  #define logf(...) Serial.printf(__VA_ARGS__)
  #define LOGGING 1
#else
  #define log(...)
  #define logln(...)
  #define logf(...)
  #define LOGGING 0
#endif

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

  logln("Attempting to add nodes");

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

/* ---- SETUP ---- */
void setup(void) {

  // Start serial if we are logging data
  if(LOGGING){
    Serial.begin(115200);
    Serial.println("Serial ready!");
  }

  init_mouse();
  init_maze();

  pinMode(START_BUTTON, INPUT);
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
}

/* ---- MAIN ---- */
void loop(void) {
  logln("");
  logf("Robot at (x: %d, y: %d)\n", current->x, current->y);

  // Reads sensor data and updates the maze
  logln("Updating maze");
  updateMaze();

  logln("Closing current node");
  closeNode(current); // Marks the current node as closed

  if (shouldFloodFill) {
    logln("Running flood fill on all open nodes.");
    for (int i = closedNodes; i < numNodes; i++) {
      if (!ff(nodes[i])) {
        logf("FF closing node x: %d, y: %d", nodes[i]->x, nodes[i]->y);
        closeNode(nodes[i]);
      }
    }
  }

  logln("Updating goal");
  updateGoal(); // Figures out what node we're moving to
  logf("Creating back path to goal (x: %d, y: %d)\n", goal->x, goal->y);
  createBackPath(); // Calculates a path from current to goal
  logf("Moving to goal\n");
  moveToGoal(); // Moves along that path to goal


  logln("Checking done");
  if (isGoal(current->x, current->y)) {
    logln("Solving maze");
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
