/* --- Includes --- */
// Simulator Functions
#include "API.c"
// A* Algorithm
#include "Micro.c"
/* ---- Defines ---- */
// Comment out this line to hide logs
// #define DEBUG

//Update the known information about the maze with what we know
// Needs to be called when we're in the middle of a tile
// Only updates the two sides, so we either need to call it once,
//  rotate the robot and call it again, or update the forward case based on the ultrasonic data
void updateMaze() {

  // One annoying edge case is when we get a value from one sensor on one side, but not from the other one
  // In this case, we increment and decrement, leaving the wall data for that wall unchanged
  if (robot.facing == NORTH) {
    // Update the wall to our right, that is, East of us.
    // If the sensor errored, that probably means there's no wall there
    maze[current->y][current->x + 1][1] = wallRight();
    if(wallRight() == 1)
      setWall(current->x, current->y, 'n');
    
    // Left is west
    maze[current->y][current->x][1] = wallLeft();
    if(wallLeft() == 1)
      setWall(current->x, current->y, 's');


    // In front of us, north
    maze[current->y][current->x][0] = wallFront();
    if(wallFront() == 1)
      setWall(current->x, current->y, 'w');

  }else if (robot.facing == EAST) {
    // Right is south
    maze[current->y + 1][current->x][0] = wallRight();
    if(wallRight() == 1)
      setWall(current->x, current->y, 'e');

    // Left is north
    maze[current->y][current->x][0] = wallLeft();
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'w');

    // In front of us, east
    maze[current->y][current->x + 1][1] = wallFront();
    if(wallFront() == 1)
      setWall(current->x, current->y, 'n');

  }else if (robot.facing == SOUTH) {
    // Right is west
    maze[current->y][current->x][1] = wallRight();
    if(wallRight() == 1)
      setWall(current->x, current->y, 'x');

    // Left is east
    maze[current->y][current->x + 1][1] = wallLeft();
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'n');

    // In front of us, south
    maze[current->y + 1][current->x][0] = wallFront();
    if(wallFront() == 1)
      setWall(current->x, current->y, 'e');

  }else if (robot.facing == WEST) {
    // Right is north
    maze[current->y][current->x][0] = wallRight();
    if(wallRight() == 1)
      setWall(current->x, current->y, 'w');

    // Left is south
    maze[current->y + 1][current->x][0] = wallLeft();
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'e');

    // In front of us, west
    maze[current->y][current->x][1] = wallFront();
    if(wallFront() == 1)
      setWall(current->x, current->y, 's');
  }

  logln("Attempting to add nodes");

  //If there's not a wall to each of our sides, add a new node there
  //North
  if (maze[current->y][current->x][0] < 0) {
    addNodeIfNotExists(current->x, current->y - 1);
    // setText(current->x, current->y + 1, current->score + '0');
  }
  //West
  if (maze[current->y][current->x][1] < 0) {
    addNodeIfNotExists(current->x - 1, current->y);
    // setText(current->x - 1, current->y, current->score + '0');
  }
  //South
  if (maze[current->y + 1][current->x][0] < 0) {
    addNodeIfNotExists(current->x, current->y + 1);
    // setText(current->x, current->y + 1, current->score + '0');
  }
  //East
  if (maze[current->y][current->x + 1][1] < 0) {
    addNodeIfNotExists(current->x + 1, current->y);
    // setText(current->x, current->y + 1, current->score + '0');
  }
}

/* ---- MAIN ---- */
void main(int argc, char* argv[]) {
  while(1){
  logln("");
  logf("Robot at (x: %d, y: %d)\n", current->x, current->y);

  // Initializing maze
  init_maze();
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

    // Then we've solved the maze
    // Create a maze
    createPath();

    // Run the maze in reverse, then forward
    // Move along path
    // TODO: pull into a function so that this loop method is super clean
    while (true) {
      for (int i = pathLength - 1; i >= 0; i--) {
        moveRobot(mainPath[i]);
      }
      for (int i = 0; i < pathLength; i++) {
        moveRobot(mainPath[i]);
      }
    }
  }

  if (numNodes == closedNodes) {
    log("All nodes in maze explored");
  }
  }
}
