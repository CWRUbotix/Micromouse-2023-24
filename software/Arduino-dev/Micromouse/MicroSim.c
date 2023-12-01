/* --- Includes --- */
// Simulator Functions
#include "API.c"
// A* Algorithm
// #include "Micro.c"
// FloodFill Algorithm
#include "Converted.c"

/* ---- Defines ---- */
// maze[Y][X][1: vertical; 0: horizontal]
// Normal X vs Y graph
// Starting peg in bottom left

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
    maze[current->x + 1][current->y][1] = wallRight() ? 1 : -1;
    if(wallRight() == 1)
      setWall(current->x, current->y, 'e');
    // Left is west
    maze[current->x][current->y][1] = wallLeft()  ? 1 : -1;
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'w');
    // In front of us, north
    maze[current->x][current->y + 1][0] = wallFront() ? 1 : -1;
    if(wallFront() == 1)
      setWall(current->x, current->y, 'n');

  }else if (robot.facing == EAST) {
    // Right is south
    maze[current->x][current->y][0] = wallRight() ? 1 : -1;
    if(wallRight() == 1)
      setWall(current->x, current->y, 's');
    // Left is north
    maze[current->x][current->y + 1][0] = wallLeft() ? 1 : -1;
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'n');
    // In front of us, east
    maze[current->x + 1][current->y][1] = wallFront() ? 1 : -1;
    if(wallFront() == 1)
      setWall(current->x, current->y, 'e');

  }else if (robot.facing == SOUTH) {
    // Right is west
    maze[current->x][current->y][1] = wallRight() ? 1 : -1;
    if(wallRight() == 1)
      setWall(current->x, current->y, 'w');
    // Left is east
    maze[current->x + 1][current->y][1] = wallLeft() ? 1 : -1;
    if(wallLeft() == 1)
      setWall(current->x, current->y, 'e');
    // In front of us, south
    maze[current->x][current->y][0] = wallFront() ? 1 : -1;
    if(wallFront() == 1)
      setWall(current->x, current->y, 's');

  }else if (robot.facing == WEST) {
    // Right is north
    maze[current->x][current->y + 1][0] = wallRight() ? 1 : -1;
    if(wallRight() == 1)
      setWall(current->x, current->y, 'n');
    // Left is south
    maze[current->x][current->y][0] = wallLeft() ? 1 : -1;
    if(wallLeft() == 1)
      setWall(current->x, current->y, 's');
    // In front of us, west
    maze[current->x][current->y][1] = wallFront() ? 1 : -1;
    if(wallFront() == 1)
      setWall(current->x, current->y, 'w');
  }
  
  interimLogic();
}

/* ---- MAIN ---- */
int main(int argc, char* argv[]) {
  init_maze();
  while(1){
  // log("Robot at (x: %d, y: %d)\n", current->x, current->y);
  // Reads sensor data and updates the maze
  updateMaze();

  // Reassigns the score to all visited nodes
  for(int i = 0; i < numNodes; i++){
    Node *cell = mazeNodes[i];
    setText(cell->x, cell->y, cell->guess);
  }
  // Marks the current node as closed
  closeNode(current);
  
//   if (shouldFloodFill) {
//     logf("Running flood fill on all open nodes.\n");
//     for (int i = closedNodes; i < numNodes; i++) {
//       if (!ff(nodes[i])) {
//         logf("FF closing node x: %d, y: %d\n", nodes[i]->x, nodes[i]->y);
//         closeNode(nodes[i]);
//       }
//     }
//   }

  updateGoal(); // Figures out what node we're moving to
  logf("Creating back path to goal (x: %d, y: %d)\n", goal->x, goal->y);
  createBackPath(); // Calculates a path from current to goal
  moveToGoal(); // Moves along that path to goal
  
  if(isGoal(robot.x, robot.y)){
    logf("Maze Solved\n");
    break;
  }
  // if (isGoal(current->x, current->y)) {
  //   logln("Solving maze\n");

  //   // Then we've solved the maze
  //   // Create a maze
  //   createPath();

  //   // Run the maze in reverse, then forward
  //   // Move along path
  //   // TODO: pull into a function so that this loop method is super clean
  //   while (true) {
  //     for (int i = pathLength - 1; i >= 0; i--) {
  //       moveRobot(mainPath[i]);
  //     }
  //     for (int i = 0; i < pathLength; i++) {
  //       moveRobot(mainPath[i]);
  //     }
  //   }
  // }

  // if (numNodes == closedNodes) {
  //   logln("All nodes in maze explored\n");
  // }
  }
  return 0;
}