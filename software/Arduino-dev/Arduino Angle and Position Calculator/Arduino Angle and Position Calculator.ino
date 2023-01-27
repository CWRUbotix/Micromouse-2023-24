// distance measurements from lidar sensors
volatile double lidarDistanceFL, lidarDistanceFR, lidarDistanceBL, lidarDistanceBR;
// separation distance between lidar sensors
const double lidarSeparation = 4;
// total width of maze
const double mazeWidth = 25.4;
// total width of the robot
const double robotWidth = 4;
// angle from "vertical" (CCW positive)
double angle = 0;
// displacement from center line (right positive)
double position = 0;

void setup() {
  // set up sensors and interrupts
  // test values in cm
  lidarDistanceFL = 10;
  lidarDistanceFR = 10;
  lidarDistanceBL = 10;
  lidarDistanceBR = 10;
}

void loop() {
  // pulse each sensor and store values
}

void getAngle() {
  // calculate angle from each side's data
  double angleL = arctan((lidarDistanceBL - lidarDistanceFL)/lidarSeparation);
  double angleR = arctan((lidarDistanceFR - lidarDistanceBR)/lidarSeparation);
  angle = (angleL + angleR) / 2;
}

void getHorizontalPosition() {
  // calculate horizontal distance to each wall
  double leftDistance = (lidarDistanceFL + lidarDistanceBL) / 2 * cos(angle);
  double rightDistance = (lidarDistanceFR + lidarDistanceBR) / 2 * cos(angle);
  position = ((rightDistance + robotWidth / 2) + (mazeWidth - robotWidth / 2 - leftDistance)) / 2;
}