#include "Queue.h"
#include "motor.h"
#include "CalcSensor.h"
#include "Driving.h"
#include <Servo.h>
#include <Arduino.h>

//IRSENSORS: 0 is wall and 1 is space
const int IR_LEFT = 0;
const int IR_RIGHT = 1;
const int IR_FRONT = 2;
const int IR_BACK = 3;

// ENCODER: count ticks to measure distance 1 tick is .49mm and each cell is 180 mm(18 cm) so each cell is about 376 ticks
// 1 encoder is used because it's magnetic 
const int enc_pin = 5;
unsigned long tick =0; // measure ticks should restart after every cell traveled

//SERVO: 90 is natural position 0 is left 180 is right
Servo myServo;

//MOTOR: Speed increase for going straigth and should decrease for turning
const int FORWARD_SPEED = 100;
const int TURN_SPEED = 100;
const int motorFoward = 9; // pwm pins
const int motorBackward = 10; //pwm pins

// DISTANCE: 180 mm is one cell
const int CELL_DISTANCE = 180;

//MAZE info:
// each cell is 18 cm or 180mm
// in ticks(wheel revloution) about 367 ticks
// maze has 8 colums and 8 rows so 64 cells in total
// every time a new wall is meant floodfill starts again 
// middle point is 0 and goal to reach that point

// SOlution 
// 1. floodfill 
// 2. evyertime a new wall is meant floodfill start again
// 3. find the shortest path
// 4. follow the path
// 5. if there is a dead end go back to the last point and go a different way
//6.  goal is middle(4,4) and startingpoint is left side so (0,0)
//7. First travel from starting point to goal, then goal to starting point Delay of around 1 minute should occur after coming from goal to starting point
// to check if the robot is damage and is ready to go again
//8. After a couple of runs the robot should be able to go from starting point to goal and back to starting point with the shortest path

// Constants for maze dimensions
const int MAZE_ROWS = 8;
const int MAZE_COLS = 8;
const int CELL_DISTANCE_MM = 180;

// Define maze grid
int maze[MAZE_ROWS][MAZE_COLS];


// Define starting and goal positions
const int START_ROW = 0;
const int START_COL = 0;
const int GOAL_ROW = 4;
const int GOAL_COL = 4;

struct Cell {
    int row;
    int col;
};

void setup()
{
   // Sensor + encoder
   pinMode(IR_BACK,INPUT);
   pinMode(IR_FRONT,INPUT);
   pinMode(IR_LEFT,INPUT);
   pinMode(IR_RIGHT,INPUT);
   pinMode(enc_pin,INPUT);
   attachInterrupt(digitalPinToInterrupt(enc_pin), countTicks, CHANGE);

   //Motor + servo
   pinMode(motorFoward, OUTPUT);
   pinMode(motorBackward, OUTPUT);
   myServo.attach(14);// pin 14

  // Maze initialization
   initializeMaze();
   
   
   
}

void loop()
{
    // Flood-fill algorithm to calculate distances
    floodFill();
  // Find the shortest path
    findShortestPath();
    
    // Follow the shortest path
    followPath();
    
    // Return to the starting point
    
    returnToStart();
    
    // Pause and check for damage
    delayAndCheck();
}

void countTicks()
{
  tick++;
}
void initializeMaze() {
    // Initialize maze with walls and distances
    // Set all cells to initially unreachable (e.g., -1) and no walls
    for (int row = 0; row < MAZE_ROWS; row++) {
        for (int col = 0; col < MAZE_COLS; col++) {
            maze[row][col] = -1; // Set distance to unreachable
        }
    }
    
    // Set walls for the edges of the maze (adjust as needed)
    for (int row = 0; row < MAZE_ROWS; row++) {
        maze[row][0] = 1; // Left wall
        maze[row][MAZE_COLS - 1] = 1; // Right wall
    }
    for (int col = 0; col < MAZE_COLS; col++) {
        maze[0][col] = 1; // Top wall
        maze[MAZE_ROWS - 1][col] = 1; // Bottom wall
    }
}
void floodFill() {
    // Implement flood-fill algorithm
    // Should update the maze array with the distances from the starting point
    // to each cell in the maze
    // Should update after a new wall is encountered
    // Calculate distances from the starting point to each cell in the maze
    
    // Initialize queue for BFS traversal
    Queue<int> q;

    // Mark starting point as visited and enqueue it
    maze[START_ROW][START_COL] = 0; // Distance to starting point is 0
    q.enqueue(START_ROW * MAZE_COLS + START_COL); // Enqueue starting point as a single integer
    
    // Define direction arrays for movement (up, down, left, right)
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    // Perform BFS traversal
    while (!q.isEmpty()) {
        // Dequeue current cell
        int current = q.dequeue();
        int row = current / MAZE_COLS;
        int col = current % MAZE_COLS;

        // Visit all adjacent cells
        for (int i = 0; i < 4; i++) {
            int newRow = row + dx[i];
            int newCol = col + dy[i];

            // Check if the new cell is within the maze bounds and not a wall
            if (newRow >= 0 && newRow < MAZE_ROWS && newCol >= 0 && newCol < MAZE_COLS &&
                maze[newRow][newCol] == -1) {
                // Update distance to the new cell
                maze[newRow][newCol] = maze[row][col] + 1;
                // Enqueue the new cell
                q.enqueue(newRow * MAZE_COLS + newCol);
            }
        }
    }
}

const int MAX_DISTANCE = 1000; // Maximum distance value
 int distances[MAZE_ROWS][MAZE_COLS];
void findShortestPath() {
    // Create a 2D array to store the distance to each cell
    
    for (int i = 0; i < MAZE_ROWS; ++i) {
        for (int j = 0; j < MAZE_COLS; ++j) {
            distances[i][j] = MAX_DISTANCE; // Initialize distances to a large value
        }
    }

    // Start with the goal cell
    Cell startCell = {GOAL_ROW, GOAL_COL};
    distances[startCell.row][startCell.col] = 0;

    // Define direction arrays for movement (up, down, left, right)
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    // Perform Dijkstra's algorithm
    for (int k = 0; k < MAZE_ROWS * MAZE_COLS; ++k) {
        int minDistance = MAX_DISTANCE;
        Cell minCell = {-1, -1};

        // Find the cell with the smallest distance among unvisited cells
        for (int i = 0; i < MAZE_ROWS; ++i) {
            for (int j = 0; j < MAZE_COLS; ++j) {
                if (distances[i][j] < minDistance) {
                    minDistance = distances[i][j];
                    minCell = {i, j};
                }
            }
        }

        // Visit the cell with the smallest distance
        for (int i = 0; i < 4; ++i) {
            int newRow = minCell.row + dx[i];
            int newCol = minCell.col + dy[i];

            // Check if the new cell is within the maze bounds
            if (newRow >= 0 && newRow < MAZE_ROWS && newCol >= 0 && newCol < MAZE_COLS) {
                // Calculate Manhattan distance to the neighboring cell
                int manhattanDistance = abs(newRow - startCell.row) + abs(newCol - startCell.col);
                // Calculate the tentative distance to the neighboring cell
                int tentativeDistance = distances[minCell.row][minCell.col] + manhattanDistance;

                // Update the distance if it's shorter than the current distance
                if (tentativeDistance < distances[newRow][newCol]) {
                    distances[newRow][newCol] = tentativeDistance;
                }
            }
        }

        // Mark the current cell as visited (set distance to a large value)
        distances[minCell.row][minCell.col] = MAX_DISTANCE;
    }
}

void followPath() {
    // Define direction arrays for movement (up, down, left, right)
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    // Start from the starting cell
    Cell currentCell = {START_ROW, START_COL};
    while (!(currentCell.row == GOAL_ROW && currentCell.col == GOAL_COL)) {
        int minDistance = distances[currentCell.row][currentCell.col];
        Cell nextCell = currentCell;

        // Check adjacent cells for the one with the shortest distance
        for (int i = 0; i < 4; ++i) {
            int newRow = currentCell.row + dx[i];
            int newCol = currentCell.col + dy[i];

            // Check if the new cell is within the maze bounds
            if (newRow >= 0 && newRow < MAZE_ROWS && newCol >= 0 && newCol < MAZE_COLS) {
                // Check if the distance to the new cell is smaller than the current minimum distance
                if (distances[newRow][newCol] < minDistance) {
                    minDistance = distances[newRow][newCol];
                    nextCell = {newRow, newCol};
                }
            }
        }

        // Move to the cell with the shortest distance
        // Implement logic to move the robot from currentCell to nextCell
        // For example, you might call functions like TurningLeft, TurningRight, Straight, etc.
        int dRow = nextCell.row - currentCell.row;
        int dCol = nextCell.col - currentCell.col;

        // Implement logic to move the robot based on the direction
        if (dCol == -1) {
            // Move left
            // Check left sensor and call appropriate method
            // tick restarted to 0 beacue we have to move back 2 cm before turing
            // once tick reach 41 that means we have space to turn
            if(LightSensorValue(IR_LEFT)==1)
            {
                tick = 0;
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
                while (tick < 41) 
                {
                    // Wait until tick reaches a certain value corresponding to 2 cm
                }
                // Stop moving back
                Stop(motorFoward, motorBackward);
                // Reset tick counter for turning
                tick = 0;
                // Start turning left
                TurningLeft(myServo, motorFoward, motorBackward, TURN_SPEED);
                // Check tick counter to determine when to stop turning
                while (tick < 80) {
                // Wait until tick reaches a certain value for the turn (adjust as necessary)
                }
                // Stop turning left
                Stop(motorFoward, motorBackward);
                
            }
        } else if (dCol == 1) {
            // Move right
            // Check right sensor and call appropriate method
            // tick restarted to 0 beacue we have to move back 2 cm before turing
            // once tick reach 41 that means we have space to turn
            if(LightSensorValue(IR_RIGHT)==1)
            {
                tick = 0;
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
                while (tick < 41) 
                {
                    // Wait until tick reaches a certain value corresponding to 2 cm
                }
                // Stop moving back
                Stop(motorFoward, motorBackward);
                // Reset tick counter for turning
                tick = 0;
                // Start turning left
                TurningRight(myServo, motorFoward, motorBackward, TURN_SPEED);
                // Check tick counter to determine when to stop turning
                while (tick < 80) {
                // Wait until tick reaches a certain value for the turn (adjust as necessary)
                }
                // Stop turning left
                Stop(motorFoward, motorBackward);
            }
        } else if (dRow == -1) {
            // Move up
            // Implement logic for moving up
            if(LightSensorValue(IR_LEFT)==0 && LightSensorValue(IR_RIGHT)==0)
            {
                Straight(myServo,motorFoward,motorBackward,FORWARD_SPEED);
            }
        } else if (dRow == 1) {
            // Move down
            // Implement logic for moving down
            if(LightSensorValue(IR_BACK)==1)
            {
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
            }
        }

        // Update currentCell for the next iteration
        currentCell = nextCell;
    }
}
void returnToStart() {
    // Define direction arrays for movement (up, down, left, right)
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    // Navigate back to the starting point following the shortest path
    // Start from the goal cell
    Cell currentCell = {GOAL_ROW, GOAL_COL};
    while (!(currentCell.row == START_ROW && currentCell.col == START_COL)) {
        int minDistance = distances[currentCell.row][currentCell.col];
        Cell nextCell = currentCell;

        // Check adjacent cells for the one with the shortest distance
        for (int i = 0; i < 4; ++i) {
            int newRow = currentCell.row + dx[i];
            int newCol = currentCell.col + dy[i];

            // Check if the new cell is within the maze bounds
            if (newRow >= 0 && newRow < MAZE_ROWS && newCol >= 0 && newCol < MAZE_COLS) {
                // Check if the distance to the new cell is smaller than the current minimum distance
                if (distances[newRow][newCol] < minDistance) {
                    minDistance = distances[newRow][newCol];
                    nextCell = {newRow, newCol};
                }
            }
        }

        // Move to the cell with the shortest distance
        int dRow = nextCell.row - currentCell.row;
        int dCol = nextCell.col - currentCell.col;

        // Implement logic to move the robot from currentCell to nextCell
        // ...
        if (dCol == -1) {
            // Move left
            // Check left sensor and call appropriate method
            // tick restarted to 0 beacue we have to move back 2 cm before turing
            // once tick reach 41 that means we have space to turn
            if(LightSensorValue(IR_LEFT)==1)
            {
                tick = 0;
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
                while (tick < 41) 
                {
                    // Wait until tick reaches a certain value corresponding to 2 cm
                }
                // Stop moving back
                Stop(motorFoward, motorBackward);
                // Reset tick counter for turning
                tick = 0;
                // Start turning left
                TurningLeft(myServo, motorFoward, motorBackward, TURN_SPEED);
                // Check tick counter to determine when to stop turning
                while (tick < 80) {
                // Wait until tick reaches a certain value for the turn (adjust as necessary)
                }
                // Stop turning left
                Stop(motorFoward, motorBackward);
                
            }
        } else if (dCol == 1) {
            // Move right
            // Check right sensor and call appropriate method
            // tick restarted to 0 beacue we have to move back 2 cm before turing
            // once tick reach 41 that means we have space to turn
            if(LightSensorValue(IR_RIGHT)==1)
            {
                tick = 0;
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
                while (tick < 41) 
                {
                    // Wait until tick reaches a certain value corresponding to 2 cm
                }
                // Stop moving back
                Stop(motorFoward, motorBackward);
                // Reset tick counter for turning
                tick = 0;
                // Start turning left
                TurningRight(myServo, motorFoward, motorBackward, TURN_SPEED);
                // Check tick counter to determine when to stop turning
                while (tick < 80) {
                // Wait until tick reaches a certain value for the turn (adjust as necessary)
                }
                // Stop turning left
                Stop(motorFoward, motorBackward);
            }
        } else if (dRow == -1) {
            // Move up
            // Implement logic for moving up
            if(LightSensorValue(IR_LEFT)==0 && LightSensorValue(IR_RIGHT)==0)
            {
                Straight(myServo,motorFoward,motorBackward,FORWARD_SPEED);
            }
        } else if (dRow == 1) {
            // Move down
            // Implement logic for moving down
            if(LightSensorValue(IR_BACK)==1)
            {
                Backward(motorFoward,motorBackward,FORWARD_SPEED);
            }
        }

        // Update currentCell for the next iteration
        currentCell = nextCell;
    }
}

void delayAndCheck() {
   // Implement logic to pause for around 1 minute at the starting point to check for damage
    // You can use the delay() function to pause the program execution
    // After the delay, you can perform any necessary checks or actions
    delay(60000); // 1 minute delay
    // Implement logic to check for damage or other conditions
}