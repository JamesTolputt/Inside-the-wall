#include "sonar.h"
#include "motor.h"
#include "ir.h"

// This assumes
// motor_forward( getSpeed(increment) ) moves forward at speed increment/15 full speed
// motor_stop() stops the motors
// motor_getLeft( degrees ) turns degrees left on the spot
// motor_getRight( degrees ) turns degrees right on the spot
// sonar_distance() returns distance between obstacle and ultrasonic in millimetres
// sonar_move( degrees ) rotates the servo degrees under the ultrasonic sensor 
// SONIC_RIGHT = 0
// SONIC_CENTER = 90
// SONIC_LEFT = 180
// ir_read() returns a pointer to 4 readings from the 4 ir sensors
// motor_init() initialises motor lib you seem to have this handled
// sonar_init() initialises servo, attaches it and sets it to look forward
// ir_init() no clue... maybe loads up calibrated values for IR_HIGH?
// IR_HIGH = value that represents the analog value for when sensor sees white

// State machine variables
typedef enum State { START, DRIVING, FOLLOW_WALL, STOPPED };

typedef struct StateMachine{
  State currentState = START;
};

# define WALL_DISTANCE 100
StateMachine machine;

// State machine function prototypes
void updateStateMachine(StateMachine smachine);
void start();
void drive();
void follow_wall();
void stop();

//START
void setup() {
  motor_init();
  sonar_init();
  ir_init();
  
  Serial.begin(9600); 
  delay(5000);//delay before starting loop and to show all-is-good smiley
}

void loop()
{
  updateStateMachine(machine); // FIRST RUN WILL ALWAYS BE currentState = START
  
  // Change the state
  machine.currentState = DRIVING; // DEFAULT
  
  uint16_t* ir_readings = ir_read();
  if (ir_readings[1] < IR_HIGH) {
    machine.currentState = STOPPED;
  }
  if (sonar_distance() < WALL_DISTANCE)
    machine.currentState = FOLLOW_WALL;
}

// STATE MACHINE STUFF
void updateStateMachine(StateMachine smachine) {
  switch(statemachine.currentState) {
    case START:
      start();
      break;
    case DRIVING:
      drive();
      break;
    case FOLLOW_WALL:
      follow_wall();
      break;
    case STOPPED:
      stop();
      break;
  }
}

void start() {
  // Reset servo to look forward
	sonar_move( SONIC_CENTER );
}

void drive() {
  // Start driving until state change
  motor_forward( getSpeed(5) );
}

/*
 * On the first call to this function we want to find the wall and the program's
 * execution will be stuck in the while loop till a wall within (wall_distance) is found
 * later calls will follow the wall
*/
void follow_wall() {
  // variable to check if this function has been called before
  static bool first_call = true;
  if (first_call) {
    // Stop the motors
    motor_stop();
    
    // Move head to the right
    sonar_move(SONIC_RIGHT); // servo writes are in degrees
    // Search for wall while spinning anti-clockwise
    while (sonar_distance() > WALL_DISTANCE) {
      // Slow the motors so that there is a more precise search
      motor_turnLeft(8);
    }
    
    // Found wall stop spinning
    motor_stop();
    first_call = false;
  }

  // Start going forward slowly to follow wall
  motor_forward( getSpeed(1) );
  if (sonar_distance() <= WALL_DISTANCE-10) // The wall is to close turn away a bit
    motor_turnRight(16);
  else if (sonar_distance() => WALL_DISTANCE+10) // The wall is to far turn closer a bit
    motor_turnLeft(16);
}

void stop() {
  motor_stop();
  
  // Celebrate! (turn head side to side)
  int angle = 0;
  int direction = 1;
  for(;;) {
    sonar_move(angle);
    angle += direction;
    // If fully on one side flip direction of movement
    if (angle == 180)
      direction *= -1;
    delay(10);
  }
}
//END
