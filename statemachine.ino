// Headers for Motors
// #include <ENGG1500Lib.h> Not really needed
#include <Servo.h>

// Pins
#define ENA 5 // LEFT
#define ENB 6 // RIGHT
#define IN1 8
#define IN2 4
#define IN3 10
#define IN4 11
#define ECHO 12 // Pin that senses ultrasonic pulse
#define TRIG 7 // Pin that sends ultrasonic pulse
#define SONICSTEPPER 9 // Ultrasonic sensor's stepper motor

// Enum for direction of motors
enum direction{
  forward, backward, noward
};

const float speed_sound = 340.29; // m/s
Servo sonic_servo; // Initialise servo motor driver object
#define SERVO_RIGHT 0
#define SERVO_CENTER 90
#define SERVO_LEFT 180

// Function prototypes
void left_motor(direction dir=forward);
void right_motor(direction dir=forward);
unsigned int sonar_distance(byte checks=10); // Assumes you don't want to average >255 values for REAL accurate reading

// State machine variables
typedef enum State { START, DRIVING, FOLLOW_WALL, STOPPED };

typedef struct StateMachine {
  State currentState = START;
};
# define WALL_DISTANCE 100
StateMachine machine;
// State machine function prototypes
void state_machine(StateMachine statemachine);
void start();
void drive();
void follow_wall();
void stop();

//START
void setup() {
  // put your setup code here, to run once:
  pinMode(ENA,OUTPUT); //set ENA as an output
  pinMode(ENB,OUTPUT); //set ENB as an output
  pinMode(IN1,OUTPUT); //set IN1 as an output
  pinMode(IN2,OUTPUT); //set IN2 as an output
  pinMode(IN3,OUTPUT); //set IN3 as an output
  pinMode(IN4,OUTPUT); //set IN4 as an output
  pinMode(A0, INPUT); //right IR sensor
  pinMode(A1, INPUT); //left IR sensor
  pinMode(ECHO,INPUT); //Initialise ECHO as an input
  pinMode(TRIG,OUTPUT); //Initialise TRIG as an output
  
  Serial.begin(9600); // Setup console to prtin. Set the baud rate on the serial connection
  //enc_init(); // Initialise motor encoder library // Not really needed
  sonic_servo.attach(SONICSTEPPER); // Attach pin SONICSWEEPER to sonic_servo object

  left_motor(noward);
  right_motor(noward);
  
  delay(5000);//delay before starting loop and to show all-is-good smiley
}

int servo_facing = SERVO_RIGHT;

void loop()
{
  // Clear the amount of "clicks" the motors have run
  // enc_clear(); Not really needed
  
  state_machine(machine); // FIRST RUN WILL ALWAYS BE currentState = START
  
  // Change the state
  machine.currentState = DRIVING; // DEFAULT
  /*
   * IF IR SENSOR SEES DARK PATCH  //could not code the stop transition cause i don't know what analog value "black" is
   * machine.currentState = STOPPED
  */
  if(sonar_distance() < WALL_DISTANCE)
    machine.currentState = FOLLOW_WALL;
}

// Function definitions
//This function sets IN1 = LOW and IN2 = HIGH in order to set the direction to forwards for motor 1
void left_motor(direction dir) {
  //Serial.println("forward");
  if(dir == forward)
  {
    digitalWrite(IN1,LOW); //IN1
    digitalWrite(IN2,HIGH); //IN2
  }
  else if(dir == backward)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(dir == noward)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
}

//This function sets IN3 = LOW and IN4 = HIGH in order to set the direction to forwards for motor 2
void right_motor(direction dir) {
  //Serial.println("backward");
  if(dir == forward)
  {
    digitalWrite(IN3,LOW); //IN3
    digitalWrite(IN4,HIGH); //IN4
  }
  else if(dir == backward)
  {
    digitalWrite(IN3,HIGH); //IN3
    digitalWrite(IN4,LOW); //IN4
  }
  else if(dir == noward)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

unsigned int sonar_distance(byte checks) {
  // Sum of all sensor readings so that they can be "smoothed"
  unsigned long average_duration = 0L;
  
  for (byte i = 0; i < checks;i++) {
    // Make a 10 microsecond pulse
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // half (cause we don't want back AND forth) * duration (micro s) * 1e-6(conversion to s) * speed of sound (m/s) * 1e3 (conversion to mm)
    average_duration += pulseIn(ECHO, HIGH); 
  }
  
  average_duration /= checks; // Average all the readings so we get a "smoothed" (more accurate) reading
  
  Serial.print("Distance=");Serial.println((unsigned int)(0.5 * average_duration * 1e-6 * speed_sound * 1e3)); //print to console
  
  return (unsigned int)(0.5 * average_duration * 1e-6 * speed_sound * 1e3);  
}

// STATE MACHINE STUFF
void state_machine(StateMachine statemachine) {
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
	sonic_servo.write(SERVO_CENTER);
}

void drive() {
  // Move forward (might need to change values to make it stay straight)
  analogWrite(ENA, 105);
  analogWrite(ENB, 150);
  left_motor(forward);
  right_motor(forward);
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
    right_motor(noward);
    left_motor(noward);
    
    // Move head to the right
    sonic_servo.write(SERVO_RIGHT); // servo writes are in degrees
    // Search for wall while spinning anti-clockwise
    while (sonar_distance() > WALL_DISTANCE) {
      // Slow the motors so that there is a more precise search
      analogWrite(ENA, (int)105*0.7);
      analogWrite(ENB, (int)150*0.7);
      right_motor(forward);
      left_motor(backward);
    }
    
    // Found wall stop spinning
    right_motor(noward);
    // Set left motor to have correct speed
    analogWrite(ENA, 105);
    first_call = false;
  }
  
  // Set right motor to go forward
  right_motor(forward);
  // if the wall is too close stop the left motor and let the right motor catch up
  if (sonar_distance() <= WALL_DISTANCE-10)
    left_motor(noward);
  else
    left_motor(forward);
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Celebrate! (turn head side to side)
  int angle = 0;
  int direction = 1;
  for(;;) {
    sonic_servo.write(angle);
    angle += direction;
    // If fully on one side flip direction of movement
    if (angle == 180)
      direction *= -1;
    delay(10);
  }
}
//END
