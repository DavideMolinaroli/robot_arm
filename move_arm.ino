#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>
#include <Servo.h>

// Actuation properties
#define ANGLE_PER_STEP 1.8
#define MICROSTEP_FRACTION 8
#define REDUCTION_RATIO 18.666666

// Gripper positions
#define GRIPPER_OPEN 50
#define GRIPPER_CLOSED 10

uint8_t n_steppers = 3;

// Stepper motor pins
uint8_t stepPins[3] = {2,3,4};
uint8_t msPins[3] = {5,6,7}; // Quarter step microstepping
uint8_t dirPins[3] = {11,12,13};

// AccelStepper array
AccelStepper steppers[3];

// Servo object for the gripper
Servo gripper;

// Positions
// uint8_t nsteps[3] = {0,0,0};
// unsigned long cur_millis[3] = {0,0,0};
// unsigned long prev_millis[3] = {0,0,0};
// unsigned long millis_between_steps[3] = {2,2,2};

// Serial command
String command;
double x;
double y;
double z;
double q0;
double q1;
double q2;

/*bool perform_step(uint8_t stepPin, unsigned long millis_between_steps, unsigned long cur_millis, unsigned long &prev_millis){
  if(cur_millis - prev_millis >= millis_between_steps){
    prev_millis = cur_millis;
    digitalWrite(stepPin,HIGH);
    digitalWrite(stepPin,LOW);
    return true;
  }

  return false;
}
*/

// Analytical inverse kinematics for a 3D0F antropomorphic arm.
// If the commanded point is reachable, finds the corresponding solution with shoulder up and elbow down
bool ik() {
  double a2 = 0.18;
  double a3 = 0.25;
  double r = sqrt(x*x+y*y);

  double cos3 =(x*x+y*y+z*z-a2*a2-a3*a3)/(2*a2*a3);
  // Unreachable
  if(abs(cos3) > 1) return false;
  double sin3 = sqrt(1-cos3*cos3);

  q2 = -atan2(sin3, cos3); // in [-pi, pi]

  double den = a2*a2+a3*a3+2*a2*a3*cos3;
  double cos2 = (-r*(a2+a3*cos3)+z*a3*sin3)/den;
  double sin2 = (z*(a2+a3*cos3)+r*a3*sin3)/den;

  q1 = atan2(sin2, cos2); // in [-pi, pi]

  if(x == 0 && y == 0) {
    // Infinitely many solutions, here q0 = 0 is chosen
    q0 = 0;
  }
  else {
    q0 = atan2(y,x); // in [-pi, pi]
  }

  // Convert to degrees and account for 90 deg offset on the second joint
  q0 = q0*180/M_PI;
  q1 = q1*180/M_PI - 90;
  q2 = q2*180/M_PI;

  return true;
}

// Converts from desired joint angle to desired position in steps
void moveJoint(uint8_t id, double q) {
  // From q (joint position in degrees) to position in steps
  double motor_angle = q*REDUCTION_RATIO;
  int steps = motor_angle/ANGLE_PER_STEP*MICROSTEP_FRACTION;

  // Debug
  Serial.println(String(id) + " " + String(q) + " " + String(motor_angle) + " " + String(steps));

  // Set target in steps
  steppers[id].moveTo(steps);
}

void setup() {
  /*for(uint8_t i = 0; i<n_steppers; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(msPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);

    digitalWrite(msPins[i], HIGH);
    digitalWrite(dirPins[i], HIGH);
  }*/

  for(uint8_t i = 0; i<n_steppers; i++) {
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMaxSpeed(i != 1 ? 900 : 700);
    steppers[i].setAcceleration(i != 1 ? 5000 : 3000);
    digitalWrite(msPins[i], HIGH);
  }

  gripper.attach(9);

  // Check gripper
  gripper.write(GRIPPER_OPEN);
  delay(1000);
  gripper.write(GRIPPER_CLOSED);

  Serial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    command = Serial.readStringUntil('\n');
    if(command.equals("0")) {
      // Homing routine
      moveJoint(0,0);
      moveJoint(1,0);
      moveJoint(2,0);
    }
    else if (command.equals("grip open")) {
      gripper.write(GRIPPER_OPEN);
      delay(500);
    }
    else if (command.equals("grip close")) {
      gripper.write(GRIPPER_CLOSED);
      delay(500);
    }
    else {
      // Expected string: "30 60 90\n"
      uint8_t firstSpaceIndex = command.indexOf(' ');
      uint8_t secondSpaceIndex = command.indexOf(' ', firstSpaceIndex + 1);

      if (firstSpaceIndex > 0 && secondSpaceIndex > firstSpaceIndex) {
        // Extract each value from the command string
        x = command.substring(0, firstSpaceIndex).toDouble();
        y = command.substring(firstSpaceIndex + 1, secondSpaceIndex).toDouble();
        z = command.substring(secondSpaceIndex + 1).toDouble();
      }

      if(ik()) {
        // Set target for each stepper
        moveJoint(0,q0);      
        moveJoint(1,q1);
        moveJoint(2,q2);
      }
      else {
        Serial.println("Unreachable");
      }
    }
  }

  // Take steps when possible
  steppers[0].run();
  steppers[1].run();
  steppers[2].run();
}
