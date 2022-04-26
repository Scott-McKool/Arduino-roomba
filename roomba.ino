/* class to make a stream of data more resistant to perturbations 
 *  ( mathamatically simmilar to the moving average algorythm)
*/
class smoother{
public:
  // how much of the current value is given by the most recent mesurement 0 to 1
  float alpha = 0.0;
  smoother(float _alpha){
    alpha = _alpha;
  }
  float accumulator = 0.0;
  float smooth(float value){
    // set the value equal to itself combined with the last mesurement according to alpha
    accumulator = (alpha*value)+(1.0-alpha)*accumulator;
    return accumulator;
  }
};

// class to allow for high-level usage of the ultrasonic sensor
class ultrasonicSensor{
public:
  // used to keep track of the time spent waiting for the echo
  long duration;
  // used to keep track of the distance in cm mesured by the sensor
  int distance;
  // the pin that sends out the pulse
  int triggerPin;
  // the pin that signals when the sensor gets the echo
  int echoPin;

  // set up pins and variables
  ultrasonicSensor(int _triggerPin, int _echoPin) {
    triggerPin = _triggerPin;
    echoPin = _echoPin;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
  // tell the sensor to send out a pulse and return the resulting distance (cm)
  int getDistance(){
    // sensor expects 2 milis of time between HIGH signals
    digitalWrite(triggerPin,LOW);
    delayMicroseconds(2);
  
    digitalWrite(triggerPin,HIGH);
    delayMicroseconds(10);

    duration = pulseIn(echoPin, HIGH);
    // use the durration times a constant to get the distance
    distance = duration*0.034/2;
    return distance;
  }
};

// high level implimentation of DC motor & H-bridge
class motor{
public:
  // what direction should be the motor's default
  int my_direction = 1;
  // from -1(full reverse) to 1(full forward) how should the motor spin
  double throttle = 0.0;
  // the clockwise rotation pin for this motor ( goes to the H-Bridge)
  int pinC = 0;
  // the clockwise rotation pin for this motor ( goes to the H-Bridge)
  int pinW = 1;
  // method to set up the motor, sets up pin modes and variables
  motor(int _pinC, int _pinW, int _my_direction=1){
    pinC = _pinC;
    pinW = _pinW;
    my_direction = _my_direction;
    pinMode(pinC,OUTPUT);
    pinMode(pinW,OUTPUT);
  }
  setPower(double throttle){
    // adjust throttle to go in the desired direction
    throttle = throttle*my_direction;
    // determine which pin shoud get the signal
    bool reverse = false;
    if(throttle < 0.0){
      reverse = true;
    }
    // turn the absolute value of throttle into an analog signal
    double throttleSignal = min(1,abs(throttle))*255;
    // depending on reverse, send the signal to the C or W pin
    if(reverse){
      analogWrite(pinW,throttleSignal);
      analogWrite(pinC,0);
    }else{
      analogWrite(pinW,0);
      analogWrite(pinC,throttleSignal);
    }
  }
};

// place to collect and control motors
class driveTrain{
public:
  // the left motor of the drivetrain
  motor leftMotor = motor(5,6,-1);
  // right motor of the drivetrain
  motor rightMotor = motor(10,11,1);
  driveTrain(){
    
  }
  // method that justs sets the throttle to the motors
  drive(double forwards, double turn){
    // arcade drive scheme where throttle per motor is 
    // equal to forwards+turn where both are from -1 to 1 to allow for driving and turning
    leftMotor.setPower(forwards + turn);
    rightMotor.setPower(forwards - turn);
  }
};

smoother ultrasonicSmoother = smoother(0.25);
ultrasonicSensor distanceSensor = ultrasonicSensor(12,13);
driveTrain DriveTrain = driveTrain();
//used to track the distance from the wall
int distance = 0;
// sought distance from wall (cm)
int distanceTrigger = 50;
void setup() {
  Serial.begin(9600);
}
void loop() {

  // get the sensor's mesured distance in cm
  distance = distanceSensor.getDistance();
  // smooth the sensor data to make it more reliable
  distance = round( ultrasonicSmoother.smooth( distance ) );
  Serial.println(distance);
  // adjust distance for use in turning
  distance = distance - distanceTrigger;
  // if too far from the wall
  //if(distance > 0){
    // turn to the left 20% while driving forwards 75%
    DriveTrain.drive(0.75,-0.3);
    delay(1000);
    DriveTrain.drive(-0.75,0.3);
    delay(1000);
  // if too close to the wall
  //}else{
    // turn to the right 20% while driving forwards 75%
  //  DriveTrain.drive(0.75,0.3);
  //} 
}
