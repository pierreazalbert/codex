#include <Adafruit_NeoPixel.h>
#include <Ultrasonic.h>

#define IDLE_STATE 0
#define LISTEN_STATE 1
#define TRANSMIT_STATE 2
#define RECEIVE_STATE 3

#define LOOP_DELAY 500
#define LED_DELAY 3000

#define MOTOR_SLOW 128
#define MOTOR_FAST 255
#define MOTOR_STOP 0
#define MOTOR_DELAY 5

#define VOLUME_THRESHOLD 1.0

#define NUMPIXELS 8

// ************ PIN DEFINITIONS ************************************************************

// Ultrasonic sensor
const int trigPin = 9; //PWM
const int echoPin = 10; //PWM

// Motor output
const int motorPin = 11; //PWM

// Mic sensor
const int micPin = 0; //Analog

// LEDs
const int ledPin = 6; // Digital IO for LED control
const int ledPin2 = 5;


// ************ VAR DEFINITIONS ************************************************************

// Ultrasonic sensor
Ultrasonic ultrasonic(trigPin, echoPin);
int currentRange = 0;   // 0:stop, 1:mid speed, 2: full speed
int previousRange = 0;  // buffer to compare new and old value
int rangeCount = 0;     // counter to wait between transitions

// Motor output
int currentSpeed = 0;

// Mic sensor
bool currentSpeech = false;
bool previousSpeech = false;
int speechCount = 0;
int speechLength = 0;

// LEDs
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, ledPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(NUMPIXELS, ledPin2, NEO_GRB + NEO_KHZ800);

int r0, g0, b0;
int r1, g1, b1;
int r2, g2, b2;
int r3, g3, b3;

// Finite State Machine
unsigned int state;

// ************ MAIN CODE ************************************************************

void setup() {

  // Debugging
  Serial.begin(9600); // Starts the serial communication
  while (!Serial);    // Wait here until serial port is readys

  // Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Motor
  pinMode(motorPin, OUTPUT);

  // LEDs
  pixels.begin(); // This initializes the NeoPixel library.
  pixels2.begin();
  randomSeed(analogRead(1)); // This initialises the random number generator for LEDs input

  // Finite State Machine
  state = IDLE_STATE;

  // Initialisation messages
  Serial.println("FSM initialised to state 0 (IDLE_STATE)");
  Serial.println("Range initialised to 0 (NO USER)");
  Serial.println("Motor speed initialised to 0 (MOTOR STOP)");
}

void loop() {

  //Serial.print("Current state: ");
  //Serial.println(state);

  switch (state)
  {

    // IDLE - Check for presence of user - If present go to state 1 to start motor, otherwise do nothing
    case IDLE_STATE:

      // Turn off motor
      motorStop();

      // Check if user is here - if there is a user go to LISTEN state
      detectRange();

      break;

    // LISTEN - User detected, check for start of speech - If speech started, go to state 2 to increase motor speed
    case LISTEN_STATE:

      // Set speed of motor to SLOW
      motorSlow();

      // Detect beginning of speech
      detectSpeechStart();

      // Check if user leaves - if he does go back to IDLE state, otherwise stay in LISTEN
      detectRange();

      break;

    // TRANSMIT - User detected, speech started, check for end of speech - If speech ended, go to state 3 to activate LEDs
    case TRANSMIT_STATE:

      // Set speed of motor to FAST
      motorFast();

      // Track length of message
      if (currentSpeech == true) {
        speechLength++;
      }
      else speechLength = 0;

      // Detect end of speech
      detectSpeechStop();

      break;

    // RECEIVE - User detected, end of speech detected - If user stays, go to state 1, otherwise go to state 0
    case RECEIVE_STATE:

      // Wait before activating LEDs
      delay(LED_DELAY);

      // Activate LED
      activateLEDs();

      break;

  }

  delay(LOOP_DELAY);

}


// ************ CUSTOM FUNCTIONS ************************************************************

int getDistance() {

  int distance;
  distance = ultrasonic.distanceRead();

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  // map the result of the distance to speed range
  if (distance > 150) {
    return 0;
  }
  else if (distance > 100) {
    return 1;
  }
  else if (distance > 50) {
    return 2;
  }

}

double getVolume() {

  const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
  unsigned int sample;

  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(micPin);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

  return volts;
}

void motorStop() {
  // Turn motor off
  if (currentSpeed != MOTOR_STOP) {
    Serial.print("Decreasing motor speed to STOP (current speed: ");
    Serial.print(currentSpeed);
    Serial.println(")");
    for (int x = currentSpeed; x >= MOTOR_STOP; x--) {
      analogWrite(motorPin, x);
      delay(MOTOR_DELAY);
    }
    currentSpeed = MOTOR_STOP;
  }
}

void motorSlow() {
  // Set motor to low-medium speed (check first it we are slowing down or speeding up from previous speed)
  // If current motor speed higher than slow setting, ramp down speed
  if (currentSpeed != MOTOR_SLOW) {
    if (currentSpeed > MOTOR_SLOW) {
      Serial.print("Increasing motor speed to SLOW (current speed: ");
      Serial.print(currentSpeed);
      Serial.println(")");
      for (int x = currentSpeed; x >= MOTOR_SLOW; x--) {
        analogWrite(motorPin, x);
        delay(MOTOR_DELAY);
      }
      currentSpeed = MOTOR_SLOW;
    }
    // Else (i.e. current motor speed lower than slow setting), ramp up speed
    else {
      Serial.print("Decreasing motor speed to SLOW (current speed: ");
      Serial.print(currentSpeed);
      Serial.println(")");
      for (int x = currentSpeed; x <= MOTOR_SLOW; x++) {
        analogWrite(motorPin, x);
        delay(MOTOR_DELAY);
      }
      currentSpeed = MOTOR_SLOW;
    }
  }
}

void motorFast() {
  // Increase speed of motor to high speed
  if (currentSpeed != MOTOR_FAST) {
    Serial.print("Increasing motor speed to FAST (current speed: ");
    Serial.print(currentSpeed);
    Serial.println(")");
    for (int x = currentSpeed; x <= MOTOR_FAST; x++) {
      analogWrite(motorPin, x);
      delay(MOTOR_DELAY);
    }
    currentSpeed = MOTOR_FAST;
  }
}

void detectSpeechStart() {

  // Detect sound level
  double newVolume;
  newVolume = getVolume();
  Serial.print("Volume: ");
  Serial.println(newVolume);

  // Convert voltage into speaking/not speaking result
  bool newSpeech;
  if (newVolume >= VOLUME_THRESHOLD) {
    newSpeech = true;
  }
  else {
    newSpeech = false;
  }

  if (newSpeech == previousSpeech) {
    speechCount++;
  }
  else {
    speechCount = 0;
  }

  if (speechCount == 3) {
    if (newSpeech != currentSpeech) {
      // update speech value
      currentSpeech = newSpeech;
      if (currentSpeech == true) {
        Serial.print("Started speaking... (");
        Serial.print(newVolume);
        Serial.println(")");
        state = TRANSMIT_STATE;
        Serial.print("Transitioned to TRANSMIT state");
        //motorFast();
        //speechLength = 3;
      }
    }
    speechCount = 0;
  }
  previousSpeech = newSpeech;

  //  Serial.print("speechCount: ");
  //  Serial.println(speechCount);
}

void detectSpeechStop() {
  //
  //  Serial.print("previousSpeech: ");
  //  Serial.println(previousSpeech);

  // Detect sound level
  double newVolume;
  newVolume = getVolume();
  Serial.print("Volume: ");
  Serial.println(newVolume);

  // Convert voltage into speaking/not speaking result
  bool newSpeech;
  if (newVolume >= VOLUME_THRESHOLD) {
    newSpeech = true;
  }
  else {
    newSpeech = false;
  }

  if (newSpeech == previousSpeech) {
    speechCount++;
  }
  else {
    speechCount = 0;
  }

  if (speechCount == 3) {
    //Serial.println("Counter threshold reached");
    if (newSpeech != currentSpeech) {
      // update speech value
      //Serial.println("Updating current speech value");
      currentSpeech = newSpeech;
      if (currentSpeech == false) {
        Serial.print("Stopped speaking... (");
        Serial.print(newVolume);
        Serial.println(")");
        state = RECEIVE_STATE;
        Serial.println("Transitioned to RECEIVE state");
      }
    }
    speechCount = 0;
  }
  previousSpeech = newSpeech;

  //  Serial.print("speechCount: ");
  //  Serial.println(speechCount);
  //  Serial.print("currentSpeech: ");
  //  Serial.println(currentSpeech);
}

void activateLEDs() {

  //Serial.println("Activating LED pattern");

  double randomInput = random(110,490)/100.0;
  Serial.print("Random LED input: ");
  Serial.println(randomInput);

  Serial.print("Remaining length: ");
  Serial.println(speechLength);

  pixels.setPixelColor(0, input_to_color0 (randomInput));
  pixels.setPixelColor(1, input_to_color1());
  pixels.setPixelColor(2, input_to_color2());
  pixels.setPixelColor(3, input_to_color3());

  pixels2.setPixelColor(0, input_to_color0 (randomInput));
  pixels2.setPixelColor(1, input_to_color1());
  pixels2.setPixelColor(2, input_to_color2());
  pixels2.setPixelColor(3, input_to_color3());

  pixels.show(); // This sends the updated pixel color to the hardware
  pixels2.show();

  if (speechLength > 0) {
    speechLength--;
  }

  if (speechLength == 0) {
    // Go back to LISTEN state
    state = LISTEN_STATE;
    Serial.print("Transitioned to LISTEN state");
    colorWipe(pixels.Color(0, 0, 0), 50);
    colorWipe(pixels2.Color(0, 0, 0), 50);
  }

}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixels.show();
    pixels2.setPixelColor(i, c);
    pixels2.show();
    delay(wait);
  }
}

void detectRange() {

  //Serial.print("Current range: ");
  //Serial.println(currentRange);

  // Detect presence and/or distance of user
  int newRange;
  newRange = getDistance();
  if (newRange == previousRange) {
    rangeCount++;
  }
  else {
    rangeCount = 0;
  }
  if (rangeCount == 3 ) {
    if (newRange != currentRange) {
      // update range value
      currentRange = newRange;
      Serial.print("New range detected: ");
      Serial.println(currentRange);
      // transition to corresponding state
      if (currentRange == 0) {
        state = IDLE_STATE;
        Serial.println("Transitioned to IDLE state");
      }
      if (currentRange >= 1) {
        state = LISTEN_STATE;
        Serial.println("Transitioned to LISTEN state");
      }
      //state = currentRange;
    }
    rangeCount = 0;
  }
  previousRange = newRange;
}

uint32_t input_to_color0(double input) {
  r0 = input_to_r0(input);
  g0 = input_to_g0(input);
  b0 = input_to_b0(input);

  return pixels.Color(r0, g0, b0);
}


int input_to_r0(double input) {
  if (input < 3.0) {
    return 0;
  }
  else if (input < 4) {
    return map(input, 3, 4, 0, 255);
  }
  else if (input < 5) {
    return 255;
  }
  else {
    return 0;
  }
}

int input_to_g0(double input) {
  if (input < 1) {
    return 0;
  }
  else if (input < 2) {
    return map(input, 1, 2, 0, 255);
  }
  else if (input < 4) {
    return 255;
  }
  else if (input < 5) {
    return map(input, 4, 5, 255, 0);
  }
  else {
    return 0;
  }
}

int input_to_b0(double input) {
  if (input < 1) {
    return 255;
  }
  else if (input < 2) {
    return 255;
  }
  else if (input < 3) {
    return map(input, 2, 3, 255, 0);
  }
  else {
    return 0;
  }
}

uint32_t input_to_color1() {
  int r1 = 255 - r0;
  int g1 = 255 - g0;
  int b1 = 255 - b0;

  return pixels.Color(r1, g1, b1);
}

uint32_t input_to_color2() {
  int r2 = g0;
  int g2 = b0;
  int b2 = r0;

  return pixels.Color(r2, g2, b2);
}

uint32_t input_to_color3() {
  int r3 = b1;
  int g3 = r1;
  int b3 = g1;

  return pixels.Color(r3, g3, b3);
}


