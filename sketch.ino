#include <Arduino.h>
#include <math.h>
#include <IRremote.hpp>
#include <LiquidCrystal.h>

// Sound Following Car with 3 MAX9814 Microphones
// Triangle configuration with 18cm sides

#define TRIANGLESIDE 524      // 18cm converted to microseconds (speed of sound = 343m/s)
#define MAXTIME 15000         // Maximum time to wait for all sensors to trigger
#define DELAYTIME 5000        // Minimum delay between detections
#define TRIGGERVALUE 100      // Analog threshold for sound detection
#define MAX_TIMEDIFF 10000    // Maximum expected time difference between sensors (microseconds)
#define MIN_ANALOG_DIFF 20    // Minimum difference in analog values to consider as separate triggers
#define PEAK_WINDOW 10        // Number of samples to look for peak
#define SAMPLE_DELAY 50       // Microseconds between samples (20kHz sampling rate)
#define MIN_TIMEDIFF 100      // Minimum time difference to consider as separate triggers
#define MAX_PHYSICAL_DIST 1000  // Maximum physical distance between sensors in cm
#define ECHO_THRESHOLD 500    // Minimum time between peaks to consider as separate sounds
#define SENSOR_RESPONSE_TIME 3000  // Typical MAX9814 response time in microseconds
#define irReceiver 5
#define irFront 0xB946FF00
#define irRight 0xBC43FF00
#define irBack 0xEA15FF00
#define irLeft 0xBB44FF00
#define irStop 0xBF40FF00
#define irRemote 0xBA45FF00
#define irSound 0xB847FF00




const int rs = 22, en = 27, d4 = 23, d5 = 25, d6 = 24, d7 = 26;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Microphone pins - adjust these according to your Arduino connections
const int micA = A1;  // Front microphone
const int micB = A2;  // Right microphone
const int micC = A0;  // Left microphone

bool micA_trigger = false;
bool micB_trigger = false;
bool micC_trigger = false;

unsigned long micA_timestamp, micB_timestamp, micC_timestamp;
unsigned long lastDebugPrint = 0;  // For controlling debug output timing
unsigned long lastPeakTime = 0;    // For echo detection

struct SoundSource {
  double angle;     // Direction in degrees (0-360)
  double distance;  // Distance in cm
  bool valid;       // Whether the calculation was successful
};

SoundSource soundSource = {0, 0, false};

// Motor driver pins
int IN11 = 6; // rear right -
int IN12 = 7; // rear right +
int IN13 = 9; // rear left -
int IN14 = 8; // rear left +
int IN21 = 10; // forward right -
int IN22 = 11; // forward right +
int IN23 = 12; // forward left -
int IN24 = 13; // forward left +

// Ultrasonic sensor pins
int frontTrig = 45;
int frontEcho = 44;
int rightTrig = 47;
int rightEcho = 49;
int leftTrig = 46;
int leftEcho = 48;

// Constants
const int safeDist = 10;  // Safe distance in cm
// long duration;
// int distance;

// Movement constants
const float TARGET_SPEED = 0.2;  // Target speed in m/s (20 cm/s)
const int MOTOR_SPEED = 255;     // Motor PWM value (0-255)

// Movement control variables
unsigned long movementStartTime = 0;
unsigned long movementDuration = 0;
bool isMoving = false;

// Motor control functions
void forward() {
  digitalWrite(IN11, LOW);
  digitalWrite(IN12, HIGH);
  digitalWrite(IN13, LOW);
  digitalWrite(IN14, HIGH);
  digitalWrite(IN21, LOW);
  digitalWrite(IN22, HIGH);
  digitalWrite(IN23, LOW);
  digitalWrite(IN24, HIGH);
}

void back() {
  digitalWrite(IN11, HIGH);
  digitalWrite(IN12, LOW);
  digitalWrite(IN13, HIGH);
  digitalWrite(IN14, LOW);
  digitalWrite(IN21, HIGH);
  digitalWrite(IN22, LOW);
  digitalWrite(IN23, HIGH);
  digitalWrite(IN24, LOW);
}

void right() {
  digitalWrite(IN11, LOW);
  digitalWrite(IN12, HIGH);
  digitalWrite(IN13, HIGH);
  digitalWrite(IN14, LOW);
  digitalWrite(IN21, LOW);
  digitalWrite(IN22, HIGH);
  digitalWrite(IN23, HIGH);
  digitalWrite(IN24, LOW);
}

void left() {
  digitalWrite(IN11, HIGH);
  digitalWrite(IN12, LOW);
  digitalWrite(IN13, LOW);
  digitalWrite(IN14, HIGH);
  digitalWrite(IN21, HIGH);
  digitalWrite(IN22, LOW);
  digitalWrite(IN23, LOW);
  digitalWrite(IN24, HIGH);
}

void stopMotors() {
  digitalWrite(IN11, LOW);
  digitalWrite(IN12, LOW);
  digitalWrite(IN13, LOW);
  digitalWrite(IN14, LOW);
  digitalWrite(IN21, LOW);
  digitalWrite(IN22, LOW);
  digitalWrite(IN23, LOW);
  digitalWrite(IN24, LOW);
}

// Ultrasonic sensor function
int getDistance(int trigPin, int echoPin) {
  long duration;
  int distance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1; // Return -1 if no echo
  distance = duration * 0.0343 / 2;
  return distance;
}

// Function to find the true peak in a window of samples
int findTruePeak(int pin, unsigned long startTime, unsigned long& peakTime) {
  int peak = 0;
  int samples[PEAK_WINDOW];
  unsigned long times[PEAK_WINDOW];
  
  // Collect samples
  for(int i = 0; i < PEAK_WINDOW; i++) {
    samples[i] = analogRead(pin);
    times[i] = micros();
    delayMicroseconds(SAMPLE_DELAY);
  }
  
  // Find the highest peak
  for(int i = 0; i < PEAK_WINDOW; i++) {
    if(samples[i] > peak) {
      peak = samples[i];
      peakTime = times[i];
    }
  }
  
  return peak;
}

SoundSource calculate(double a, double b, double c, double G) {
  SoundSource result;
  
  // Debug input values
  Serial.println("Calculation inputs:");
  Serial.println("a: " + String(a) + " us");
  Serial.println("b: " + String(b) + " us");
  Serial.println("c: " + String(c) + " us");
  Serial.println("G: " + String(G) + " us");
  
  // If all delays are 0, source is equidistant from all sensors
  if (a == 0 && b == 0 && c == 0) {
    result.angle = 0;
    result.distance = G / sqrt(3);  // Distance to center of triangle
    result.valid = true;
    return result;
  }

  // Convert time differences to distances, accounting for sensor response time
  double distA = (a > SENSOR_RESPONSE_TIME) ? (a - SENSOR_RESPONSE_TIME) * 0.0343 : 0;
  double distB = (b > SENSOR_RESPONSE_TIME) ? (b - SENSOR_RESPONSE_TIME) * 0.0343 : 0;
  double distC = (c > SENSOR_RESPONSE_TIME) ? (c - SENSOR_RESPONSE_TIME) * 0.0343 : 0;
  
  Serial.println("Converted distances (adjusted for sensor response):");
  Serial.println("distA: " + String(distA) + " cm");
  Serial.println("distB: " + String(distB) + " cm");
  Serial.println("distC: " + String(distC) + " cm");

  // Validate physical distances
  if (distA > MAX_PHYSICAL_DIST || distB > MAX_PHYSICAL_DIST || distC > MAX_PHYSICAL_DIST) {
    Serial.println("WARNING: Physical distance too large after sensor response adjustment!");
    Serial.println("Max allowed: " + String(MAX_PHYSICAL_DIST) + " cm");
    result.valid = false;
    return result;
  }

  // Adjust time differences for sensor response
  a = (a > SENSOR_RESPONSE_TIME) ? a - SENSOR_RESPONSE_TIME : 0;
  b = (b > SENSOR_RESPONSE_TIME) ? b - SENSOR_RESPONSE_TIME : 0;
  c = (c > SENSOR_RESPONSE_TIME) ? c - SENSOR_RESPONSE_TIME : 0;

  double r_1 = (-2 * a * a * a + a * a * (b + c) - sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G)) - (b + c) * (2 * b * b - 3 * b * c + 2 * c * c - G * G) + a * (b * b + c * c + G * G)) / (4 * (a * a + b * b - b * c + c * c - a * (b + c)) - 3 * G * G);
  double r_2 = (-2 * a * a * a + a * a * (b + c) + sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G)) - (b + c) * (2 * b * b - 3 * b * c + 2 * c * c - G * G) + a * (b * b + c * c + G * G)) / (4 * (a * a + b * b - b * c + c * c - a * (b + c)) - 3 * G * G);

  Serial.println("Intermediate results (after sensor response adjustment):");
  Serial.println("r_1: " + String(r_1));
  Serial.println("r_2: " + String(r_2));

  if (isnan(r_1) || isnan(r_2)) {
    Serial.println("Calculation failed: Invalid intermediate results");
    result.valid = false;
    return result;
  }

  double i_1 = 2 * a * a * a * (b - c) + 2 * b * b * b * c + 4 * c * c * G * G - 3 * G * G * G * G;
  double i_2 = 3 * a * a * (2 * c * (-b + c) + G * G) + b * b * (-6 * c * c + 5 * G * G);
  double i_3 = sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G));
  double i_4 = b * b * b - 3 * b * b * c + 2 * c * c * c + (2 * b + c) * G * G;
  double i_5 = 8 * (a * a + b * b - b * c + c * c - a * (b + c)) * G - 6 * G * G * G;
  double i_7 = 2 * sqrt(3) * (-4 * (a * a + b * b - b * c + c * c - a * (b + c)) * G + 3 * G * G * G);

  double x, y;
  if (r_1 >= r_2) {
    x = (i_1 + i_2 + 2 * b * (2 * c * c * c - 3 * c * G * G + i_3) - 2 * a * (i_4 + i_3)) / i_5;
    y = (6 * b * b * b * (a - c) + 2 * a * i_3 + 2 * b * i_3 - 4 * c * i_3 - 3 * (a - G) * (a + G) * (2 * (a - c) * c + G * G) - 3 * b * b * (2 * (a - c) * (2 * a + c) + G * G) + 6 * b * (a * a * a + a * a * c - 2 * a * c * c + c * G * G)) / i_7;
  } else {
    x = (i_1 + i_2 + 2 * a * i_3 - 2 * a * (i_4) - 2 * b * (-2 * c * c * c + 3 * c * G * G + i_3)) / i_5;
    y = (6 * b * b * b * (a - c) - 2 * a * i_3 - 2 * b * i_3 + 4 * c * i_3 - 3 * (a - G) * (a + G) * (2 * (a - c) * c + G * G) - 3 * b * b * (2 * (a - c) * (2 * a + c) + G * G) + 6 * b * (a * a * a + a * a * c - 2 * a * c * c + c * G * G)) / i_7;
  }

  Serial.println("Final coordinates:");
  Serial.println("x: " + String(x));
  Serial.println("y: " + String(y));

  // Calculate angle and distance
  double angle = atan2(y - G / (2 * sqrt(3)), x - 0.5 * G) * 180 / PI;
  if (angle < 0) {
    angle += 360;
  }
  
  // Convert distance from microseconds to centimeters
  double distance = sqrt((x - 0.5 * G) * (x - 0.5 * G) + (y - G / (2 * sqrt(3))) * (y - G / (2 * sqrt(3))));
  distance = distance * 34300 / 1000000; // Convert microseconds to centimeters

  
  result.angle = angle;
  result.distance = distance;
  result.valid = true;
  return result;
}

void setup() {
  Serial.begin(115200);
  IrReceiver.begin(irReceiver, ENABLE_LED_FEEDBACK); 
  pinMode(micA, INPUT);
  pinMode(micB, INPUT);
  pinMode(micC, INPUT);

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  
  // Setup motor driver pins
  pinMode(IN11, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN13, OUTPUT);
  pinMode(IN14, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(IN23, OUTPUT);
  pinMode(IN24, OUTPUT);
  
  // Setup ultrasonic sensor pins
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  
  // Initialize motors to stop
  stopMotors();
  
  Serial.println("Sound Following Car Debug Mode");
  Serial.println("-----------------------------");
  Serial.println("Testing sensor connections...");
  
  // Test each sensor
  int testA = analogRead(micA);
  int testB = analogRead(micB);
  int testC = analogRead(micC);
  
  Serial.println("Initial sensor readings:");
  Serial.println("MicA: " + String(testA));
  Serial.println("MicB: " + String(testB));
  Serial.println("MicC: " + String(testC));
  
  if (testA == 0 && testB == 0 && testC == 0) {
    Serial.println("WARNING: All sensors reading 0!");
    Serial.println("Please check:");
    Serial.println("1. Power connections (5V and GND)");
    Serial.println("2. Analog pin connections (A0, A1, A2)");
    Serial.println("3. MAX9814 gain setting");
  }
  
  Serial.println("-----------------------------");
}

void printDebug() {
  // Print sensor values every 500ms
  if (millis() - lastDebugPrint >= 500) {
    int valA = analogRead(micA);
    int valB = analogRead(micB);
    int valC = analogRead(micC);
    
    Serial.println("Sensor Readings:");
    Serial.println("MicA: " + String(valA) + " (Triggered: " + String(micA_trigger) + ")");
    Serial.println("MicB: " + String(valB) + " (Triggered: " + String(micB_trigger) + ")");
    Serial.println("MicC: " + String(valC) + " (Triggered: " + String(micC_trigger) + ")");
    
    // Add warning if values are still 0
    if (valA == 0 && valB == 0 && valC == 0) {
      Serial.println("WARNING: All sensors still reading 0!");
    }
    
    Serial.println("-----------------------------");
    
    lastDebugPrint = millis();
  }
}

// Function to read microphone values
void readMicrophoneValues(int& valA, int& valB, int& valC) {
  valA = analogRead(micA);
  delayMicroseconds(SAMPLE_DELAY);
  valB = analogRead(micB);
  delayMicroseconds(SAMPLE_DELAY);
  valC = analogRead(micC);
}

// Function to check and handle microphone triggers
void checkMicrophoneTriggers(int valA, int valB, int valC, unsigned long currentTime) {
  if (!micA_trigger && valA > TRIGGERVALUE && currentTime > micA_timestamp + DELAYTIME) {
    if (currentTime - lastPeakTime < ECHO_THRESHOLD) {
      Serial.println("Echo detected on MicA - ignoring");
      return;
    }
    int peakA = findTruePeak(micA, currentTime, micA_timestamp);
    micA_trigger = true;
    lastPeakTime = micA_timestamp;
    Serial.println("MicA Triggered! Time: " + String(micA_timestamp) + " Value: " + String(peakA));
  }
  
  if (!micB_trigger && valB > TRIGGERVALUE && currentTime > micB_timestamp + DELAYTIME) {
    if (currentTime - lastPeakTime < ECHO_THRESHOLD) {
      Serial.println("Echo detected on MicB - ignoring");
      return;
    }
    int peakB = findTruePeak(micB, currentTime, micB_timestamp);
    micB_trigger = true;
    lastPeakTime = micB_timestamp;
    Serial.println("MicB Triggered! Time: " + String(micB_timestamp) + " Value: " + String(peakB));
  }
  
  if (!micC_trigger && valC > TRIGGERVALUE && currentTime > micC_timestamp + DELAYTIME) {
    if (currentTime - lastPeakTime < ECHO_THRESHOLD) {
      Serial.println("Echo detected on MicC - ignoring");
      return;
    }
    int peakC = findTruePeak(micC, currentTime, micC_timestamp);
    micC_trigger = true;
    lastPeakTime = micC_timestamp;
    Serial.println("MicC Triggered! Time: " + String(micC_timestamp) + " Value: " + String(peakC));
  }
}

// Function to handle timeout conditions
void handleTimeout(unsigned long currentTime) {
  if ((currentTime - micA_timestamp > MAXTIME && micA_trigger) ||
      (currentTime - micB_timestamp > MAXTIME && micB_trigger) ||
      (currentTime - micC_timestamp > MAXTIME && micC_trigger)) {
    if (micA_trigger || micB_trigger || micC_trigger) {
      Serial.println("Reset due to timeout. Triggered sensors:");
      if (micA_trigger) Serial.println("MicA");
      if (micB_trigger) Serial.println("MicB");
      if (micC_trigger) Serial.println("MicC");
      
      if (micA_trigger && micB_trigger && micC_trigger) {
        processTimeoutCalculation();
      }
    }
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;
  }
}

// Function to process calculations during timeout
void processTimeoutCalculation() {
  unsigned long firstTime = min(min(micA_timestamp, micB_timestamp), micC_timestamp);
  unsigned long timeA = micA_timestamp - firstTime;
  unsigned long timeB = micB_timestamp - firstTime;
  unsigned long timeC = micC_timestamp - firstTime;
  
  Serial.println("Time differences from first trigger:");
  Serial.println("First trigger: " + String(firstTime) + " us");
  Serial.println("A: " + String(timeA) + " us");
  Serial.println("B: " + String(timeB) + " us");
  Serial.println("C: " + String(timeC) + " us");
  
  if (timeA >= MIN_TIMEDIFF || timeB >= MIN_TIMEDIFF || timeC >= MIN_TIMEDIFF) {
    Serial.println("Attempting calculation with timeout values...");
    calculateSoundSource(firstTime);
  } else {
    Serial.println("Time differences too small for calculation");
  }
}

// Function to calculate sound source when all mics are triggered
void calculateSoundSource(unsigned long firstTime) {
  unsigned long timeA = micA_timestamp - firstTime;
  unsigned long timeB = micB_timestamp - firstTime;
  unsigned long timeC = micC_timestamp - firstTime;
  // Convert time into distances
  double distA = timeA * 0.0343;
  double distB = timeB * 0.0343;
  double distC = timeC * 0.0343;
  
  if (!validateDistances(distA, distB, distC)) {
    return;
  }
  
  if (!validateTimeDifferences(timeA, timeB, timeC)) {
    return;
  }
  
  // Determine which sensor triggered first and calculate
  if (micA_timestamp == firstTime) {
    soundSource = calculate(0, timeB, timeC, TRIANGLESIDE);
  } else if (micB_timestamp == firstTime) {
    soundSource = calculate(timeA, 0, timeC, TRIANGLESIDE);
  } else {
    soundSource = calculate(timeA, timeB, 0, TRIANGLESIDE);
  }
  
  printSoundSourceResult();
}


bool validateDistances(double distA, double distB, double distC) {
  if (distA > MAX_PHYSICAL_DIST || distB > MAX_PHYSICAL_DIST || distC > MAX_PHYSICAL_DIST) {
    Serial.println("WARNING: Physical distance too large!");
    Serial.println("Max allowed: " + String(MAX_PHYSICAL_DIST) + " cm");
    Serial.println("Measured distances: " + String(distA) + ", " + String(distB) + ", " + String(distC) + " cm");
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;
    return false;
  }
  return true;
}

// Function to validate time differences
bool validateTimeDifferences(unsigned long timeA, unsigned long timeB, unsigned long timeC) {
  if (timeA > MAX_TIMEDIFF || timeB > MAX_TIMEDIFF || timeC > MAX_TIMEDIFF) {
    Serial.println("WARNING: Time differences too large! Possible echo or multiple sources.");
    Serial.println("Expected max difference: " + String(MAX_TIMEDIFF) + " us");
    Serial.println("Sensor response time: " + String(SENSOR_RESPONSE_TIME) + " us");
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;
    return false;
  }
  
  if ((timeA < MIN_TIMEDIFF && timeA > 0) || 
      (timeB < MIN_TIMEDIFF && timeB > 0) || 
      (timeC < MIN_TIMEDIFF && timeC > 0)) {
    Serial.println("WARNING: Time differences too small! Possible interference or sampling issue.");
    Serial.println("Minimum expected difference: " + String(MIN_TIMEDIFF) + " us");
    Serial.println("Sensor response time: " + String(SENSOR_RESPONSE_TIME) + " us");
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;
    return false;
  }
  return true;
}

// Function to print sound source results
void printSoundSourceResult() {
  if (soundSource.valid) {
    Serial.println("Result:");
    Serial.print("Angle: ");
    Serial.print(soundSource.angle);
    Serial.print(" degrees, Distance: ");
    Serial.print(soundSource.distance);
    Serial.println(" cm");
  } else {
    Serial.println("Calculation failed!");
  }
  Serial.println("-----------------------------");
}

// Function to handle car movement based on sound source
void handleCarMovement() {
  unsigned long currentTime = millis();
  static unsigned long pauseStartTime = 0;
  static bool isPaused = false;
  
  // Check for obstacles using front ultrasonic sensor
  int frontDistance = getDistance(frontTrig, frontEcho);
  
  if (soundSource.valid) {
    // Display results on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Angle: ");
    lcd.print(soundSource.angle, 1);
    lcd.print(" deg");
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(soundSource.distance, 1);
    lcd.print(" cm");
    
    // Start pause if not already paused
    if (!isPaused) {
      Serial.println("Valid sound source found! Pausing for 3 seconds...");
      stopMotors();
      isMoving = false;
      isPaused = true;
      pauseStartTime = currentTime; 
    }
    
    // Check if pause duration is complete
    if (isPaused && (currentTime - pauseStartTime >= 10000)) {
      Serial.println("Pause complete. Resuming normal operation.");
      isPaused = false;
    }
    
    // Only proceed with movement if not paused
    if (!isPaused) {
      if (frontDistance < safeDist) {
        Serial.println("Obstacle detected! Stopping.");
        stopMotors();
        isMoving = false;
        return;
      }
      
      // If we're not moving and we have a valid distance
      if (!isMoving && soundSource.distance > 0) {
        // Calculate movement duration based on distance and speed
        // distance in cm, speed in m/s, convert to milliseconds
        movementDuration = (soundSource.distance / 100.0) / TARGET_SPEED * 1000;
        
        Serial.println("Starting movement towards sound source");
        Serial.println("Distance: " + String(soundSource.distance) + " cm");
        Serial.println("Movement duration: " + String(movementDuration/1000.0) + " seconds");
        isMoving = true;
        movementStartTime = currentTime;
        forward();
      }
      
      // If we're moving, check if we've reached the movement duration
      if (isMoving) {
        if (currentTime - movementStartTime >= movementDuration) {
          Serial.println("Movement duration completed! Stopping.");
          stopMotors();
          isMoving = false;
        }
      }
    }
  } else {
    // Clear LCD if no valid sound source
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No valid sound");
    lcd.setCursor(0, 1);
    lcd.print("source detected");
    
    if (isMoving) {
      // If we lose the sound source while moving, stop
      Serial.println("Lost sound source. Stopping.");
      stopMotors();
      isMoving = false;
    }
    isPaused = false;  // Reset pause state if sound source becomes invalid
  }
  delay(3000);
}

void loop() {
  static bool measurementPaused = false;
  static unsigned long pauseStartTime = 0;

  if (IrReceiver.decode()) {
    uint32_t command = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(command, HEX);
    IrReceiver.resume();

    while (command != irSound) {
      if (command == irFront) {
        forward();
        int frontDist = getDistance(frontTrig, frontEcho);
        if (frontDist != -1 && frontDist < 10) {
          stopMotors();
          Serial.println("Obstacle detected!");
          Serial.print("Distance: ");
          Serial.print(frontDist);
          Serial.println(" cm");
        }
      } else if (command == irLeft) {
        left();
        int leftDist = getDistance(leftTrig, leftEcho);
        if (leftDist != -1 && leftDist < 10) {
          stopMotors();
          Serial.println("Obstacle detected!");
          Serial.print("Distance: ");
          Serial.print(leftDist);
          Serial.println(" cm");
        }
      } else if (command == irRight) {
        right();
        int rightDist = getDistance(rightTrig, rightEcho);
        if (rightDist != -1 && rightDist < 10) {
          stopMotors();
          Serial.println("Obstacle detected!");
          Serial.print("Distance: ");
          Serial.print(rightDist);
          Serial.println(" cm");
        }
      } else if (command == irStop) {
        stopMotors();
      } else if (command == irBack) {
        back();
      }

      // Listen for next IR command
      if (IrReceiver.decode()) {
        command = IrReceiver.decodedIRData.decodedRawData;
        IrReceiver.resume();
      }

      delay(100); // Avoid flooding and gives time for ultrasonic to stabilize
    }

    // Sound localization mode
    while (command != irRemote) {
      printDebug();
      unsigned long currentTime = micros();

      if (!measurementPaused) {
        int valA, valB, valC;
        readMicrophoneValues(valA, valB, valC);
        checkMicrophoneTriggers(valA, valB, valC, currentTime);
        handleTimeout(currentTime);

        if (micA_trigger && micB_trigger && micC_trigger) {
          Serial.println("All mics triggered. Calculating:");
          unsigned long firstTime = min(min(micA_timestamp, micB_timestamp), micC_timestamp);
          calculateSoundSource(firstTime);
          micA_trigger = false;
          micB_trigger = false;
          micC_trigger = false;

          if (soundSource.valid) {
            measurementPaused = true;
            pauseStartTime = millis();

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(soundSource.angle, 1);
            lcd.setCursor(0, 1);
            lcd.print(soundSource.distance, 1);
          }
        }
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Angle: ");
        lcd.print(soundSource.angle, 1);
        lcd.print(" deg");
        lcd.setCursor(0, 1);
        lcd.print("Dist: ");
        lcd.print(soundSource.distance, 1);
        lcd.print(" cm");

        if (millis() - pauseStartTime >= 10000) {
          measurementPaused = false;
        }
      }

      handleCarMovement();
      delay(5000);
      }

    Serial.println("Sound mode");
  }
}
