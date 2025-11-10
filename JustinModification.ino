#include <Servo.h>

// === Pin Definitions ===
#define MOTOR_SERVO_PIN 9   // servo
#define PWM1_PIN 5
#define PWM2_PIN 6
#define SHCP_PIN 2   // shift register clock (SRCLK)
#define STCP_PIN 4   // latch (RCLK)
#define DATA_PIN 8   // data (SER)
#define EN_PIN 7     // enable pin
#define TRIG_PIN 10
#define ECHO_PIN 1  
#define LED_PIN LED_BUILTIN

// === Motor bit-mapping (X-Configuration) ===
// bit0: M4_FWD  (Left Rear   - M4)
// bit1: M4_REV
// bit2: M3_FWD  (Left Front  - M3)
// bit3: M3_REV
// bit4: M2_FWD  (Right Rear  - M2)
// bit5: M2_REV
// bit6: M1_FWD  (Right Front - M1)
// bit7: M1_REV

class Shift595 {
public:
  Shift595(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin) 
    : _data(dataPin), _clock(clockPin), _latch(latchPin), _outByte(0) {}

  void begin() {
    pinMode(_data, OUTPUT);
    pinMode(_clock, OUTPUT);
    pinMode(_latch, OUTPUT);
    digitalWrite(_data, LOW);
    digitalWrite(_clock, LOW);
    digitalWrite(_latch, LOW);
    write(0);
  }

  void write(uint8_t b) {
    _outByte = b;
    digitalWrite(_latch, LOW);
    shiftOut(_data, _clock, MSBFIRST, _outByte);
    digitalWrite(_latch, HIGH);
  }

  uint8_t read() { return _outByte; }

private:
  uint8_t _data, _clock, _latch;
  uint8_t _outByte;
};

class Ultrasonic {
public:
  Ultrasonic(uint8_t trigPin, uint8_t echoPin) : _trig(trigPin), _echo(echoPin) {}

  void begin() {
    pinMode(_trig, OUTPUT);
    pinMode(_echo, INPUT);
    digitalWrite(_trig, LOW);
  }

  float distanceCM() {
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    unsigned long duration = pulseIn(_echo, HIGH, 30000UL);
    if (duration == 0) return 0.0f;

    float dist = (duration * 0.0343f) / 2.0f;
    if (dist < 2.0f || dist > 400.0f) return 0.0f;
    return dist;
  }

private:
  uint8_t _trig, _echo;
};

class MotorController {
public:
  MotorController(Shift595 &sr, uint8_t pwm1, uint8_t pwm2, uint8_t enPin)
    : _sr(sr), _pwm1(pwm1), _pwm2(pwm2), _en(enPin) {}

  void begin() {
    pinMode(_pwm1, OUTPUT);
    pinMode(_pwm2, OUTPUT);
    pinMode(_en, OUTPUT);
    digitalWrite(_en, LOW);
    stop();
  }

  void setWheel(int wheel, int dir) {
    uint8_t bitF = wheelBitForward(wheel);
    uint8_t bitR = wheelBitReverse(wheel);
    uint8_t b = _sr.read();
    b &= ~((1 << bitF) | (1 << bitR));
    if (dir == 1) {
      b |= (1 << bitF);
    } else if (dir == -1) {
      b |= (1 << bitR);
    }
    _sr.write(b);
  }

  void setWheels(int d1, int d2, int d3, int d4) {
    uint8_t b = 0;
    if (d1 == 1) b |= (1 << wheelBitForward(1));
    else if (d1 == -1) b |= (1 << wheelBitReverse(1));

    if (d2 == 1) b |= (1 << wheelBitForward(2));
    else if (d2 == -1) b |= (1 << wheelBitReverse(2));

    if (d3 == 1) b |= (1 << wheelBitForward(3));
    else if (d3 == -1) b |= (1 << wheelBitReverse(3));

    if (d4 == 1) b |= (1 << wheelBitForward(4));
    else if (d4 == -1) b |= (1 << wheelBitReverse(4));
    _sr.write(b);
  }

  void setSpeed(uint8_t speed) {
    analogWrite(_pwm1, speed);
    analogWrite(_pwm2, speed);
  }

  void stop() {
    _sr.write(0);
    analogWrite(_pwm1, 0);
    analogWrite(_pwm2, 0);
  }

private:
  Shift595 &_sr;
  uint8_t _pwm1, _pwm2, _en;

  uint8_t wheelBitForward(int wheel) {
    switch (wheel) {
      case 1: return 6; // M1_FWD (Right Front)
      case 2: return 4; // M2_FWD (Right Rear)
      case 3: return 2; // M3_FWD (Left Front)
      case 4: return 0; // M4_FWD (Left Rear - swapped)
      default: return 0;
    }
  }
  uint8_t wheelBitReverse(int wheel) {
    switch (wheel) {
      case 1: return 7; // M1_REV (Right Front)
      case 2: return 5; // M2_REV (Right Rear)
      case 3: return 3; // M3_REV (Left Front)
      case 4: return 1; // M4_REV (Left Rear - swapped)
      default: return 1;
    }
  }
};

class ServoScanner {
public:
  ServoScanner(uint8_t pin) : _pin(pin) {}

  void begin() {
    _servo.attach(_pin);
    center();
  }

  void write(uint8_t angle) {
    _servo.write(angle);
  }
  void center() { write(90); }
  void left()   { write(150); }
  void right()  { write(30); }

private:
  uint8_t _pin;
  Servo _servo;
};

class MecanumRobot {
public:
  MecanumRobot(MotorController &mc, Ultrasonic &ultra, ServoScanner &scan, uint8_t ledPin)
    : motors(mc), sonar(ultra), scanner(scan), led(ledPin), speed(180) {}

  void begin() {
    motors.begin();
    sonar.begin();
    scanner.begin();
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
    motors.setSpeed(speed);
  }

  // Movement Patterns (matching reference code logic)
  // M1=RF, M2=RR, M3=LF, M4=LR
  // Left side = M3+M4, Right side = M1+M2
  
  void moveForward() {
    // Left Forward + Right Forward (all wheels forward)
    motors.setWheels(1, 1, 1, 1);  // M1, M2, M3, M4 all forward
    motors.setSpeed(speed);
  }

  void moveBackward() {
    // Left Backward + Right Backward (all wheels backward)
    motors.setWheels(-1, -1, -1, -1);  // M1, M2, M3, M4 all backward
    motors.setSpeed(speed);
  }

  void rotateCW() {
    // Turn Right: Left Forward + Right Backward
    motors.setWheels(-1, -1, 1, 1);  // Right side back, Left side forward
    motors.setSpeed(speed);
  }

  void rotateCCW() {
    // Turn Left: Left Backward + Right Forward
    motors.setWheels(1, 1, -1, -1);  // Right side forward, Left side back
    motors.setSpeed(speed);
  }

  void stop() {
    motors.stop();
  }

  void obstacleAvoidanceStep() {
    float front = sonar.distanceCM();
    Serial.print("Front: "); Serial.print(front); Serial.println(" cm");

    if (front == 0) {
      Serial.println("Sensor error, stopping.");
      digitalWrite(led, HIGH);
      stop();
      digitalWrite(led, LOW);
      delay(50);
    } else if (front <= 18.0) {
      digitalWrite(led, HIGH);
      Serial.println("Obstacle detected! Avoiding...");
      stop();
      delay(150);
      
      // Back up
      moveBackward();
      delay(350);
      stop(); 
      delay(120);
      
      // Scan right
      scanner.right(); 
      delay(350);
      float rightDist = sonar.distanceCM();
      Serial.print("Right: "); Serial.print(rightDist); Serial.println(" cm");
      
      // Scan left
      scanner.left(); 
      delay(350);
      float leftDist = sonar.distanceCM();
      Serial.print("Left: "); Serial.print(leftDist); Serial.println(" cm");
      
      scanner.center(); 
      delay(200);
      
      if (rightDist >= leftDist) {
        Serial.println("Turning right");
        rotateCW();
        delay(450);
      } else {
        Serial.println("Turning left");
        rotateCCW();
        delay(450);
      }
      stop();
      digitalWrite(led, LOW);
      delay(50);
    } else {
      // Path clear - keep moving forward
      Serial.println("Path clear - moving forward");
      moveForward();
      delay(100);
    }
  }

private:
  MotorController &motors;
  Ultrasonic &sonar;
  ServoScanner &scanner;
  uint8_t led;
  uint8_t speed;
};

// === Global objects ===
Shift595 sr(DATA_PIN, SHCP_PIN, STCP_PIN);
Ultrasonic sonar(TRIG_PIN, ECHO_PIN);
MotorController mc(sr, PWM1_PIN, PWM2_PIN, EN_PIN);
ServoScanner scanner(MOTOR_SERVO_PIN);
MecanumRobot robot(mc, sonar, scanner, LED_PIN);


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== Mecanum X-Config Obstacle Avoidance ==="));
  Serial.println(F("Starting components..."));
  sr.begin();
  mc.begin();
  sonar.begin();
  scanner.begin();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("System ready - starting obstacle avoidance mode"));
  delay(800);

  
}

void loop() {
  robot.obstacleAvoidanceStep();
}
