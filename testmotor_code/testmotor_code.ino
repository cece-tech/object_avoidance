// === Pin Definitions ===
#define M1_PWM 5   // Right Front
#define M1_DIR 4
#define M2_PWM 6   // Right Rear
#define M2_DIR 7
#define M3_PWM 9   // Left Front
#define M3_DIR 8
#define M4_PWM 10  // Left Rear

#define M4_DIR 11

#define TRIG_PIN 12
#define ECHO_PIN 13
#define LED_PIN LED_BUILTIN

// === Ultrasonic Class ===
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

// === Motor Controller for L293D ===
class MotorController {
public:
  MotorController(uint8_t pwm1, uint8_t dir1,
                  uint8_t pwm2, uint8_t dir2,
                  uint8_t pwm3, uint8_t dir3,
                  uint8_t pwm4, uint8_t dir4)
    : m1_pwm(pwm1), m1_dir(dir1),
      m2_pwm(pwm2), m2_dir(dir2),
      m3_pwm(pwm3), m3_dir(dir3),
      m4_pwm(pwm4), m4_dir(dir4) {}

  void begin() {
    pinMode(m1_pwm, OUTPUT); pinMode(m1_dir, OUTPUT);
    pinMode(m2_pwm, OUTPUT); pinMode(m2_dir, OUTPUT);
    pinMode(m3_pwm, OUTPUT); pinMode(m3_dir, OUTPUT);
    pinMode(m4_pwm, OUTPUT); pinMode(m4_dir, OUTPUT);
    stop();
  }

  void setSpeed(uint8_t speed) {
    m_speed = speed;
  }

  void moveForward() {
    digitalWrite(m1_dir, HIGH); analogWrite(m1_pwm, m_speed);
    digitalWrite(m2_dir, HIGH); analogWrite(m2_pwm, m_speed);
    digitalWrite(m3_dir, LOW);  analogWrite(m3_pwm, m_speed);
    digitalWrite(m4_dir, LOW);  analogWrite(m4_pwm, m_speed);
  }

  void moveBackward() {
    digitalWrite(m1_dir, LOW);  analogWrite(m1_pwm, m_speed);
    digitalWrite(m2_dir, LOW);  analogWrite(m2_pwm, m_speed);
    digitalWrite(m3_dir, HIGH); analogWrite(m3_pwm, m_speed);
    digitalWrite(m4_dir, HIGH); analogWrite(m4_pwm, m_speed);
  }

  void rotateCW() {
    digitalWrite(m1_dir, HIGH); analogWrite(m1_pwm, m_speed);
    digitalWrite(m2_dir, HIGH); analogWrite(m2_pwm, m_speed);
    digitalWrite(m3_dir, HIGH); analogWrite(m3_pwm, m_speed);
    digitalWrite(m4_dir, HIGH); analogWrite(m4_pwm, m_speed);
  }

  void rotateCCW() {
    digitalWrite(m1_dir, LOW); analogWrite(m1_pwm, m_speed);
    digitalWrite(m2_dir, LOW); analogWrite(m2_pwm, m_speed);
    digitalWrite(m3_dir, LOW); analogWrite(m3_pwm, m_speed);
    digitalWrite(m4_dir, LOW); analogWrite(m4_pwm, m_speed);
  }

  void stop() {
    analogWrite(m1_pwm, 0);
    analogWrite(m2_pwm, 0);
    analogWrite(m3_pwm, 0);
    analogWrite(m4_pwm, 0);
  }

private:
  uint8_t m1_pwm, m1_dir, m2_pwm, m2_dir, m3_pwm, m3_dir, m4_pwm, m4_dir;
  uint8_t m_speed = 180;
};

// === Robot Class ===
class MecanumRobot {
public:
  MecanumRobot(MotorController &mc, Ultrasonic &ultra, uint8_t ledPin)
    : motors(mc), sonar(ultra), led(ledPin) {}

  void begin() {
    motors.begin();
    sonar.begin();
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
  }

  void obstacleAvoidanceStep() {
    float front = sonar.distanceCM();
    Serial.print("Front: "); Serial.print(front); Serial.println(" cm");

    if (front > 0 && front <= 18.0) {
      digitalWrite(led, HIGH);
      Serial.println("Obstacle detected! Backing up...");
      motors.stop();
      delay(100);
      motors.moveBackward();
      delay(400);
      motors.stop();
      delay(100);
      // simple rotation
      motors.rotateCW();
      delay(400);
      motors.stop();
      digitalWrite(led, LOW);
    } else {
      motors.moveForward();
    }
  }

private:
  MotorController &motors;
  Ultrasonic &sonar;
  uint8_t led;
};

// === Global objects ===
Ultrasonic sonar(TRIG_PIN, ECHO_PIN);
MotorController mc(M1_PWM, M1_DIR, M2_PWM, M2_DIR, M3_PWM, M3_DIR, M4_PWM, M4_DIR);
MecanumRobot robot(mc, sonar, LED_PIN);

void setup() {
  Serial.begin(115200);
  robot.begin();
  Serial.println("Obstacle Avoidance Robot Ready!");
}

void loop() {
  robot.obstacleAvoidanceStep();
  delay(50);
}
