#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* THINGSPEAK_API_KEY = "YOUR_THINGSPEAK_WRITE_API_KEY"; // replace

const unsigned long THINGSPEAK_INTERVAL_MS = 15000UL; // ThingSpeak free: >=15s recommended

// ---------- PIN ASSIGNMENTS (change as needed) ----------
// Ultrasonic sensor 1 (bin fill level)
const int US1_TRIG = 25;
const int US1_ECHO = 26;

// Ultrasonic sensor 2 (detect plastic presence)
const int US2_TRIG = 27;
const int US2_ECHO = 14;

// IR presence sensor (digital)
const int IR_PIN = 34; // input-only DAC pins are ok; use any digital input

// Servo for lid
const int SERVO_PIN = 13;

// Motor driver (example L298N / driver)
const int MOTOR_EN = 12; // PWM enable (speed) - can use analogWrite (ledc)
const int MOTOR_IN1 = 32;
const int MOTOR_IN2 = 33;

// Virtual terminal (Serial1) pins for Proteus virtual terminal (RX, TX)
const int VT_RX = 16;
const int VT_TX = 17;
const unsigned long SERIAL_BAUD = 9600;

// I2C LCD (address 0x27 typical)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- LOGIC THRESHOLDS ----------
const float US_MAX_DISTANCE_CM = 400.0; // max sensor range
const float BIN_MAX_DEPTH_CM = 40.0; // distance when bin considered empty (tune)
const float BIN_FULL_THRESHOLD_CM = 10.0; // when distance <= this -> bin full (tune)
const float PLASTIC_DETECT_THRESHOLD_CM = 8.0; // if US2 distance <= this -> plastic detected (tune)

// Servo positions
const int SERVO_OPEN_POS = 90;
const int SERVO_CLOSED_POS = 0;

// Motor PWM channel (ESP32 ledc)
const int MOTOR_PWM_CHANNEL = 0;
const int MOTOR_PWM_FREQ = 5000;
const int MOTOR_PWM_RES = 8; // 8-bit resolution (0-255)
const int MOTOR_SPEED = 200; // default speed (0-255)

// ---------- STATE ----------
Servo lidServo;
bool motorRunning = false;
bool lidOpen = false;
unsigned long lastThingSpeakMillis = 0;
unsigned long lastUSMeasureMillis = 0;
const unsigned long US_MEASURE_INTERVAL = 500; // ms between sensor reads

// ---------- Helpers: ultrasonic distance (cm) ----------
float measureDistanceCM(int trigPin, int echoPin, unsigned long timeout = 30000UL) {
  // 1) trigger 10us pulse
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  unsigned long duration = pulseIn(echoPin, HIGH, timeout); // microseconds
  if (duration == 0) return -1.0; // no echo
  // speed of sound ~343 m/s => 29.1 microsec per cm (round-trip 58.2)
  float distance = (duration / 2.0) / 29.1;
  return distance;
}

// ---------- Motor control ----------
void motorStartForward(int speed = MOTOR_SPEED) {
  // forward: IN1 HIGH, IN2 LOW (adjust if wiring reversed)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(MOTOR_PWM_CHANNEL, constrain(speed, 0, 255));
  motorRunning = true;
}
void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);
  motorRunning = false;
}

// ---------- Lid control ----------
void openLid() {
  lidServo.write(SERVO_OPEN_POS);
  lidOpen = true;
}
void closeLid() {
  lidServo.write(SERVO_CLOSED_POS);
  lidOpen = false;
}

// ---------- ThingSpeak update ----------
void sendThingSpeak(float binDistance, bool plasticDetected, bool lidOpenLocal, bool motorOn) {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String("http://api.thingspeak.com/update?api_key=") + THINGSPEAK_API_KEY;
  // map fields (change field numbers if you prefer)
  // field1: bin distance (cm)
  // field2: plastic present (0/1)
  // field3: lid open (0/1)
  // field4: motor running (0/1)
  url += "&field1=" + String(binDistance, 2);
  url += "&field2=" + String(plasticDetected ? 1 : 0);
  url += "&field3=" + String(lidOpenLocal ? 1 : 0);
  url += "&field4=" + String(motorOn ? 1 : 0);

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    // optional: read response
    String payload = http.getString();
    Serial1.printf("[TS] update code=%d response=%s\n", httpCode, payload.c_str());
  } else {
    Serial1.printf("[TS] update failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// ---------- Setup ----------
void setup() {
  // Serial for debug
  Serial.begin(115200);

  // Virtual terminal (Serial1) for Proteus: TX=VT_TX, RX=VT_RX
  Serial1.begin(SERIAL_BAUD, SERIAL_8N1, VT_RX, VT_TX);
  Serial1.println("ESP32 Smart Bin starting... (Virtual terminal)");

  // pins
  pinMode(US1_TRIG, OUTPUT); digitalWrite(US1_TRIG, LOW);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT); digitalWrite(US2_TRIG, LOW);
  pinMode(US2_ECHO, INPUT);
  pinMode(IR_PIN, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Setup PWM for motor enable
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_EN, MOTOR_PWM_CHANNEL);
  motorStop();

  // Servo
  lidServo.setPeriodHertz(50); // 50 Hz for servos
  lidServo.attach(SERVO_PIN);
  closeLid();

  // I2C LCD init
  Wire.begin(); // default SDA,SCL pins on many ESP32 dev boards will be used (21=SDA,22=SCL)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Smart Bin System");
  lcd.setCursor(0,1);
  lcd.print("Initializing...");

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial1.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000UL) {
    delay(250);
    Serial1.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial1.println("\nWiFi connected.");
    Serial1.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("WiFi: Connected");
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP().toString());
  } else {
    Serial1.println("\nWiFi connect failed.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("WiFi: Failed");
    lcd.setCursor(0,1);
    lcd.print("Check creds");
  }

  lastThingSpeakMillis = millis();
}

// ---------- Main loop ----------
void loop() {
  static float lastBinDistance = -1.0;
  static bool lastPlasticDetected = false;

  unsigned long now = millis();

  // 1) Periodic ultrasonic measurements
  if (now - lastUSMeasureMillis >= US_MEASURE_INTERVAL) {
    lastUSMeasureMillis = now;

    float binDist = measureDistanceCM(US1_TRIG, US1_ECHO);
    float plasticDist = measureDistanceCM(US2_TRIG, US2_ECHO);

    // sanitize
    if (binDist < 0 || binDist > US_MAX_DISTANCE_CM) {
      // invalid
      binDist = -1.0;
    }
    if (plasticDist < 0 || plasticDist > US_MAX_DISTANCE_CM) {
      plasticDist = -1.0;
    }

    // Determine plastic presence
    bool plasticPresent = (plasticDist > 0 && plasticDist <= PLASTIC_DETECT_THRESHOLD_CM);

    // Motor control based on plastic presence
    if (plasticPresent) {
      motorStartForward();
    } else {
      motorStop();
    }

    // IR sensor: open lid if person present
    int irVal = digitalRead(IR_PIN);
    // assume IR sensor outputs HIGH when person present; invert if your sensor is opposite
    if (irVal == HIGH) {
      openLid();
    } else {
      closeLid();
    }

    // Update LCD
    lcd.clear();
    lcd.setCursor(0,0);
    if (binDist > 0) {
      lcd.printf("Bin: %.1fcm ", binDist);
    } else {
      lcd.print("Bin: ---cm ");
    }
    lcd.setCursor(0,1);
    lcd.printf("P:%d L:%d M:%d", plasticPresent ? 1 : 0, lidOpen ? 1 : 0, motorRunning ? 1 : 0);

    // Virtual terminal prints (Proteus)
    Serial1.printf("BIN_DIST: %.2f cm | PLASTIC_DIST: %.2f cm | PLASTIC: %d | IR:%d | LID:%d | MOTOR:%d\n",
                   binDist, plasticDist, plasticPresent ? 1 : 0, irVal, lidOpen ? 1 : 0, motorRunning ? 1 : 0);

    // store for ThingSpeak
    lastBinDistance = binDist;
    lastPlasticDetected = plasticPresent;
  }

  // 2) ThingSpeak update every interval
  if (now - lastThingSpeakMillis >= THINGSPEAK_INTERVAL_MS) {
    lastThingSpeakMillis = now;
    sendThingSpeak(lastBinDistance, lastPlasticDetected, lidOpen, motorRunning);
  }

  // small yield
  delay(10);
}
