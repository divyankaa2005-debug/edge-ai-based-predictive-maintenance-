#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

Adafruit_ADXL345_Unified accel(12345);

const int ACS_PIN = 34;      // ACS712 OUT through 10k/15k divider
const int BUZZER_PIN = 25;   // Buzzer signal

const float ADC_VREF = 3.3f;
const float ADC_MAX = 4095.0f;

// Divider: ACS712 OUT -- 10k -- GPIO34 -- 15k -- GND
const float DIVIDER_RATIO = 25.0f / 15.0f;

// ACS712 sensitivity:
// 5A module  = 0.185 V/A
// 20A module = 0.100 V/A
// 30A module = 0.066 V/A
const float ACS_SENSITIVITY = 0.100f;  // Change to match your module

const float WARNING_THRESHOLD = 0.65f;
const float FAULT_THRESHOLD = 0.90f;

const int SAMPLE_COUNT = 100;

float currentZeroVoltage = 1.5f;
float vibrationBaseline = 0.0f;
float currentBaseline = 0.0f;
int baselineReadings = 0;

float readACS712Voltage() {
  float adcVoltage = analogRead(ACS_PIN) * ADC_VREF / ADC_MAX;
  return adcVoltage * DIVIDER_RATIO;
}

float readMotorCurrent() {
  float voltageSum = 0.0f;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    voltageSum += readACS712Voltage();
    delay(2);
  }

  float sensorVoltage = voltageSum / SAMPLE_COUNT;
  float current = (sensorVoltage - currentZeroVoltage) / ACS_SENSITIVITY;

  return fabs(current);
}

float readVibrationRMS() {
  double sumOfSquares = 0.0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sensors_event_t event;
    accel.getEvent(&event);

    // Approximate gravity removal for a sensor mounted with Z pointing up.
    float x = event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z - 9.80665f;

    sumOfSquares += x * x + y * y + z * z;
    delay(10);
  }

  return sqrt(sumOfSquares / SAMPLE_COUNT);
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  // ADXL345 I2C wiring: SDA -> GPIO21, SCL -> GPIO22
  Wire.begin(21, 22);

  if (!accel.begin()) {
    Serial.println("ERROR: ADXL345 not found. Check its power, ground, SDA and SCL.");
    while (true) {
      delay(1000);
    }
  }

  accel.setRange(ADXL345_RANGE_4_G);

  Serial.println("Keep motor OFF. Calibrating ACS712 for 3 seconds...");
  delay(3000);

  double voltageSum = 0.0;
  for (int i = 0; i < 500; i++) {
    voltageSum += readACS712Voltage();
    delay(4);
  }

  currentZeroVoltage = voltageSum / 500.0;

  Serial.print("ACS712 zero voltage: ");
  Serial.println(currentZeroVoltage, 3);
  Serial.println("Start the motor. The next 10 readings set the baseline.");
}

void loop() {
  float vibration = readVibrationRMS();
  float current = readMotorCurrent();

  // The first 10 readings become the normal operating baseline.
  if (baselineReadings < 10) {
    vibrationBaseline += vibration / 10.0f;
    currentBaseline += current / 10.0f;
    baselineReadings++;

    Serial.print("BASELINE,");
    Serial.print(baselineReadings);
    Serial.print(",");
    Serial.print(vibration, 3);
    Serial.print(",");
    Serial.println(current, 3);

    delay(500);
    return;
  }

  float vibrationRatio = vibration / max(vibrationBaseline, 0.15f);
  float currentRatio = current / max(currentBaseline, 0.10f);

  // Combined anomaly score, limited to the range 0.0–1.0.
  float score =
      0.65f * min(vibrationRatio / 2.0f, 1.0f) +
      0.35f * min(currentRatio / 2.0f, 1.0f);

  const char* status;

  if (score < WARNING_THRESHOLD) {
    status = "HEALTHY";
  } else if (score < FAULT_THRESHOLD) {
    status = "WARNING";
  } else {
    status = "FAULT";
  }

  digitalWrite(BUZZER_PIN, score >= FAULT_THRESHOLD ? HIGH : LOW);

  Serial.print("READING,");
  Serial.print(vibration, 3);
  Serial.print(",");
  Serial.print(current, 3);
  Serial.print(",");
  Serial.print(score, 3);
  Serial.print(",");
  Serial.println(status);

  delay(1000);
}
