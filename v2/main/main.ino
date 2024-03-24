#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Period between two ToF readings, in milliseconds.
const int periodMs = 500;

// An ON event is triggered when the detected distance exceeds this measure, in millimiters.
const int triggerOnMinDistanceMm = 100;

// An OFF event is triggered when the detected distance falls below this measure, in millimiters.
// This is set lower than the ON distance on purpopse, to set some hysteresis and avoid flip-flopping
// between states.
const int triggerOffMaxDistanceMm = 50;

// Force an OFF event after this timer, in milliseconds.
// This is to avoid remaining in ON mode indefinitely and thus save power.
const int forceOffAfterMs = 1000 * 60 * 5;

int reading;

bool stateIsOn = false;

void setup()
{
  Serial.begin(9600);
  // while (!Serial);

  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (true);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, OUTPUT);

  stateIsOn = false;

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(A0, HIGH);  

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(periodMs);
}

void loop()
{
  reading = sensor.readRangeContinuousMillimeters();
  Serial.print("My reading is: ");
  Serial.println(reading);

  if (stateIsOn) {
    if (reading <= triggerOffMaxDistanceMm) {
      stateIsOn = false;
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(A0, HIGH);
    }
  } else {
    if (reading >= triggerOnMinDistanceMm) {
      stateIsOn = true;
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(A0, LOW);
    }
  }

  // digitalWrite(LED_BUILTIN, HIGH);
  
  // Serial.print(sensor.readRangeContinuousMillimeters());
  // if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  // Serial.println();
}
