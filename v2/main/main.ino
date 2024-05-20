#include "Arduino.h"

#define DEBUG 1

// Force an OFF event after this timer, in milliseconds.
// This is to avoid staying in ON mode indefinitely and thus save power.
const int FORCE_OFF_AFTER_SECONDS = 20;

const unsigned long UPDATE_INTERVAL_MS = 1000;

const int OUTPUT_0 = A0;
const int OUTPUT_1 = A1;
const int OUTPUT_2 = A2;

const int INPUT_0 = 2;
const int INPUT_1 = 3;
const int INPUT_2 = 4;

const bool LIGHT_ON = LOW;

const int SUCCESS = 0;


class FridgeDoor {
private:
  enum class State {
    DOOR_CLOSED_LIGHT_OFF,
    DOOR_OPEN_LIGHT_ON,
    DOOR_OPEN_LIGHT_OFF
  };

public:
  // Multiple input sensors can be used on the same input pin, as long as they are
  // connected in or-wiring, in parallel with a pull-up resistor.
  // This works on the assumption that IR sensors work in open-drain mode (no detection: open pin,
  // detection: drive to GND).
  FridgeDoor(const int inputPin, const int outputPin, const int forceOffAfterSeconds);

  // Performs an update cycle.
  int update();

private:
  int stopTimer();
  int restartTimer();
  bool isDoorOpen() const;
  bool isTimerExpired() const;
  int switchLamp(const bool switchOn);

  const int inputPin;
  const int outputPin;
  const unsigned long forceOffAfterMs;

  State state = State::DOOR_CLOSED_LIGHT_OFF;

  bool timerIsRunning = false;
  unsigned long timerStartedAtMs = 0;
};

const int numElements = 1;
FridgeDoor fridgeDoors[numElements] = {
  FridgeDoor(INPUT_0, OUTPUT_0, FORCE_OFF_AFTER_SECONDS),
  // FridgeDoor(INPUT_1, OUTPUT_1, FORCE_OFF_AFTER_SECONDS),
  // FridgeDoor(INPUT_2, OUTPUT_2, FORCE_OFF_AFTER_SECONDS),
};

void trap() {
  while(true);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial);
#endif
}

void loop()
{
#ifdef DEBUG
  Serial.println("##########");
#endif

  int error;
  FridgeDoor *fridgeDoor;
  for (int i = 0; i < numElements; ++i) {
#ifdef DEBUG
    Serial.print("FridgeDoor index: ");
    Serial.println(i);
#endif

    error = fridgeDoor[i].update();
    if (error != SUCCESS) {
#ifdef DEBUG
      Serial.print("  Failed with error code: ");
      Serial.println(error);
#endif
      trap();
    }
#ifdef DEBUG
    else {
      Serial.println("  Successful update");
    }
#endif
  }

  delay(UPDATE_INTERVAL_MS);

#ifdef DEBUG
  Serial.println();
#endif
}

FridgeDoor::FridgeDoor(const int inputPin, const int outputPin, const int forceOffAfterSeconds)
:inputPin(inputPin)
,outputPin(outputPin)
,forceOffAfterMs(max(0, forceOffAfterSeconds) * 1000) {
  pinMode(inputPin, INPUT_PULLUP);
  pinMode(outputPin, OUTPUT);
  switchLamp(false);
}

int FridgeDoor::stopTimer() {
  timerIsRunning = false;
  timerStartedAtMs = -1;
  return SUCCESS;
}

int FridgeDoor::restartTimer() {
  timerIsRunning = true;
  timerStartedAtMs = millis();
  return SUCCESS;
}

bool FridgeDoor::isDoorOpen() const {
  // When the door is open, signal is no longer detected and the sensor output is driven to HIGH by the pull-up resistor.
  return digitalRead(inputPin);
}

bool FridgeDoor::isTimerExpired() const {
  if (!timerIsRunning) {
    return false;
  }

  if (forceOffAfterMs == 0) {
    // No timer is used.
    return false;
  }

  const unsigned long elapsedTimeMs = millis() - timerStartedAtMs;
  return elapsedTimeMs > forceOffAfterMs;
}

int FridgeDoor::switchLamp(const bool switchOn) {
  if (switchOn) {
    digitalWrite(outputPin, LIGHT_ON);
  } else {
    digitalWrite(outputPin, !LIGHT_ON);
  }
  return SUCCESS;
}

int FridgeDoor::update() {
  int error;

  const bool doorIsOpen = isDoorOpen();
  const bool timerExpired = isTimerExpired();

#ifdef DEBUG
  Serial.print("  Current state: ");
  switch (state) {
    case State::DOOR_CLOSED_LIGHT_OFF:
      Serial.println("DOOR_CLOSED_LIGHT_OFF");
      break;
    case State::DOOR_OPEN_LIGHT_ON:
      Serial.println("DOOR_OPEN_LIGHT_ON");
      break;
    case State::DOOR_OPEN_LIGHT_OFF:
      Serial.println("DOOR_OPEN_LIGHT_OFF");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }

  Serial.print("  Door is open: ");
  Serial.println(doorIsOpen);

  Serial.print("  Timer expired: ");
  Serial.println(timerExpired);
#endif

  switch (state) {
    case State::DOOR_CLOSED_LIGHT_OFF:
      if (doorIsOpen) {
        switchLamp(true);
        error = restartTimer();
        if (error != SUCCESS) return error;
        state = State::DOOR_OPEN_LIGHT_ON;
#ifdef DEBUG
        Serial.println("  DOOR_CLOSED_LIGHT_OFF -> DOOR_OPEN_LIGHT_ON");
#endif
      }
      break;
    
    case State::DOOR_OPEN_LIGHT_ON:
      if (!doorIsOpen) {
        switchLamp(false);
        error = stopTimer();
        if (error != SUCCESS) return error;
        state = State::DOOR_CLOSED_LIGHT_OFF;
#ifdef DEBUG
        Serial.println("  DOOR_OPEN_LIGHT_ON -> DOOR_CLOSED_LIGHT_OFF");
#endif
      } else if (timerExpired) {
        switchLamp(false);
        state = State::DOOR_OPEN_LIGHT_OFF;
#ifdef DEBUG
        Serial.println("  DOOR_OPEN_LIGHT_ON -> DOOR_OPEN_LIGHT_OFF");
#endif
      }
      break;
    
    case State::DOOR_OPEN_LIGHT_OFF:
      if (!doorIsOpen) {
        error = stopTimer();
        if (error != SUCCESS) return error;
        state = State::DOOR_CLOSED_LIGHT_OFF;
#ifdef DEBUG
        Serial.println("  DOOR_OPEN_LIGHT_OFF -> DOOR_CLOSED_LIGHT_OFF");
#endif
      }
      break;
    
    default:
      break;
  }

  return SUCCESS;
}
