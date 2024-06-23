#include "Arduino.h"

// #define DEBUG 1

// Force an OFF event after this timer, in milliseconds.
// This is to avoid staying in ON mode indefinitely and thus save power.
const int FORCE_OFF_AFTER_SECONDS = 60 * 5;

const unsigned long UPDATE_INTERVAL_MS = 10;

// To avoid high frequency flip-flopping.
const unsigned long MIN_TIME_BETWEEN_TRANSITIONS_SEC = 1;

const int OUTPUT_0 = A0;
const int OUTPUT_1 = A1;
const int OUTPUT_2 = A2;

const int INPUT_0 = 2;
const int INPUT_1 = 3;
const int INPUT_2 = 4;

// The relay in use is low-trigger: we need to provide a LOW signal to
// activate the relay and thus closing the circuit.
// Mode of use should be: "normally open".
const bool LIGHT_ON = LOW;

const int SUCCESS = 0;


class FridgeDoor {
private:
  enum class State {
    OFF,
    INIT,
    DOOR_CLOSED_LIGHT_OFF,
    DOOR_OPEN_LIGHT_ON,
    DOOR_OPEN_LIGHT_OFF
  };

public:
  // Multiple input sensors can be used on the same input pin, as long as they are
  // connected in or-wiring, in parallel with a pull-up resistor.
  // This works on the assumption that IR sensors work in open-drain mode (no detection: open pin,
  // detection: drive to GND).
  FridgeDoor(const int inputPin, const int outputPin, const int forceOffAfterSeconds, const int minSecondsBetweenTransitions);

  // Performs an update cycle.
  int update();

  // Turns this entitiy off. Cannot be undone.
  void turnOff();

private:
  int stopTimer();
  int restartTimer();
  bool isStateTransitionAllowed() const;
  bool isDoorOpen() const;
  bool isTimerExpired() const;
  int switchLamp(const bool switchOn) const;

  const int inputPin_;
  const int outputPin_;
  const unsigned long forceOffAfterMs_;
  const unsigned long minTimeBetweenTransitionsMs_;

  State state_ = State::INIT;

  bool timerIsRunning_ = false;
  unsigned long timerStartedAtMs_ = 0;
  unsigned long lastTransitionAtMs_ = 0;
};

const int numElements = 3;
FridgeDoor fridgeDoors[numElements] = {
  FridgeDoor(INPUT_0, OUTPUT_0, FORCE_OFF_AFTER_SECONDS, MIN_TIME_BETWEEN_TRANSITIONS_SEC),
  FridgeDoor(INPUT_1, OUTPUT_1, FORCE_OFF_AFTER_SECONDS, MIN_TIME_BETWEEN_TRANSITIONS_SEC),
  FridgeDoor(INPUT_2, OUTPUT_2, FORCE_OFF_AFTER_SECONDS, MIN_TIME_BETWEEN_TRANSITIONS_SEC),
};

void blinkError(const int value) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  while(true) {
    // Blink N times, with N the error value.
    for (int i = 0; i < value; ++i) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }

    // Stay off for a longer period.
    delay(3000);
  }
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
  for (int i = 0; i < numElements; ++i) {
#ifdef DEBUG
    Serial.print("FridgeDoor index: ");
    Serial.println(i);
#endif

    error = fridgeDoors[i].update();
    if (error != SUCCESS) {
#ifdef DEBUG
      Serial.print("  Failed with error code: ");
      Serial.println(error);
#endif
      // Turn all the FridgeDoors off.
      for (int j = 0; j < numElements; ++j) {
        fridgeDoors[j].turnOff();
      }

      blinkError(i + 1);
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

FridgeDoor::FridgeDoor(
  const int inputPin,
  const int outputPin,
  const int forceOffAfterSeconds,
  const int minSecondsBetweenTransitions)
:inputPin_(inputPin)
,outputPin_(outputPin)
,forceOffAfterMs_((unsigned long)max(0, forceOffAfterSeconds) * 1000) 
,minTimeBetweenTransitionsMs_((unsigned long)max(0, minSecondsBetweenTransitions) * 1000) {
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  switchLamp(false);
}

int FridgeDoor::stopTimer() {
  timerIsRunning_ = false;
  timerStartedAtMs_ = -1;
  return SUCCESS;
}

int FridgeDoor::restartTimer() {
  timerIsRunning_ = true;
  timerStartedAtMs_ = millis();
  return SUCCESS;
}

bool FridgeDoor::isDoorOpen() const {
  // When the door is open, signal is no longer detected and the sensor output is driven to HIGH by the pull-up resistor.
  // This activates the NPN-BJT, which drives the output to HIGH.
  // In short: high input means open door.
  return digitalRead(inputPin_);
}

bool FridgeDoor::isTimerExpired() const {
  if (!timerIsRunning_) {
    return false;
  }

  if (forceOffAfterMs_ == 0) {
    // No timer is used.
    return false;
  }

  const unsigned long elapsedTimeMs = millis() - timerStartedAtMs_;
  return elapsedTimeMs > forceOffAfterMs_;
}

int FridgeDoor::switchLamp(const bool switchOn) const {
  if (switchOn) {
    digitalWrite(outputPin_, LIGHT_ON);
  } else {
    digitalWrite(outputPin_, !LIGHT_ON);
  }
  return SUCCESS;
}

bool FridgeDoor::isStateTransitionAllowed() const {
  return (millis() - lastTransitionAtMs_) > minTimeBetweenTransitionsMs_;
}

void FridgeDoor::turnOff() {
  switchLamp(false);
  stopTimer();
}

int FridgeDoor::update() {
#ifdef DEBUG
  Serial.print("  Current state: ");
  switch (state_) {
    case State::OFF:
      Serial.println("OFF");
      break;
    case State::INIT:
      Serial.println("INIT");
      break;
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
#endif

  int error;

  const bool isTransitionAllowed = isStateTransitionAllowed();
  if (!isTransitionAllowed) {
#ifdef DEBUG
  Serial.println("Too early to perform a state transition");
#endif
    return SUCCESS;
  }

  const bool doorIsOpen = isDoorOpen();
  const bool timerExpired = isTimerExpired();

#ifdef DEBUG
  Serial.print("  Door is open: ");
  Serial.println(doorIsOpen);

  Serial.print("  Timer expired: ");
  Serial.println(timerExpired);
#endif

  switch (state_) {
    case State::OFF:
      return SUCCESS;
      break;

    case State::INIT:
      if (doorIsOpen) {
        switchLamp(true);
        error = restartTimer();
        if (error != SUCCESS) return error;
        state_ = State::DOOR_OPEN_LIGHT_ON;
        lastTransitionAtMs_ = millis();
      } else {
        switchLamp(false);
        error = stopTimer();
        if (error != SUCCESS) return error;
        state_ = State::DOOR_CLOSED_LIGHT_OFF;
        lastTransitionAtMs_ = millis();
      }
      break;

    case State::DOOR_CLOSED_LIGHT_OFF:
      if (doorIsOpen) {
        switchLamp(true);
        error = restartTimer();
        if (error != SUCCESS) return error;
        state_ = State::DOOR_OPEN_LIGHT_ON;
        lastTransitionAtMs_ = millis();
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
        state_ = State::DOOR_CLOSED_LIGHT_OFF;
        lastTransitionAtMs_ = millis();
#ifdef DEBUG
        Serial.println("  DOOR_OPEN_LIGHT_ON -> DOOR_CLOSED_LIGHT_OFF");
#endif
      } else if (timerExpired) {
        switchLamp(false);
        state_ = State::DOOR_OPEN_LIGHT_OFF;
        lastTransitionAtMs_ = millis();
#ifdef DEBUG
        Serial.println("  DOOR_OPEN_LIGHT_ON -> DOOR_OPEN_LIGHT_OFF");
#endif
      }
      break;
    
    case State::DOOR_OPEN_LIGHT_OFF:
      if (!doorIsOpen) {
        error = stopTimer();
        if (error != SUCCESS) return error;
        state_ = State::DOOR_CLOSED_LIGHT_OFF;
        lastTransitionAtMs_ = millis();
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
