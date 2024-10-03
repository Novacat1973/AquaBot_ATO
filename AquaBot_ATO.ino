// Implement a state management for pump states
// Implement error handlings
// Reduce Code Duplication
// Add function documentation to each function header: explaining its purpose, parameters and return values
// Integration of the RGB sensor LED via LightWS2812 library



// Pin assignments
const int sensorPinInput = A1;  // A1 for water sensor input
const int mosfetPin = 2;        // D2 for MOSFET control
const int buttonPin = 4;        // D4 for button input

// Time intervals and thresholds (in milliseconds)
const unsigned long readingInterval = 30000;  // Time interval for checking water level
const int stableReadingThreshold = 350;       // Analog reading threshold indicating water contact
const unsigned long stableDuration = 2000;    // Duration for stable sensor reading to confirm water level
const float maxPumpDurationFactor = 1.1;      // Safety factor: max pump duration increase for overflow prevention

// Button timing thresholds
const unsigned long debounceDelay = 50;           // Debounce delay in milliseconds
const unsigned long shortPressDuration = 350;     // Maximum duration for a short press (in ms)
const unsigned long doublePressInterval = 400;    // Maximum interval between two short presses (in ms)
const unsigned long longPressDuration = 1000;     // Minimum duration for a long press (in ms)
const unsigned long pressAndHoldDuration = 2000;  // Minimum duration for press and hold (2 seconds)

// Variables for tracking time and readings
unsigned long previousCheckMillis = 0;   // Last time water level was checked
unsigned long pumpStartMillis = 0;       // Time when the pump was activated
unsigned long previousFillDuration = 0;  // Duration of the last filling
unsigned long lastButtonPressTime = 0;   // Time of the last button press
unsigned long buttonPressStartTime = 0;  // Time when the button was first pressed
unsigned long lastShortPressTime = 0;    // Time of the last short press

// Flags
bool initialRun = true;    // Flag to indicate first run since power-up
bool alarmRaised = false;  // Flag to indicate if the alarm has been raised due to long pump runtime
bool isPriming = false;    // Flag to indicate if the pump is in priming mode
bool isRunning = false;    // Flag to indicatenif the pump is running

// Button state variables
bool buttonState = HIGH;      // Current button state
bool lastButtonState = HIGH;  // Previous button state
enum ButtonPressType {        // Enum to define button press types
  NONE,
  SHORT_PRESS,
  DOUBLE_SHORT_PRESS,
  LONG_PRESS,
  PRESS_AND_HOLD
};
ButtonPressType buttonPressType = NONE;  // Variable to store detected press type

enum ButtonState {  // Button states
  IDLE,
  DEBOUNCING,
  PRESSED,
  WAIT_DOUBLE_PRESS,
  HOLDING
};
ButtonState buttonStateMachine = IDLE;  // Button state machine state

// Function prototypes
void checkWaterLevel();
void activatePump();
void deactivatePump();
void monitorPumpDuringFilling();
void checkButtonPress();
void handleButtonPress();
void processButtonPress(ButtonPressType pressType);
bool isLongPress(unsigned long pressStartTime);


void setup() {
  // Set pin modes
  pinMode(sensorPinInput, INPUT);
  pinMode(mosfetPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Button with internal pull-up resistor

  // Initialize states
  digitalWrite(mosfetPin, LOW);  // Set MOSFET initially off
  Serial.begin(9600);            // Initialize serial for debugging
}


void loop() {

  // Override all monitoring actions if the button is in PRESS_AND_HOLD mode (priming the pump)
  if (!isPriming) {
    if (!isRunning) {
      // If the pump is not running, check water level at regular intervals
      checkWaterLevel();
    } else {
      // If the pump is running, monitor its duration to avoid overflow
      monitorPumpDuringFilling();
    }
  }

  // Continuously check for button presses
  checkButtonPress();

  // Process button press types based on detected press type
  handleButtonPress();
}


// Function to check the water level at set intervals
void checkWaterLevel() {
  unsigned long currentMillis = millis();

  // Check if the reading interval has passed
  if (currentMillis - previousCheckMillis >= readingInterval) {
    previousCheckMillis = currentMillis;

    // Read sensor value and monitor for stability over stableDuration time
    unsigned long stableStart = millis();
    bool stateChanged = false;                     // Flag to track if the state changes during stableDuration
    bool initialState;                             // Initial state of the sensor reading (high or low)
    int sensorValue = analogRead(sensorPinInput);  // First reading

    // Determine initial state based on the first reading
    if (sensorValue >= stableReadingThreshold) {
      initialState = true;  // Water is detected
    } else if (sensorValue < stableReadingThreshold) {
      initialState = false;  // No water detected
    } else {
      return;  // Exit if the first reading is not clearly high or low (fluctuating)
    }

    // Continuously check sensor values over stableDuration
    while (millis() - stableStart <= stableDuration) {
      sensorValue = analogRead(sensorPinInput);

      // Determine if the state has changed
      if ((sensorValue >= stableReadingThreshold && initialState == false) || (sensorValue < stableReadingThreshold && initialState == true)) {
        stateChanged = true;  // State changed, exit early
        break;
      }
    }

    // If no state changes were detected, consider the reading stable
    if (!stateChanged) {
      if (initialState) {
        Serial.println("Water level is stable and high.");
        // Water is detected, no action needed
      } else {
        Serial.println("Water level is low, activating pump.");
        activatePump();  // Water is low, activate the pump
      }
    } else {
      Serial.println("Unstable readings detected, ignoring this cycle.");
    }
  }
}

// Function to activate the pump
void activatePump() {
  digitalWrite(mosfetPin, HIGH);  // Activate MOSFET to start pump
  isRunning = true;

  pumpStartMillis = millis();  // Record the pump start time
  Serial.println("Pump activated.");
}

// Function to deactivate the pump
void deactivatePump() {
  digitalWrite(mosfetPin, LOW);  // Deactivate MOSFET to stop pump
  isRunning = false;

  unsigned long fillDuration = millis() - pumpStartMillis;  // Berechne die Laufzeit der Pumpe
  Serial.print("Pump deactivated. Runtime: ");
  Serial.print(fillDuration / 1000.0);  // Ausgabe der Laufzeit in Millisekunden
  Serial.println(" s");
}

// Function to monitor the pump's behavior during filling
void monitorPumpDuringFilling() {
  unsigned long currentMillis = millis();
  unsigned long maxPumpDuration = previousFillDuration * maxPumpDurationFactor;  // Calculate the max allowed pump duration

  // Always check if the pump has been running for too long (overflow prevention)
  if (!initialRun && currentMillis - pumpStartMillis >= maxPumpDuration) {
    Serial.println("Pump running too long, deactivating pump for safety.");
    deactivatePump();
    initialRun = false;  // After the first fill cycle, set initialRun to false
    return;              // Exit function if pump duration exceeded to prevent overflow
  }

  // Read initial sensor value
  int initialSensorValue = analogRead(sensorPinInput);

  // If the initial sensor value is below the threshold (no water detected), skip stability check but continue monitoring
  if (initialSensorValue < stableReadingThreshold) {
    Serial.println("Water not detected initially. Continuing to monitor pump duration.");
    return;  // Exit early as no water is detected
  }

  // If the initial value is high (indicating water is detected), start stability check
  unsigned long stableStart = millis();
  bool stateChanged = false;  // Flag to track if the state changes during stableDuration
  int sensorValue;

  // Continuously check sensor values over stableDuration
  while (millis() - stableStart <= stableDuration) {
    sensorValue = analogRead(sensorPinInput);

    // Determine if the state has changed from water detected (high) to no water (low)
    if (sensorValue < stableReadingThreshold) {
      stateChanged = true;  // State changed (from high to low), exit early
      break;
    }
  }

  // If no state changes were detected and water remains detected, stop the pump
  if (!stateChanged) {
    Serial.println("Water detected, deactivating pump.");
    deactivatePump();
    previousFillDuration = currentMillis - pumpStartMillis;  // Record fill duration
  }
}


// Function to handle button presses
void checkButtonPress() {
  // Read the current state of the button (active-low)
  bool currentButtonState = digitalRead(buttonPin);

  switch (buttonStateMachine) {
    case IDLE:
      if (currentButtonState == LOW) {  // Button pressed
        buttonStateMachine = DEBOUNCING;
        buttonPressStartTime = millis();  // Record when the button was pressed
      }
      break;

    case DEBOUNCING:
      if (millis() - buttonPressStartTime > debounceDelay) {  // Debounce delay passed
        if (currentButtonState == LOW) {                      // Button is still pressed
          buttonStateMachine = PRESSED;
          Serial.println("Button pressed, start timing.");
        } else {
          buttonStateMachine = IDLE;  // False trigger, go back to IDLE
        }
      }
      break;

    case PRESSED:
      if (currentButtonState == HIGH) {  // Button released
        unsigned long pressDuration = millis() - buttonPressStartTime;
        if (pressDuration < shortPressDuration) {  // Short press detected
          if (millis() - lastShortPressTime <= doublePressInterval) {
            buttonPressType = DOUBLE_SHORT_PRESS;  // Double short press detected
            buttonStateMachine = IDLE;             // Reset state after confirming double press
          } else {
            lastShortPressTime = millis();           // Record time of the first short press
            buttonPressType = NONE;                  // Do not process action yet, wait for second press
            buttonStateMachine = WAIT_DOUBLE_PRESS;  // Wait for a possible second press
          }
        } else if (pressDuration >= longPressDuration && pressDuration < pressAndHoldDuration) {  // Long press detected
          buttonPressType = LONG_PRESS;
          buttonStateMachine = IDLE;
        } else if (pressDuration >= pressAndHoldDuration) {  // Transition to press and hold
          buttonPressType = PRESS_AND_HOLD;
          buttonStateMachine = HOLDING;  // Transition to HOLDING state
        }
      } else if (millis() - buttonPressStartTime >= pressAndHoldDuration) {  // Button held for pressAndHoldDuration
        buttonPressType = PRESS_AND_HOLD;
        buttonStateMachine = HOLDING;  // Transition to HOLDING state
      }
      break;

    case HOLDING:
      if (currentButtonState == HIGH) {  // Button released
        buttonStateMachine = IDLE;       // Reset to IDLE on release
        buttonPressType = NONE;          // Clear press type
        deactivatePump();                // Turn off pump when button is released
        isPriming = false;               // Exit priming mode
      } else {
        buttonPressType = PRESS_AND_HOLD;                              // Keep PRESS_AND_HOLD active
        if (!isPriming) {                                              // Activate pump only once when entering priming mode
          Serial.println("Press and Hold action: Priming the pump.");  // Print message once
          activatePump();                                              // Turn on the pump while holding
          isPriming = true;                                            // Set priming mode flag
        } else {
          // Pump is already running, print running status
          Serial.println("Pump is running.");
        }
      }
      break;

    case WAIT_DOUBLE_PRESS:
      if (millis() - lastShortPressTime > doublePressInterval) {
        buttonPressType = SHORT_PRESS;  // No second press within interval, confirm short press
        buttonStateMachine = IDLE;      // Return to IDLE state and process short press action
      }
      if (currentButtonState == LOW) {    // Second press detected
        buttonStateMachine = DEBOUNCING;  // Go to debounce state for second press
        buttonPressStartTime = millis();  // Start timing the second press
      }
      break;
  }

  // Update the lastButtonState to the current state
  lastButtonState = currentButtonState;
}

// Function to handle detected button presses and execute actions based on press type
void handleButtonPress() {
  switch (buttonPressType) {
    case SHORT_PRESS:
      Serial.println("Short press action.");
      // Execute short press action here
      break;

    case DOUBLE_SHORT_PRESS:
      Serial.println("Double short press action.");
      // Execute double short press action here
      break;

    case LONG_PRESS:
      Serial.println("Long press action.");
      if (alarmRaised) {
        Serial.println("Alarm acknowledged and reset.");
        alarmRaised = false;  // Reset the alarm when long press is detected
      }
      // Execute other long press action here
      break;

    case PRESS_AND_HOLD:
      //Serial.println("Press and Hold action: Priming the pump.");
      // Pump activation logic is handled directly in the HOLDING state of the state machine.
      // No additional logic needed here.
      break;

    case NONE:
      // No press action
      break;
  }

  // Reset press type after handling
  buttonPressType = NONE;
}
