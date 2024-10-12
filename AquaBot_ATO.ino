// Implement error handlings
// Reduce Code Duplication
// Add function documentation to each function header: explaining its purpose, parameters and return values
// Integration of the RGB sensor LED via LightWS2812 library

// Include libraries for the OLED display management
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Include fonts for the OLED display
#include <Fonts/Org_01.h>
#include <Fonts/TomThumb.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Debug Mode
// When debugging is disabled, the code that generates debug output is not included in the compiled program.
#define DEBUG_MODE  // Uncomment this line to enable debug mode

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Pin assignments
const int sensorPinInput = A1;  // A1 for water sensor input
const int mosfetPin = 2;        // D2 for MOSFET control
const int buttonPin = 3;        // D3 for button input

// Time intervals and thresholds (in milliseconds)
const unsigned long readingInterval = 10000;  // Time interval for checking water level
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

// Enumeration to define pump states
enum class PumpState {
  OFF,      // Pump is off
  RUNNING,  // Pump is running (regular water level monitoring)
  PRIMING   // Pump is priming (manual activation via press-and-hold)
};
PumpState pumpState = PumpState::OFF;  // Initial state of the pump

// Enumeration to define button states
enum class ButtonState {
  IDLE,
  DEBOUNCING,
  PRESSED,
  WAIT_DOUBLE_PRESS,
  HOLDING
};
ButtonState buttonStateMachine = ButtonState::IDLE;

// Button state variables
bool buttonState = HIGH;      // Current button state
bool lastButtonState = HIGH;  // Previous button state
enum class ButtonPressType {  // Enum to define button press types
  NONE,
  SHORT_PRESS,
  DOUBLE_SHORT_PRESS,
  LONG_PRESS,
  PRESS_AND_HOLD
};
ButtonPressType buttonPressType = ButtonPressType::NONE;

// Enumeration to define OLED display states
enum class DisplayState {
  MAIN_VIEW,            // Showing the main system information
  MENU_SELECTION,       // Menu where user selects "Reservoir Full" or "Set Volume"
  MENU_RESERVOIR_FULL,  // Option to reset reservoir
  MENU_SET_VOLUME,      // Option to set net reservoir volume
  SETTING_VOLUME        // Adjusting the net volume of the reservoir
};
DisplayState displayState = DisplayState::MAIN_VIEW;  // Initial display state

// Function prototypes
void checkWaterLevel();
void activatePump();
void deactivatePump();
void monitorPumpDuringFilling();
void checkButtonPress();
void handleButtonPress();
void processButtonPress(ButtonPressType pressType);
bool isWaterLevelStable(int initialReading);

void setup() {

// Initialize serial communication (for debugging)
  Serial.begin(9600);

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  delay(100);  // Add this before display.begin()
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Set pin modes
  pinMode(sensorPinInput, INPUT);
  pinMode(mosfetPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Button with internal pull-up resistor

  
  // Initialize states
  digitalWrite(mosfetPin, LOW);  // Set MOSFET initially off




  // Initialize the OLED display
  //setupOLED();
}

void loop() {

  //displayMainView();

  // Override all actions if the pump is in PRIMING state
  if (pumpState != PumpState::PRIMING) {
    if (!alarmRaised && pumpState == PumpState::OFF) {
      // If the pump is off, check water level at regular intervals
      checkWaterLevel();
    } else if (pumpState == PumpState::RUNNING) {
      // If the pump is running, monitor its duration to avoid overflow
      monitorPumpDuringFilling();
    }
  }

  // Continuously check for button presses
  checkButtonPress();

  // Process button press types based on detected press type
  handleButtonPress();
}


/* ---------------------------------------------------------------
 ------------- Main MONITORING and CONTROL functions -------------
 ----------------------------------------------------------------*/

// Helper function to check if water level is stable over a specified duration
// Parameters:
// - initialReading: The initial analog reading of the water sensor
// Returns: true if the water level remains stable for the stableDuration, false otherwise
bool isWaterLevelStable(int initialReading) {
  unsigned long stableStart = millis();
  // Continuously check sensor values over stableDuration
  while (millis() - stableStart <= stableDuration) {
    int sensorValue = analogRead(sensorPinInput);  // Read sensor value

    // Determine if the state has changed from initial reading (indicates instability)
    if ((sensorValue >= stableReadingThreshold && initialReading < stableReadingThreshold) || (sensorValue < stableReadingThreshold && initialReading >= stableReadingThreshold)) {
      return false;  // State changed, not stable
    }
  }
  return true;  // State remained stable throughout the duration
}

// Function to check the water level at set intervals
// Reads the water sensor value and activates the pump if water level is low
void checkWaterLevel() {
  unsigned long currentMillis = millis();

  // Check if the reading interval has passed
  if (currentMillis - previousCheckMillis >= readingInterval) {
    previousCheckMillis = currentMillis;

    // Read initial sensor value
    int sensorValue = analogRead(sensorPinInput);

    // Check if water level is stable before making decisions
    if (isWaterLevelStable(sensorValue)) {
      if (sensorValue >= stableReadingThreshold) {
        DEBUG_PRINTLN("Water level is stable and high.");
        // Water is detected, no action needed
      } else {
        DEBUG_PRINTLN("Water level is low, activating pump.");
        activatePump();  // Water is low, activate the pump
      }
    } else {
      DEBUG_PRINTLN("Unstable readings detected, ignoring this cycle.");
    }
  }
}

// Function to activate the pump
void activatePump() {
  digitalWrite(mosfetPin, HIGH);  // Activate MOSFET to start pump
  pumpStartMillis = millis();     // Record the pump start time

  // Set pump state based on the current scenario
  if (pumpState == PumpState::PRIMING) {
    DEBUG_PRINTLN("Pump priming mode activated.");
  } else {
    DEBUG_PRINTLN("Pump activated.");
    pumpState = PumpState::RUNNING;  // Set pump state to running
  }
}


// Function to deactivate the pump
void deactivatePump() {
  digitalWrite(mosfetPin, LOW);                             // Deactivate MOSFET to stop pump
  unsigned long fillDuration = millis() - pumpStartMillis;  // Calculate runtime duration

  DEBUG_PRINT("Pump deactivated. Runtime: ");
  DEBUG_PRINT(fillDuration / 1000.0);
  DEBUG_PRINTLN(" s");

  // Reset pump state to OFF
  pumpState = PumpState::OFF;
}

// Function to monitor the pump's behavior during filling
// Checks if the pump has been running for too long or if water level is stable to deactivate
void monitorPumpDuringFilling() {
  unsigned long currentMillis = millis();
  unsigned long maxPumpDuration = previousFillDuration * maxPumpDurationFactor;  // Calculate the max allowed pump duration

  // Check if the pump has been running for too long (overflow prevention)
  if (!initialRun && currentMillis - pumpStartMillis >= maxPumpDuration) {
    DEBUG_PRINTLN("Pump running too long, deactivating pump for safety.");
    deactivatePump();
    alarmRaised = true;  // Raise an alarm that must be acknowledged before any other action is taken
    DEBUG_PRINTLN("ALARM raised. Waiting for acknowledgement.");
    return;  // Exit function if pump duration exceeded to prevent overflow
  }

  // Read initial sensor value
  int initialSensorValue = analogRead(sensorPinInput);

  // If the initial sensor value is below the threshold (no water detected), skip stability check but continue monitoring
  if (initialSensorValue < stableReadingThreshold) {
    DEBUG_PRINTLN("Water not detected initially. Continuing to monitor pump duration.");
    return;  // Exit early as no water is detected
  }

  // If the initial value is high (indicating water is detected), start stability check
  if (isWaterLevelStable(initialSensorValue)) {
    DEBUG_PRINTLN("Water detected, deactivating pump.");
    deactivatePump();
    initialRun = false;
    previousFillDuration = currentMillis - pumpStartMillis;  // Record fill duration
  }
}


/* ---------------------------------------------------------------
 --------------------- BUTTON HANDLING ---------------------------
 ----------------------------------------------------------------*/

// Function to handle button presses
// Check and process button state transitions, debouncing, and press types
void checkButtonPress() {
  bool currentButtonState = digitalRead(buttonPin);  // Read the current state of the button (active-low)

  switch (buttonStateMachine) {
    case ButtonState::IDLE:
      if (currentButtonState == LOW) {  // Button pressed
        buttonStateMachine = ButtonState::DEBOUNCING;
        buttonPressStartTime = millis();  // Record when the button was pressed
      }
      break;

    case ButtonState::DEBOUNCING:
      if (millis() - buttonPressStartTime > debounceDelay) {  // Debounce delay passed
        if (currentButtonState == LOW) {                      // Button is still pressed
          buttonStateMachine = ButtonState::PRESSED;
          DEBUG_PRINTLN("Button pressed, start timing.");  // Print button pressed message
        } else {
          buttonStateMachine = ButtonState::IDLE;  // False trigger, go back to IDLE
        }
      }
      break;

    case ButtonState::PRESSED:
      if (currentButtonState == HIGH) {  // Button released
        unsigned long pressDuration = millis() - buttonPressStartTime;
        if (pressDuration < shortPressDuration) {  // Short press detected
          if (millis() - lastShortPressTime <= doublePressInterval) {
            buttonPressType = ButtonPressType::DOUBLE_SHORT_PRESS;  // Double short press detected
            buttonStateMachine = ButtonState::IDLE;                 // Reset state after confirming double press
          } else {
            lastShortPressTime = millis();                        // Record time of the first short press
            buttonPressType = ButtonPressType::NONE;              // Do not process action yet, wait for second press
            buttonStateMachine = ButtonState::WAIT_DOUBLE_PRESS;  // Wait for a possible second press
          }
        } else if (pressDuration >= longPressDuration && pressDuration < pressAndHoldDuration) {  // Long press detected
          buttonPressType = ButtonPressType::LONG_PRESS;
          buttonStateMachine = ButtonState::IDLE;
        } else if (pressDuration >= pressAndHoldDuration) {  // Transition to press and hold
          buttonPressType = ButtonPressType::PRESS_AND_HOLD;
          buttonStateMachine = ButtonState::HOLDING;  // Transition to HOLDING state
        }
      } else if (millis() - buttonPressStartTime >= pressAndHoldDuration) {  // Button held for pressAndHoldDuration
        buttonPressType = ButtonPressType::PRESS_AND_HOLD;
        buttonStateMachine = ButtonState::HOLDING;  // Transition to HOLDING state
      }
      break;

    case ButtonState::HOLDING:
      if (currentButtonState == HIGH) {           // Button released
        buttonStateMachine = ButtonState::IDLE;   // Reset to IDLE on release
        buttonPressType = ButtonPressType::NONE;  // Clear press type
        deactivatePump();                         // Turn off pump when button is released
      } else {
        buttonPressType = ButtonPressType::PRESS_AND_HOLD;            // Keep PRESS_AND_HOLD active
        if (pumpState != PumpState::PRIMING) {                        // Enter priming mode only once
          DEBUG_PRINTLN("Press and Hold action: Priming the pump.");  // Print message once
          activatePump();                                             // Turn on the pump while holding
          pumpState = PumpState::PRIMING;                             // Set pump state to priming
        } else {
          // Pump is already running, print running status
          DEBUG_PRINTLN("Pump is running.");
        }
      }
      break;

    case ButtonState::WAIT_DOUBLE_PRESS:
      if (millis() - lastShortPressTime > doublePressInterval) {
        buttonPressType = ButtonPressType::SHORT_PRESS;  // No second press within interval, confirm short press
        buttonStateMachine = ButtonState::IDLE;          // Return to IDLE state and process short press action
      }
      if (currentButtonState == LOW) {                 // Second press detected
        buttonStateMachine = ButtonState::DEBOUNCING;  // Go to debounce state for second press
        buttonPressStartTime = millis();               // Start timing the second press
      }
      break;
  }

  // Update the lastButtonState to the current state
  lastButtonState = currentButtonState;
}

// Function to handle detected button presses and execute actions based on press type
void handleButtonPress() {
  switch (buttonPressType) {
    case ButtonPressType::SHORT_PRESS:
      DEBUG_PRINTLN("Short press action.");
      // Execute short press action here
      break;

    case ButtonPressType::DOUBLE_SHORT_PRESS:
      DEBUG_PRINTLN("Double short press action.");
      // Execute double short press action here
      break;

    case ButtonPressType::LONG_PRESS:
      DEBUG_PRINTLN("Long press action.");
      if (alarmRaised) {
        DEBUG_PRINTLN("Alarm acknowledged and reset.");
        alarmRaised = false;  // Reset the alarm when long press is detected
      }
      // Execute other long press action here
      break;

    case ButtonPressType::PRESS_AND_HOLD:
      // Pump activation logic is handled directly in the HOLDING state of the state machine.
      // No additional logic needed here.
      break;

    case ButtonPressType::NONE:
      // No press action
      break;
  }

  // Reset press type after handling
  buttonPressType = ButtonPressType::NONE;
}


/* ---------------------------------------------------------------
 ------------------- OLED DISPLAY interaction --------------------
 ----------------------------------------------------------------*/

// Function to initialize the OLED display
void setupOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {  // Address 0x3D (61) for 128x64
    DEBUG_PRINTLN(F("SSD1306 allocation failed"));
	//for(;;); // Don't proceed, loop forever
  }

  display.display();
  delay(2000);  // Pause for 2 seconds
  display.clearDisplay();
}


// Function to display the main system information
void displayMainView() {
  display.clearDisplay();

  display.setFont(&Org_01);
  display.setTextSize(2);

  // Show system states
  display.setCursor(5, 17);

  // Check if an alarm has been raised
  if (alarmRaised) {
    display.println("!! ERROR !!");
  } else {
    // Show pump state if no alarm is raised
    switch (pumpState) {
      case PumpState::RUNNING:
        display.println("Filling up");
        break;
      case PumpState::PRIMING:
        display.println("Priming");
        break;
      case PumpState::OFF:
        display.println("In level");
        break;
    }
  }

  display.setTextSize(1);

  // Show the evaporation rate in liter / week
  display.setCursor(5, 34);
  display.println("Evaporation: 10 L/w");

  // Show how many water remains in the reservoir
  display.setCursor(5, 45);
  display.println("Remaining: 10/20 L");

  // Show how many days the reservoir still lasts until a refill is required
  // LW (Low Warning)
  // LA (Low Alarm -> no pump start possible)
  display.setCursor(5, 56);
  display.println("Empty in: 3 d (LW)");

  // Show bar graph of the reservoir filling level and percentage of filling
  display.setFont(&TomThumb);
  display.setCursor(107, 10);
  display.println("100 %");
  display.drawRect(107, 13, 16, 47, 1);
  display.fillRect(109, 35, 12, 23, 1);

  // Update the OLED display
  display.display();
}
