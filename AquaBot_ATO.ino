// noch wird die pumpe jede check periode angeschmissen, wenn die probe nicht im wasser ist. die pumpe wird dann automatisch gestoppe.
// wenn die pumpe automatisch gestoppt wurde, darf sie erst wieder starten, wenn eine bestätigung durch einen button erfolgte.
// die pump duration soll im serial monitor ausgegeben werden. 

// Pin assignments
const int sensorPinInput = A1;  // A1 for water sensor input
const int mosfetPin = 2;        // D2 for MOSFET control

// Time intervals and thresholds (in milliseconds)
const unsigned long readingInterval = 10000;  // Time interval for checking water level
const int stableReadingThreshold = 350;       // Analog reading threshold indicating water contact
const unsigned long stableDuration = 2000;    // Duration for stable sensor reading to confirm water level
const float maxPumpDurationFactor = 1.1;      // Safety factor: max pump duration increase for overflow prevention

// Variables for tracking time and readings
unsigned long previousCheckMillis = 0;   // Last time water level was checked
unsigned long pumpStartMillis = 0;       // Time when the pump was activated
unsigned long previousFillDuration = 0;  // Duration of the last filling
bool initialRun = true;                  // Flag to indicate first run since power-up

// Function prototypes
void checkWaterLevel();
void activatePump();
void deactivatePump();
void monitorPumpDuringFilling();

void setup() {
  // Set pin modes
  pinMode(sensorPinInput, INPUT);
  pinMode(mosfetPin, OUTPUT);

  // Initialize states
  digitalWrite(mosfetPin, LOW);  // Set MOSFET initially off
  Serial.begin(9600);            // Initialize serial for debugging
}

void loop() {
  // Check water level at regular intervals
  checkWaterLevel();

  // If the pump is active, monitor its duration to avoid overflow
  if (digitalRead(mosfetPin) == HIGH) {
    monitorPumpDuringFilling();
  }
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
        activatePump();              // Water is low, activate the pump
        pumpStartMillis = millis();  // Record the pump start time
      }
    } else {
      Serial.println("Unstable readings detected, ignoring this cycle.");
    }
  }
}

// Function to activate the pump
void activatePump() {
  digitalWrite(mosfetPin, HIGH);  // Activate MOSFET to start pump
  Serial.println("Pump activated.");
}

// Function to deactivate the pump
void deactivatePump() {
  digitalWrite(mosfetPin, LOW);  // Deactivate MOSFET to stop pump
  Serial.println("Pump deactivated.");
  initialRun = false;  // After the first fill cycle, set initialRun to false
}

// Function to monitor the pump's behavior during filling
void monitorPumpDuringFilling() {
  unsigned long currentMillis = millis();
  unsigned long maxPumpDuration = previousFillDuration * maxPumpDurationFactor;  // Calculate the max allowed pump duration

  // Always check if the pump has been running for too long (overflow prevention)
  if (!initialRun && currentMillis - pumpStartMillis >= maxPumpDuration) {
    Serial.println("Pump running too long, deactivating pump for safety.");
    deactivatePump();
    return;  // Exit function if pump duration exceeded to prevent overflow
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
