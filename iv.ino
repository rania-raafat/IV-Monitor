#include <LiquidCrystal.h>
#include <PulseSensorPlayground.h>
#include <string.h>
// =====================================
// PIN DEFINITIONS
// =====================================

#define SENSOR_PIN A0
#define BUZZER_PIN 6
#define PULSE_SENSOR_PIN A3
#define PULSE_THRESHOLD 550

// =====================================
// LCD
// RS, E, D4, D5, D6, D7
// =====================================

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// =====================================
// PULSE SENSOR
// =====================================

PulseSensorPlayground pulseSensor;

// =====================================
// SETTINGS
// =====================================

// Drop detection threshold
const int DROP_DELTA = 60;

// Release threshold (hysteresis)
const int RELEASE_DELTA = 35;

// Minimum time between drops
const unsigned long COOLDOWN_MS = 250;

// Estimated volume per drop
const float DROP_VOLUME_ML = 0.05;

// Moving average sample count
const int NUM_SAMPLES = 16;

// Baseline adaptation speed
const float BASELINE_ALPHA = 0.002;

// Required consecutive detections
const int REQUIRED_HITS = 3;

// No drops timeout (1 minute)
const unsigned long NO_DROP_TIMEOUT = 60000UL;

// Rapid flow threshold
const float HIGH_FLOW_THRESHOLD = 8.0;

// =====================================
// BUZZER SETTINGS
// =====================================

// Buzzer tone frequency
const int BUZZER_FREQUENCY = 2000;

// Buzzer ON duration
const unsigned long BUZZER_DURATION = 1000;

// =====================================
// VARIABLES
// =====================================

int samples[NUM_SAMPLES];
int sampleIndex = 0;

float baseline = 0;

bool dropActive = false;

unsigned long drops = 0;

unsigned long lastDropTime = 0;
unsigned long startTime = 0;

int consecutiveHits = 0;

const char* statusMessage = "NORMAL FLOW";

int bpm=0; 

// =====================================
// BUZZER VARIABLES
// =====================================

bool buzzerActive = false;
unsigned long buzzerStart = 0;

// =====================================
// START BUZZER
// =====================================

void triggerBuzzer()
{
    if (!buzzerActive)
    {
        tone(BUZZER_PIN, BUZZER_FREQUENCY);

        buzzerActive = true;

        buzzerStart = millis();
    }
}

// =====================================
// UPDATE BUZZER
// =====================================

void updateBuzzer()
{
    if (buzzerActive &&
        millis() - buzzerStart >= BUZZER_DURATION)
    {
        noTone(BUZZER_PIN);

        buzzerActive = false;
    }
}

// =====================================
// READ SMOOTHED SENSOR
// =====================================

int readSmoothed()
{
    // Flush ADC after channel switching
    analogRead(SENSOR_PIN);
    analogRead(SENSOR_PIN);

    delayMicroseconds(100);

    int stableReading = analogRead(SENSOR_PIN);

    samples[sampleIndex] = stableReading;

    sampleIndex++;

    if (sampleIndex >= NUM_SAMPLES)
    {
        sampleIndex = 0;
    }

    long total = 0;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        total += samples[i];
    }

    return total / NUM_SAMPLES;
}
// =====================================
// SETUP
// =====================================

void setup()
{
    Serial.begin(115200);

    // LCD setup
    lcd.begin(16, 2);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Drip Monitor");

    // Buzzer setup
    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);

    // Pulse sensor setup
    pulseSensor.analogInput(PULSE_SENSOR_PIN);
    pulseSensor.setThreshold(PULSE_THRESHOLD);

    if (pulseSensor.begin())
    {
        Serial.println("Pulse Sensor Ready");
    }

    // Allow sensor stabilization
    delay(1000);

    // Initialize baseline
    int initial = analogRead(SENSOR_PIN);

    baseline = initial;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        samples[i] = initial;
    }

    startTime = millis();

    Serial.println("Drip Counter Started");
}

// =====================================
// MAIN LOOP
// =====================================

void loop()
{
    unsigned long now = millis();

    // =====================================
    // UPDATE BUZZER
    // =====================================

    updateBuzzer();

    // =====================================
    // READ SENSOR
    // =====================================

static unsigned long lastDripRead = 0;
static int raw = 0;

if (millis() - lastDripRead >= 15)
{
    lastDripRead = millis();

    raw = readSmoothed();
}

    // =====================================
    // UPDATE BASELINE
    // =====================================

    if (!dropActive)
    {
        baseline =
            baseline * (1.0 - BASELINE_ALPHA) +
            raw * BASELINE_ALPHA;
    }

    // =====================================
    // SIGNAL DELTA
    // =====================================

    float delta = baseline - raw;

    // =====================================
    // PERSISTENCE FILTER
    // =====================================

    if (delta > DROP_DELTA)
    {
        consecutiveHits++;
    }
    else
    {
        consecutiveHits = 0;
    }

    // =====================================
    // DROP DETECTION
    // =====================================

    if (!dropActive &&
        consecutiveHits >= REQUIRED_HITS &&
        (now - lastDropTime > COOLDOWN_MS))
    {
        dropActive = true;

        drops++;

        lastDropTime = now;

        consecutiveHits = 0;

        Serial.println("DROP DETECTED");
    }

    // =====================================
    // DROP END DETECTION
    // =====================================

    if (dropActive &&
        delta < RELEASE_DELTA)
    {
        dropActive = false;
    }

    // =====================================
    // FLOW CALCULATION
    // =====================================

    float elapsedMinutes =
        (now - startTime) / 60000.0;

    if (elapsedMinutes < 0.001)
    {
        elapsedMinutes = 0.001;
    }

    float totalVolume =
        drops * DROP_VOLUME_ML;

    float flowRate =
        totalVolume / elapsedMinutes;

    // =====================================
    // FLOW ALERT LOGIC
    // =====================================

    statusMessage = "NORMAL FLOW";

    // No drops detected for timeout duration
    if (drops > 0 &&
        (now - lastDropTime) > NO_DROP_TIMEOUT)
    {
        statusMessage = "INCREASE FLOW";

        triggerBuzzer();
    }

    // Flow too high
    else if (flowRate > HIGH_FLOW_THRESHOLD)
    {
        statusMessage = "DECREASE FLOW";

        triggerBuzzer();
    }

    // =====================================
    // HEARTBEAT DISPLAY
    // =====================================
   
    if (pulseSensor.sawStartOfBeat())
    {
        bpm = pulseSensor.getBeatsPerMinute();

    }

    // =====================================
    // LCD UPDATE
    // =====================================

    static unsigned long lastLCD = 0;

    if (now - lastLCD > 300)
    {
        lastLCD = now;

        lcd.clear();

        // Line 1
        lcd.setCursor(0, 0);

        lcd.print("Flow:");
        lcd.print(flowRate, 1);
        lcd.print("mL/m");
        // Line 2
        lcd.setCursor(0, 1);

lcd.print("BPM:");
lcd.print(bpm);

lcd.print(" ");

if (strcmp(statusMessage, "NORMAL FLOW") == 0)
{
    lcd.print("OK ");
}
else
{
    lcd.print("ALERT");
}
lcd.print("     ");
    }

    // =====================================
    // SERIAL DEBUG OUTPUT
    // =====================================

    static unsigned long lastPrint = 0;

    if (now - lastPrint > 200)
    {
        lastPrint = now;

        Serial.print("Raw: ");
        Serial.print(raw);

        Serial.print("  Base: ");
        Serial.print((int)baseline);

        Serial.print("  Delta: ");
        Serial.print(delta);

        Serial.print("  Drops: ");
        Serial.print(drops);

        Serial.print("  Flow(mL/min): ");
        Serial.println(flowRate, 2);
    }


}