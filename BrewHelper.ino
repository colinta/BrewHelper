// This #include statement was automatically added by the Particle IDE.
SYSTEM_THREAD(ENABLED);

#include "Adafruit_GFX/Adafruit_GFX.h"
#include "ledmatrix-max7219-max7221/ledmatrix-max7219-max7221.h"
#include "Adafruit_PCD8544/Adafruit_PCD8544.h"

const double VCC = 3.3;
const double ADC_RES = 4096.0;
const int POWER_PIN = D0;
const int TEMP_PIN = A0;
const int NEXT_PIN = D7;
const int TIMER_PIN = D1;


unsigned short numTemps = 5;
unsigned short currentTempI = 0;
double currentTemp;
double temps[] = {
    0, 0, 0, 0, 0
};

unsigned short currentStep = 0;
unsigned short numSteps = 5;

String titles[] = {
    "Begin",
    "Steep",
    "Boil hops",
    "Fin. hops",
    "Cool down",
};

double targetTemps[] = {
    70.5,
    155,
    200,
    200,
    67.5,
};
double deltas[] = {
    10,
    5,
    5,
    5,
    2.5,
};
unsigned short minutes[] = {
    0,
    60,
    30,
    15,
    0,
};
bool timerStarted = false;
unsigned long timerStartTime;
unsigned int timerRemTime;

bool ignoreButton = false;
bool nextButtonDown = false;
bool timerButtonDown = false;

Adafruit_PCD8544 nokiaLCD = Adafruit_PCD8544(SS, D2, D3);
LEDMatrix *led;

void onNextDown() {
    nextButtonDown = true;
}

void onTimerDown() {
    timerButtonDown = true;
}


void setup() {
    Particle.function("setTitle1", setTitle1);

    led = new LEDMatrix(1, 1, D6, D5, D4);
    led->addMatrix(0, 0, 90, false, false);
    led->setIntensity(0, 1);

    pinMode(POWER_PIN, OUTPUT);
    pinMode(TEMP_PIN, INPUT);
    pinMode(NEXT_PIN, INPUT_PULLDOWN);
    pinMode(TIMER_PIN, INPUT_PULLDOWN);
    digitalWrite(POWER_PIN, HIGH);
    nokiaLCD.begin(); // This will setup our pins, and initialize the LCD
    nokiaLCD.setContrast(55); // Pretty good value, play around with it
    nokiaLCD.setTextSize(1);
    nokiaLCD.setTextColor(BLACK);

    attachInterrupt(NEXT_PIN, onNextDown, RISING);
    attachInterrupt(TIMER_PIN, onTimerDown, RISING);
    for (int i = 0; i < numTemps; i++ ) {
        temps[i] = analogToTemp(measure());
    }

    nokiaLCD.clearDisplay();
}

void loop() {
    if (nextButtonDown || timerButtonDown) {
        if ( ignoreButton ) {
            bool buttonDown = digitalRead(NEXT_PIN) || digitalRead(TIMER_PIN);
            if ( !buttonDown ) {
                timerButtonDown = false;
                nextButtonDown = false;
                ignoreButton = false;
                delay(50);
            }
        }
        else {
            ignoreButton = true;

            if ( nextButtonDown ) {
                currentStep += 1;
                timerStarted = false;
            }
            if ( timerButtonDown ) {
                timerStartTime = millis();
                timerStarted = true;
            }
        }
    }

    updateDisplay();

    delay(250);
}

int setTitle1(String title) {
    titles[1] = title;
    if (currentStep == 1) {
        updateDisplay();
    }
    return 0;
}

void updateDisplay() {
    currentStep = currentStep % numSteps;
    String title = titles[currentStep];
    double target = targetTemps[currentStep];
    double delta = deltas[currentStep];
    unsigned long timerTime = minutes[currentStep] * 60 * 1000;

    unsigned short timeInSec, timeInMin;
    if ( timerStarted ) {
        timerRemTime = (timerTime - (millis() - timerStartTime)) / 1000;
        if ( timerRemTime < 0 || timerRemTime > 4000000 ) {
            timerRemTime = 0;
        }
    }
    else {
        timerRemTime = timerTime / 1000;
    }
    timeInMin = timerRemTime / 60;
    timeInSec = timerRemTime - timeInMin * 60;

    // put your main code here, to run repeatedly:
    int currentV = measure();
    temps[currentTempI] = analogToTemp(currentV);
    currentTempI = (currentTempI + 1) % numTemps;
    currentTemp = 0;
    for (int i = 0; i < numTemps; i++ ) {
        currentTemp += temps[i];
    }
    currentTemp /= numTemps;
    char deg = 247;

    nokiaLCD.clearDisplay();
    nokiaLCD.print("  ");
    nokiaLCD.println(title);
    nokiaLCD.print("Temp: ");
    nokiaLCD.print(currentTemp, 1);
    nokiaLCD.println(deg);
    nokiaLCD.print("Trgt: ");
    nokiaLCD.println(target, 1);
    nokiaLCD.print("Time: ");
    nokiaLCD.print(timeInMin);
    nokiaLCD.print(":");
    if (timeInSec < 10) {
        nokiaLCD.print("0");
    }
    nokiaLCD.println(timeInSec);
    nokiaLCD.println("");

    if (currentStep < numSteps - 1) {
        if ( !timerStarted && timerTime > 0 ) {
            nokiaLCD.print("< Start");
        }
        else {
            nokiaLCD.print("       ");
        }
        nokiaLCD.print(" Next >");
    }
    else {
        nokiaLCD.print("     Done!");
    }
    nokiaLCD.display();

    led->fillScreen(false);

    int val;
    if ( currentTemp > target + 3 * delta ) {  // 3*delta - 5*delta
        led->drawFastVLine(7, 1, 7, true);
        led->drawFastHLine(0, 1, 8, true);
        val = round(interpolate(currentTemp, target + 3 * delta, target + 5 * delta, 1, 7));
        led->drawFastVLine(0, 1, val, true);
    }
    else if ( currentTemp > target + delta ) {  // delta - 3*delta
        led->drawFastVLine(7, 1, 7, true);
        val = round(interpolate(currentTemp, target + delta, target + 3 * delta, 7, 0));
        led->drawFastHLine(val, 1, 8 - val, true);
    }
    else if ( currentTemp > target + delta / 2 ) {  // delta/2 - delta
        val = round(interpolate(currentTemp, target + delta / 2, target + delta, 0, 7));
        led->drawLine(7, 7, val, 1, true);
    }
    else if ( currentTemp > target ) {  // 0 - delta/2
        val = round(interpolate(currentTemp, target + delta / 2, target, 1, 7));
        led->drawLine(7, 7, 0, val, true);
    }
    else if ( currentTemp < target - 3 * delta ) {  // -5*delta - -3*delta
        led->drawFastVLine(0, 1, 7, true);
        led->drawFastHLine(0, 7, 8, true);
        val = round(interpolate(currentTemp, target - 5 * delta, target - 3 * delta, 1, 7));
        val = max(val, 1);
        led->drawFastVLine(7, val, 8 - val, true);
    }
    else if ( currentTemp < target - delta ) {  // -3*delta - -delta
        led->drawFastVLine(0, 1, 7, true);
        val = round(interpolate(currentTemp, target - delta, target - 3 * delta, 0, 7));
        led->drawFastHLine(0, 7, val, true);
    }
    else if (currentTemp < target - delta / 2) {  // -delta - -delta/2
        val = round(interpolate(currentTemp, target - delta, target - delta / 2, 0, 7));
        led->drawLine(0, 7, val, 1, true);
    }
    else {  // -delta/2 - 0
        val = round(interpolate(currentTemp, target - delta / 2, target, 1, 7));
        led->drawLine(0, 7, 7, val, true);
    }

    if ( minutes[currentStep] > 0 ) {
        if ( timerRemTime == 0 ) {
            if ( (int)(millis() / 1000) % 2) {
                led->drawLine(0, 0, 7, 0, true);
            }
        }
        else {
            val = round(interpolate(timerRemTime - 1, minutes[currentStep] * 60, 0.0f, 7.0f, 0.0f));
            if ( val > 0 ) {
                led->drawLine(0, 0, val - 1, 0, true);
            }
            if ( !timerStarted || (int)(millis() / 1000) % 2) {
                led->drawPixel(val, 0, true);
            }
        }
    }

    led->flush();
}

double interpolate(double x, double a1, double a2, double b1, double b2) {
    return (b2 - b1) / (a2 - a1) * (x - a1) + b1;
}

int measure() {
  digitalWrite(POWER_PIN, HIGH);
  delay(2);

  int v_out = analogRead(TEMP_PIN);
  digitalWrite(POWER_PIN, LOW);
  return v_out;
}

double analogToTemp(int adcInput) {
    double voltage = adcInput * VCC / ADC_RES * 1000.0;
    return toFarenheit(0.0512 * voltage - 20.5128);
}

double toFarenheit(double c) {
    return c * 9.0f / 5.0f + 32.0f;
}
