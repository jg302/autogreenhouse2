#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

long seconds(long ms) {
  return (ms * 1000);
}
//
//// Pin defs
#define thermistorPin A0
#define plusPin 2
#define minusPin 4
#define normalLed 13
#define errLed 11
#define testLed 12
#define modeSelektor 10
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Pin defs - relays
#define windowPin 9
#define pumpPin 8
//
// Other constants
#define R1 1000                                                // value of R1 on board
#define c1 0.001129148
#define c2 0.000234125
#define c3 0.0000000876741//steinhart-hart coeficients for thermistor
#define kelvin 273.15;

// Settings
#define clockRate 100   // ms
#define testRate seconds(2)  // how often sensor readings are taken
#define liveRate seconds(30)

#define windowTestRate seconds(10)   // how often sensor readings are taken, and how window delays for
#define windowLiveRate seconds(240) // 4mins

#define pumpWaitTest seconds(120) // how long we wait before running the pump
#define pumpWaitLive seconds(86400) // one day
#define pumpRunTestRate seconds(30) // how long the pump runs for
#define pumpRunLiveRate seconds(120)

// Variables
byte Vo;
float logR2, R2, T, vOut;
float averagedTemp;
long samplingLoop; // keeps track of internal 'sampling' loop (ms)
long windowLoop;   // keeps track of check for windowsopening and closing (ms)
long pumpLoop;     // keeps track of pumping check state (ms)
long pumpRun = 0;
int buttonCycle = 0; // keeps track if the button was pressed recently
byte digitalTemp;
byte digitalHumidity;

// States
bool windowOpen = false;      // Will need to init off a switch i guess
bool pumpRunning = false;
bool testMode;
byte error = 0;
byte targetTemp = 23;

// Test or live rates - sampler
long getInterval() {
  if (testMode == true) {
    return testRate;
  }
  return liveRate;
}


// Test or live rates - window
long getWindowTime() {
  if (testMode == true) {
    return windowTestRate;
  }
  return windowLiveRate;
}

// Test or live rates - pump only
// wait = wait time otherwise run time
long getPumpRate(bool wait)
{
  if (testMode == true)
  {
    return wait ? pumpWaitTest : pumpRunTestRate;
  }
  return wait ? pumpWaitLive : pumpRunLiveRate;
}

void closeWindow() {
  digitalWrite(windowPin, LOW);
  windowOpen = false;
}

void openWindow() {
  digitalWrite(windowPin, HIGH);
  windowOpen = true;
}

void runPump() {
  digitalWrite(pumpPin, HIGH);
  pumpRunning = true;
}

void stopPump() {
  digitalWrite(pumpPin, LOW);
  pumpRunning = false;
}


float kelvinify(float value) {
  return value + kelvin;
}

float deKelvinify(float value) {
  return value - kelvin;
}

// Error 1 - Digital temp NA
// Error 2 - Analogue temp not correct
// Error 3 - Digital humidity not correct
void handleError(int errorCode) {
  digitalWrite(errLed, HIGH);
  error = errorCode;
}

void handleButtonChange() {
  buttonCycle = 4;
}

void setup() {
  Serial.begin(115200);

  // Pin setup
  pinMode(plusPin, INPUT);
  pinMode(minusPin, INPUT);
  pinMode(modeSelektor, INPUT);

  pinMode(pumpPin, OUTPUT);
  pinMode(windowPin, OUTPUT);
  pinMode(testLed, OUTPUT);
  pinMode(errLed, OUTPUT);

  dht.begin();

  display.begin(false, 0x3C); // Address 0x3C default
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  // Test sequence if in test mode
//  if (digitalRead(modeSelektor) == LOW) {
//    testMode = true;
//    openWindow();
//    delay(seconds(10));
//    closeWindow();
//    delay(1000);
//    runPump();
//    delay(seconds(4));
//    stopPump();
//    delay(1000);
//    digitalWrite(normalLed, HIGH);
//    delay(1000);
//    digitalWrite(normalLed, LOW);
//    digitalWrite(testLed, HIGH);
//    delay(1000);
//    digitalWrite(testLed, LOW);
//    digitalWrite(errLed, HIGH);
//    delay(1000);
//    digitalWrite(errLed, LOW);
//  }
}

void loop() {
  if (digitalRead(modeSelektor) == LOW && !testMode) {
    testMode = true;
    digitalWrite(testLed, HIGH);
    digitalWrite(normalLed, LOW);
  }

  if (testMode && digitalRead(modeSelektor) == HIGH)
  {
    testMode = false;
    digitalWrite(testLed, LOW);
  }

  digitalWrite(normalLed, HIGH);

  // Are we running a sample in this loop?
  if (samplingLoop > getInterval()) {
    digitalWrite(testLed, LOW);
    digitalWrite(normalLed, LOW);
    samplingLoop = 0;

    //  Get humidity event
    digitalHumidity = dht.readHumidity();

    // Therminsor calculations
    Vo = analogRead(thermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0); //calculate resistance on thermistor
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)); // temperature in Kelvin
    T = T - 273.15;                                             //convert Kelvin to Celcius
    digitalTemp = dht.readTemperature();

    // Take a gradual average
    averagedTemp = (averagedTemp + ((T + digitalTemp) * 0.5)) * 0.5;

    if (isnan(digitalTemp))
    {
      averagedTemp = T;
      handleError(1);
    }

    if (Vo == 0 || T > 60)
    {
      averagedTemp = digitalTemp;
      handleError(2);
    }

    if (digitalHumidity == 0)
    {
      handleError(3);
    }

  }
  // End temperature reading

  // Are we running a window check this loop?
  if (windowLoop >= getWindowTime())
  {
    // It's too hot, open if not already open
    if (averagedTemp >= targetTemp) {
      if (!windowOpen)
      {
        openWindow();
      }
    }
    // Too cold, close window if open
    if (averagedTemp < targetTemp) {
      if (windowOpen) {
        closeWindow();
      }
    }
    windowLoop = 0;
  };

  //   Is it time to run the pump?
  if (pumpLoop >= getPumpRate(true))
  {
    runPump();
    pumpLoop = 0;

  };

  // Stuff that needs checking every loop e.g. button, switch status, pump state

  // Temp button control
  // Debounce type effect
  if (buttonCycle == 0) {
    if (digitalRead(plusPin) == HIGH) {
      targetTemp = targetTemp + 1;
      handleButtonChange();
    }

    if (digitalRead(minusPin) == HIGH) {
      targetTemp = targetTemp - 1;
      handleButtonChange();
    }
  } else {
    buttonCycle = buttonCycle - 1;
  }

  // Pump run time checker
  if (pumpRunning) {
    // Add to the pump running time counter
    pumpRun = (pumpRun + clockRate);

    // Are we over the counter?
    if (pumpRun >= getPumpRate(false)) {
      pumpRun = 0;
      stopPump();
    }
  }

  // Do display stuff
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Set: ");
  display.println(targetTemp);
  display.setCursor(0, 16);
  display.print("Now: ");
  display.println(digitalTemp);
  display.setCursor(60, 32);
  display.print(digitalHumidity);
  display.println("%");
  display.display();

  // Increment each check counter
  samplingLoop = (samplingLoop + clockRate);
  windowLoop = (windowLoop + clockRate);
  pumpLoop = (pumpLoop + clockRate);
  delay(clockRate);
}
