#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

int seconds(int ms) {
  return (ms * 1000);
}

// Pin defs - ins
const int thermistorPin = A0;
const int plusPin = 2;
const int minusPin = 4;
const int normalLed = 13;
const int errLed = 11;
const int testLed = 12;
const int modeSelektor = 10;
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Pin defs - outs
const int windowPin = 9;
const int pumpPin = 8;

// Other constants
const int R1 = 1000;                                                // value of R1 on board
const float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741; //steinhart-hart coeficients for thermistor
const int kelvin = 273.15;

// Settings
const int clockRate = 100;         // ms
int testRate = seconds(2);   // how often sensor readings are taken, and how window delays for
int liveRate = seconds(240); // 4mins

const int pumpWaitTest = seconds(120);
const int pumpWaitLive = seconds(86400); // one day
const int pumpRunTestRate = seconds(30); // how long the pump runs for
const int pumpRunLiveRate = seconds(120);

// Variables
int Vo;
float logR2, R2, T, vOut, tempRangeHigh, tempRangeLow;
float averagedTemp;
int samplingLoop; // keeps track of internal 'sampling' loop
int windowLoop;   // keeps track of windows opening and closing
int pumpLoop;     // keeps track of pumping check state
int pumpRun = 0;
int digitalTemperature;
int digitalHumidity;

// States
bool windowOpen = false;      // Will need to init off a switch i guess
bool pumpRunning = false;
bool testMode = true;
int error = 0;
int targetTemp = 23;

// Test or live rates - sampler
int getInterval() {
//  if (!testMode) {
//    return liveRate;
//  }
  return testRate;
}

// Test or live rates - pump only
// wait = wait time otherwise run time
int getPumpRate(bool wait)
{
  if (!testMode)
  {
    return wait ? pumpWaitLive : pumpRunLiveRate;
  }
  return wait ? pumpWaitTest : pumpRunTestRate;
}

void closeWindow() {
  Serial.println("close window");
  digitalWrite(windowPin, LOW);
  windowOpen = false;
}

void openWindow() {
  Serial.println("open window");
  digitalWrite(windowPin, HIGH);
  windowOpen = true;
}

void runPump() {
  Serial.println("run pump");
  digitalWrite(pumpPin, HIGH);
  pumpRunning = true;
}

void stopPump() {
  Serial.println("stop pump");
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
  Serial.println("ERROR ");
  Serial.println(error);
  digitalWrite(errLed, HIGH);
  error = errorCode;
}

void setup() {
  Serial.begin(9600);

  // Pin setup
  pinMode(plusPin, INPUT);
  pinMode(minusPin, INPUT);
  pinMode(modeSelektor, INPUT);

  pinMode(pumpPin, OUTPUT);
  pinMode(windowPin, OUTPUT);
  pinMode(testLed, OUTPUT);
  pinMode(errLed, OUTPUT);

  dht.begin();
  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  Serial.println(" BEGIN ");

  // Test sequence
//  openWindow();
//  delay(seconds(10));
//  closeWindow();
//  delay(1000);
//  runPump();
//  delay(seconds(4));
//  stopPump();
//  delay(1000);
//  digitalWrite(normalLed, HIGH);
//  delay(1000);
//  digitalWrite(normalLed, LOW);
//  delay(1000);
//  digitalWrite(testLed, HIGH);
//  delay(1000);
//  digitalWrite(testLed, LOW);
//  delay(1000);
//  digitalWrite(errLed, HIGH);
//  delay(1000);
//  digitalWrite(errLed, LOW);
//  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@");
}

void loop() {
  if (digitalRead(modeSelektor) == HIGH)
  {
    testMode = true;
    digitalWrite(testLed, HIGH);
    digitalWrite(normalLed, LOW);
  }
  else
  {
    testMode = false;
    digitalWrite(testLed, LOW);
    digitalWrite(normalLed, HIGH);
  }

  // Are we running a sample in this loop?
  if (samplingLoop > getInterval())
  {
    samplingLoop = 0;

    // Get humidity event and print its value.
    digitalHumidity = dht.readHumidity();
    Serial.println(F("Digital humidity: "));
    Serial.println(digitalHumidity);
    Serial.println(F("%"));

    // Therminsor calculations
    Vo = analogRead(thermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0); //calculate resistance on thermistor
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)); // temperature in Kelvin
    T = T - 273.15;                                             //convert Kelvin to Celcius

    Serial.println("Analogue temperature ");
    Serial.println(T);
    Serial.println("c");

    digitalTemperature = dht.readTemperature();
    Serial.println(F("Digital temperature: "));
    Serial.println(digitalTemperature);
    Serial.println(F("°C"));

    averagedTemp = (averagedTemp + ((T + digitalTemperature) * 0.5)) * 0.5;

    if (isnan(digitalTemperature))
    {
      handleError(1);
    }

    if (Vo == 0 || T > 60)
    {
      averagedTemp = digitalTemperature;
      handleError(2);
    }

    if (digitalHumidity == 0)
    {
      handleError(3);
    }

    Serial.println("Average temperature: ");
    Serial.println(averagedTemp);
    Serial.println(" c");

    Serial.println("Target temperature is :");
    Serial.println(targetTemp);
    Serial.println(" c");

    Serial.println("***********************************");
  }
  // End temperature reading

  // Are we running a window check this loop?
  if (windowLoop >= getInterval())
  {
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    Serial.println("Window open state check : ");
    Serial.println(windowOpen);

    // It's too hot, open if not already open
    if (averagedTemp >= targetTemp) {
      Serial.println("it's too hot");
      if (!windowOpen)
      {
        openWindow();
      }
    }
    // Too cold, close window if open
    if (averagedTemp < targetTemp) {
      Serial.println("it's too cold");
      if (windowOpen) {
        closeWindow();
      }
    }
    windowLoop = 0;
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  };

//   Is it time to run the pump?
  if (pumpLoop >= getPumpRate(true))
  {
    Serial.println("###############################");

    runPump();
    pumpLoop = 0;

    Serial.println("###############################");
  };

Serial.print("here");
Serial.println(getInterval());
  delay(getInterval());

// Stuff that needs checking every loop e.g. button, switch status, pump state

// Temp button control
if (digitalRead(plusPin) == HIGH) {
    targetTemp = targetTemp + 1;
    Serial.println("targetTemp changed to ");
    Serial.println(targetTemp);
}

if (digitalRead(minusPin) == HIGH) {
    targetTemp = targetTemp - 1;
    Serial.print("targetTemp changed to ");
    Serial.println(targetTemp);
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

// Increment each check counter

Serial.println(samplingLoop);

samplingLoop = (samplingLoop + clockRate);
windowLoop = (windowLoop + clockRate);
pumpLoop = (pumpLoop + clockRate);
delay(clockRate);
}
