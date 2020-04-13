/*
 * DISPLAY INCLUDE SECTION
 */
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>

/*
 * SENSOR INCLUDE SECTION
 */
#include <Adafruit_BME280.h>

/*
 * RGB LED INCLUDE SECTION
 */
#include <NeoPixelPainter.h>

/*
 * FFT INCLUDE SECTION
 */
#include <arduinoFFT.h>

#define DSP_I2C_ADDRESS     0x3C

#define SCREEN_WIDTH    128   // OLED display width, in pixels
#define SCREEN_HEIGHT   64    // OLED display height, in pixels

#define LED_PIN         2
#define LED_COUNT       1

#define H_OFFSET        0.0
#define T_OFFSET        0.0
#define P_OFFSET        0.0

#define SAMPLES         64    // Must be a power of 2
#define xres            32    // Total number of  columns in the display, must be <= SAMPLES/2

#define MIN_MILLS_ON    10
#define MIN_MILLSW_ON   20

#define MIN_MILLS_OFF   10
#define MIN_MILLSW_OFF  20

#define N_LEDS          5

#define LEDW            3
#define LEDR            5
#define LEDG            6
#define LEDB            9
#define LEDY            10

#define PLW             0
#define PLR             1
#define PLG             2
#define PLB             3
#define PLY             4

#define FREQ_OFFSET     100

arduinoFFT FFT = arduinoFFT();
SSD1306AsciiAvrI2c display;
Adafruit_BME280 bme;
Adafruit_NeoPixel rgbleds = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

float temperature = -1, humidity = -1, pressure = -1;
float t_min = -1, t_max = -1;
float h_min = -1, h_max = -1;
float p_min = -1, p_max = -1;

bool first = true;

uint8_t seconds_counter = 0;

double vReal[SAMPLES];
double vImag[SAMPLES];
uint8_t data_avgs[xres];
double x;
double v;
uint32_t s = -1, e = -1;
float freq = 0.;

uint32_t smills[N_LEDS] = {0, 0, 0, 0, 0}, emills[N_LEDS] = {0, 0, 0, 0};
uint8_t status_led[N_LEDS] = {LOW, LOW, LOW, LOW, LOW};
uint8_t queue_led[N_LEDS] = {0, 0, 0, 0, 0};
//uint32_t emills[N_LEDS] = {0, 0, 0, 0, 0};

uint8_t leds[N_LEDS] = {LEDW, LEDR, LEDG, LEDB, LEDY};

ISR(TIMER1_COMPA_vect) {
  seconds_counter++;
  if (seconds_counter > 5) seconds_counter = 5;
}

void i2cScanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16)
        Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.println(F("!"));

      nDevices++;
    } else if (error == 4) {
      Serial.print(F("Error at address 0x"));
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println(F("No I2C device"));
  else
    Serial.println(F("done"));
}

uint8_t addQueue(uint8_t id) {
  printf("ADD: %d\n", id);
  uint8_t tmp[N_LEDS] = {0, 0, 0, 0, 0};
  uint8_t len = 0, r = 0;
  for (uint8_t i = 0; i < N_LEDS; i++) {
    if(queue_led[i] <= 0) {
      len = (i);
      break;
    }
  }
  bool f = false;
  uint8_t k = 0;
  for (uint8_t i = 0; i < N_LEDS; i++) {
    if(queue_led[i] == id) {
      f = true;
      break;
    }
  }
  if (len >= 2 && !f) {
    r = queue_led[0];
    k = 1;
  }
  for (uint8_t i = 0; i < N_LEDS; i++) {
    if(queue_led[k] == id) ++k;
    if(k >= N_LEDS) break;
    if(queue_led[k] <= 0) {
      tmp[i] = id;
      break;
    }
    tmp[i] = queue_led[k];
    ++k;
  }
  for (uint8_t i = 0; i < N_LEDS; i++) queue_led[i] = tmp[i];
  return (r);
}

#define LED_ON(d, _m) {\ 
  status_led[d] = HIGH;\
  digitalWrite(leds[d], HIGH);\ 
  smills[d] = _m;\ 
  emills[d] = 0;\ 
}

#define LED_OFF(d, _m) {\ 
  status_led[d] = LOW;\
  digitalWrite(leds[d], LOW);\ 
  emills[d] = _m;\ 
  smills[d] = 0;\ 
}

void ledON() {
  uint32_t _m = millis();
  if (v >= 25) {
    uint8_t d = (uint8_t)(((uint16_t)((x) / 200.)) % N_LEDS);
//    uint8_t res = addQueue(leds[d]);
//    if(res > 0) digitalWrite(res, LOW);

//    Serial.print("d: "); Serial.println(d);
//    Serial.print("emills[d]: "); Serial.println(emills[d]);
//    Serial.print("(_m - emills[d]): "); Serial.println((_m - emills[d]));
//    Serial.println();
    if((_m - emills[d]) < MIN_MILLS_OFF) return;

    for (int i = 0; i < N_LEDS; i++) {
      if (i != PLW && i != d && ((_m - smills[i]) >= MIN_MILLS_ON))
        LED_OFF(i, _m)
      else if(i == PLW && i != d && ((_m - smills[i]) >= MIN_MILLSW_ON))
        LED_OFF(i, _m)
    }

    uint8_t ons = 0;
    for (uint8_t j = 0; j < N_LEDS; j++) {
      if(status_led[j] == HIGH) ++ons;
    }
    if(ons > 2) {
      Serial.print("CUT DOWN: "); Serial.println(ons);
      uint32_t oldest = 0xFFFFFFFF;
      uint8_t oldest_id = 0xFFFFFFFF;
      for (uint8_t j = 0; j < N_LEDS; j++) {
        if(status_led[j] == HIGH && smills[j] != 0) {
          if(oldest > smills[j]) {
            oldest = smills[j];
            oldest_id = j;
          }
        }
      }
      uint32_t _m = millis();
      LED_OFF(oldest_id, _m)
    }

//    Serial.print("ons: "); Serial.println(ons);

    LED_ON(d, _m);
    
    smills[d] = _m;
    //Serial.println();
  } else {
    for (int i = 0; i < N_LEDS; i++) {
      if (i != PLW && ((_m - smills[i]) >= MIN_MILLS_ON)) digitalWrite(leds[i], LOW);
      else if(i == PLW && ((_m - smills[i]) >= MIN_MILLSW_ON)) digitalWrite(leds[i], LOW);
    }
  }
}

void setup() {
  pinMode(LEDW, OUTPUT);
  digitalWrite(LEDW, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDW, OUTPUT);
  pinMode(LEDY, OUTPUT);
  pinMode(LEDB, OUTPUT);

  first = true;

  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDW, LOW);
  digitalWrite(LEDY, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  delay(1);

//  digitalWrite(LEDR, HIGH);
//  digitalWrite(LEDG, HIGH);
//  digitalWrite(LEDW, HIGH);
//  digitalWrite(LEDY, HIGH);
//  digitalWrite(LEDB, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDW, LOW);
  digitalWrite(LEDY, LOW);
  digitalWrite(LEDB, LOW);

  for (int i = 0; i < N_LEDS; i++) {
    for (int j = 0; j < N_LEDS; j++) digitalWrite(leds[j], LOW);
    digitalWrite(leds[i], HIGH);
    delay(500);
  }

  for (int j = 0; j < N_LEDS; j++) digitalWrite(leds[j], LOW);
  digitalWrite(LEDW, HIGH);
  delay(500);

    for (int j = 0; j < N_LEDS; j++) digitalWrite(leds[j], LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDY, HIGH);
  delay(500);

    for (int j = 0; j < N_LEDS; j++) digitalWrite(leds[j], LOW);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(500);

  digitalWrite(LEDW, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDY, LOW);

  Wire.begin();
  Serial.begin(115200);

  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //allow interrupts

  i2cScanner();
  
  ADCSRA = 0b11100101;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX = 0b00000000;       // use pin A0 and external voltage reference
  digitalWrite(LED_BUILTIN, LOW);

  // Init OLED display
  display.begin(&Adafruit128x64, DSP_I2C_ADDRESS);
  delay(500);
  display.setFont(System5x7);
  display.clear();
  display.print(F("DSPINIT"));
  Serial.println(F("DSPINIT"));
  delay(500);
  
  digitalWrite(LED_BUILTIN, HIGH);

  display.clear();
  display.setCursor(0, 0);
  display.println(F("LOADING..."));
  display.println(F("BOOTING ATMega328P..."));
  display.println(F("#####################"));
  display.println(F("T/H"));
  display.println(F("by Niccolo' Ferrari"));
  display.println(F("@madnick_93"));
  display.println(F("CRIME FOR CLIMB"));
  display.println(F("OMEGASOFTWARE"));

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  if (! bme.begin(0x76)) {
    Serial.println(F("BME280 BEGIN ERROR!"));
  } else {
    Serial.println(F("BME280 BEGIN OK!"));
  }

  // weather monitoring
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

  rgbleds.begin();  // Call this to start up the LED strip.
  rgbleds.setPixelColor(0, 0x0);
  rgbleds.show();   // ...but the LEDs don't actually update until you call this.
                  
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if(first || (seconds_counter) >= 5) {
    seconds_counter = 0;
    
    bme.takeForcedMeasurement();

    float _t = (float) bme.readTemperature();
    float _h = (float) bme.readHumidity();

    float _p = (float) bme.readPressure() / 100.0F;

    if (!isnan(_h) && !isnan(_t) && !isnan(_p)) {
      humidity = _h + H_OFFSET;
      temperature = _t + T_OFFSET;
      pressure = _t + P_OFFSET;

      if (first) {
        t_min = temperature;
        t_max = temperature;
        h_max = humidity;
        h_min = humidity;
        p_max = pressure;
        p_min = pressure;
      } else {
        if (temperature > t_max) {
          t_max = temperature;
        }
        if (humidity > h_max) {
          h_max = humidity;
        }
        if (pressure > p_max) {
          p_max = pressure;
        }
        if (temperature < t_min) {
          t_min = temperature;
        }
        if (humidity < h_min) {
          h_min = humidity;
        }
        if (pressure < p_min) {
          p_min = pressure;
        }
      }
    } else {
      
    }

    display.clear();
    display.setCursor(0, 0);

    display.print(F("T:  "));
    display.print(temperature);
    display.print(F(" "));
    display.print((char)247);
    display.println(F("C"));
  
    display.print(F("RH: "));
    display.print(humidity);
    display.println(F(" %"));
  
    display.print(F("P: "));
    display.print(pressure);
    display.println(F(" hPa"));
  
    display.print(F("T Max: "));
    display.println(t_max);
    display.print(F("T Min: "));
    display.println(t_min);
    display.print(F("RH Max: "));
    display.println(h_max);
    display.print(F("RH Min: "));
    display.println(h_min);
    display.print(F("P MaMi: "));
    display.print(p_max);
    display.print(F("; "));
    display.println(p_min);
    
    if (temperature >= 25 || humidity >= 75) {
      rgbleds.setPixelColor(0, 0x4F0000);
      rgbleds.show();
    } else if (temperature >= 18 || humidity >= 60) {
      rgbleds.setPixelColor(0, 0x4F4F00);
      rgbleds.show();
    } else if (temperature >= 18 || humidity >= 50) {
      rgbleds.setPixelColor(0, 0x0A4F00);
      rgbleds.show();
    } else {
      rgbleds.setPixelColor(0, 0x004F00);
      rgbleds.show();
    }
  }

  uint8_t i = 0;
  s = micros();
  // ++ Sampling
  for (i = 0; i < SAMPLES; i++) {
    while (!(ADCSRA & 0x10));       // wait for ADC to complete current conversion ie ADIF bit set
    ADCSRA = 0b11110101 ;               // clear ADIF bit so that ADC can do next operation (0xf5)
    int value = ADC - (512 + 66) ;                 // Read from ADC and subtract DC offset caused value
    //Serial.println(value);
    vReal[i] = value / 8;                   // Copy to bins after compressing
    vImag[i] = 0;

    delayMicroseconds(100);
  }
  // -- Sampling
  e = micros();
  freq = 1. / ((((float)(e - s)) * 0.000001) / (float)SAMPLES);

  // ++ FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  // -- FFT

  FFT.MajorPeak(vReal, SAMPLES, freq, &x, &v);

  float bass0 = vReal[0];
  float bass1 = vReal[1];
  float bass2 = vReal[2];
  float bass3 = vReal[3];
  float bass4 = vReal[4];
  float bass5 = vReal[5];
  float bass6 = vReal[6];
  float bass7 = vReal[7];
  float bass8 = vReal[8];
  float bass9 = vReal[9];

  //Serial.print(">>>x: "); Serial.print(x); Serial.print(" v: "); Serial.println(v);

  ledON();

  if (/*bass0 >= 450 ||*/
        bass1 >= 200 ||
        bass2 >= 200 ||
        bass3 >= 150 ||
        bass4 >= 150 ||
        bass5 >= 150 ||
        bass6 >= 150 ||
        bass7 >= 100 ||
        bass8 >= 100 ||
        bass9 >= 100 ||
        //bass10 >= 275 ||
        (x >= FREQ_OFFSET && v >= 100)) {
      
    digitalWrite(LED_BUILTIN, HIGH);
    
//    uint8_t res = addQueue(PLW);
//    if(res > 0) digitalWrite(res, LOW);

    uint32_t _m = millis();
    LED_ON(PLW, _m)
    
//    digitalWrite(leds[PLW], HIGH);
//    smills[PLW] = millis();
  } else {
    //emills[PLW] = millis();
    uint32_t _m = millis();
    
    if (status_led[PLW] == HIGH && (_m - smills[PLW]) >= MIN_MILLSW_ON) {
      //digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      LED_OFF(PLW, _m)
//      digitalWrite(LEDW, LOW);
    }
  }

  first = false;
}
