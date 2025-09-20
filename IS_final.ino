#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MAX1704X.h>
#include <BH1750.h>
#include <RTClib.h>
#include <LowPower.h>

// I2C addresses
#define BME280_ADDR 0x77

// SD card chip‐select
#define SD_CS_PIN   3

// MAX471 analog inputs
#define VT_PIN     A0    // module’s VT output (5× divider)
#define IT_PIN    A1    // module’s AT output (shunt amplifier)

// ADC & shunt parameters
const float VREF          = 3.3f;  // Pro Mini Vcc
const float VDIV_FACTOR   = 5.0f;  // MAX471 on‐board divider
const float SENSE_V_PER_A = 0.1f;  // 100 mV per amp

// BME280, RTC, fuel‐gauge and BH1750FVI instances
Adafruit_BME280    bme;
RTC_DS3231         rtc;
Adafruit_MAX17048  maxlipo;
BH1750 lightMeter;

// wake‐tick
static uint16_t tick = 0;
static uint32_t lastLogTime = 0;

// hPa → mmHg
float hPa_to_mmHg(float h) { return h * 0.750062f; }

void setup() {
  Wire.begin();

  Serial.begin(57600);
  delay(10);
  Serial.println(F("Starting logger…"));

  if (!bme.begin(BME280_ADDR)) while (1);
  if (!rtc.begin())          while (1);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  delay(200);

  // Init MAX17048 fuel gauge
  if (!maxlipo.begin()) while (1);

  // Init BH1750FVI
  lightMeter.begin();

  // SD init + header
  if (SD.begin(SD_CS_PIN)) {
    if (!SD.exists("log.csv")) {
      File f = SD.open("log.csv", FILE_WRITE);
      if (f) {
        f.println(F("Date,Time,Temp_C,Pressure_mmHg,Humidity_%,Batt_V,Batt_%,ChargeRate_%/hr,Load_mA,Lux_lx"));
        f.close();
      }
    }
  }

}

void loop() {
  // —— Sleep ~8 seconds at a time —— 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  DateTime now = rtc.now();
  if ((now.unixtime() - lastLogTime) < 60) return; // Only log once 30 minutes (1800 seconds)
  lastLogTime = now.unixtime();


  // Read BME280
  float tempC     = bme.readTemperature();
  float pressMmHg = hPa_to_mmHg(bme.readPressure() / 100.0F);
  float humidity  = bme.readHumidity();

  // Read DS3231 RTC
  char dateStr[11], timeStr[9];
  snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d",
           now.day(), now.month(), now.year());
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // Read battery voltage
  uint16_t rawV = analogRead(VT_PIN);
  float vAtA0   = rawV * (VREF / 1023.0f);
  float loadV   = vAtA0 * VDIV_FACTOR;
  ADCSRA &= ~_BV(ADEN); // —— Disable ADC to save µA —— 

  // Read load current
  uint16_t rawI = analogRead(IT_PIN);
  float vAtA1   = rawI * (VREF / 1023.0f);
  float vDiff   = vAtA1 - (VREF / 2.0f);
  float loadmA_neg   = vDiff / SENSE_V_PER_A;
  float loadmA  = loadmA_neg * -1.00f;
  ADCSRA &= ~_BV(ADEN); // —— Disable ADC to save µA —— 

  // read voltage & % from MAX17048
  float battV = maxlipo.cellVoltage();
  float battP = maxlipo.cellPercent();
  float chargeRate = maxlipo.chargeRate();
  ADCSRA &= ~_BV(ADEN); // —— Disable ADC to save µA —— 

  // Read BH1750FVI lux
  float lux = lightMeter.readLightLevel();
  ADCSRA &= ~_BV(ADEN); // —— Disable ADC to save µA —— 

  // Re-init SD/SPI
  SPI.begin();
  if (!SD.begin(SD_CS_PIN)) return;

  // Append to SD
  File log = SD.open("log.csv", FILE_WRITE);
  if (log) {
    log.print(dateStr);      log.print(',');
    log.print(timeStr);      log.print(',');
    log.print(tempC, 2);     log.print(',');
    log.print(pressMmHg, 2); log.print(',');
    log.print(humidity, 2);  log.print(',');
    log.print(battV, 3);     log.print(',');
    log.print(battP, 2);     log.print(',');
    log.print(chargeRate, 2);     log.print(',');
    log.print(loadmA, 2);     log.print(',');
    log.println(lux, 2);
    log.close();
  }

  if (battV < 3.2 || battP < 10.0) {
  // Save power and skip SD write
  return;
}
}
