#include <dummy.h>
#include <sps30.h>
#include "SCD30.h"
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

#define scd_debug 0

// Network and MQTT broker parameters
#define wifi_ssid "ssid"
#define wifi_password "pass"
#define mqtt_server "IP"
#define mqtt_user "user"
#define mqtt_password "pass"

#define co2_topic "chemical_SCD30/CO2"
#define temp_topic "chemical_SCD30/TEMP"
#define humi_topic "chemical_SCD30/HUMI"

#define pm1_topic "pm_SPS30/PM1"
#define pm25_topic "pm_SPS30/PM25"
#define pm10_topic "pm_SPS30/PM10"

static int64_t lastMeasurementTimeMs = 0;
static int measurementIntervalMs = 5000;
int measureCycle = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  delay(100);
  
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;

  Wire.begin();
  scd30.initialize();
  scd30.setAutoSelfCalibration(1);
  scd30.setTemperatureOffset(1);

  delay(2000);

  sensirion_i2c_init();
  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
  }

  ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }

  delay(1000);

  setup_wifi();
  connectMQTT();
}

void setup_wifi() {
  delay(10);
  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker…");

    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("Connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry 5 in seconds");
      // set a different delay if having fast connection
      delay(5000);
    }
  }
}

boolean isValidMeasurement(float measurement) {
  return measurement < 10000.0 && measurement > -10000.0 && measurement != 0.0;
}

boolean isValidDisplayValue(String value) {
  return value.length() < 10;
}

void loop() {
  if((WiFi.status() != WL_CONNECTED) || !client.connected()) {
    setup();
  }

  client.loop();

  delay(10000);

  measureCO();
  measureCycle = measureCycle + 1;

  if(measureCycle < 6) {
    //do nothing
  } else {
    sps30_wake_up();
    delay(1000);
    sps30_start_measurement();
    measurePM();
    delay(1000);
    sps30_stop_measurement();
    delay(1000);
    sps30_sleep();
    measureCycle = 0;
  }
}

void measureCO() {
 float result[3] = {0};
  if (millis() - lastMeasurementTimeMs >= measurementIntervalMs) {
    if (scd30.isAvailable()) {
      scd30.getCarbonDioxideConcentration(result);
      float co2 = result[0];
      float temp = result[1];
      float humi = result[2];

      //Print
      String co2DisplayValue = String(co2).c_str();
      String tempDisplayValue = String(temp).c_str();
      String humiDisplayValue = String(humi).c_str();
      Serial.print("CO2(ppm):");
      Serial.println(co2DisplayValue);
      Serial.print("Temperature:");
      Serial.println(tempDisplayValue);
      Serial.print("humidity:");
      Serial.println(humiDisplayValue);

      // Publish MQTT topic
      if (isValidMeasurement(co2) && isValidMeasurement(temp) || isValidMeasurement(humi)) {
        if(isValidDisplayValue(co2DisplayValue) && isValidDisplayValue(tempDisplayValue) && isValidDisplayValue(humiDisplayValue)) {
          client.publish(co2_topic, String(co2).c_str(), true);
          client.publish(temp_topic, String(temp).c_str(), true);
          client.publish(humi_topic, String(humi).c_str(), true);
        }
      }

      lastMeasurementTimeMs = millis();
    }
  }
  delay(500);
}

void measurePM() {
  //SPS30
  struct sps30_measurement m;
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;

  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("error reading data-ready flag: ");
      Serial.println(ret);
    } else if (!data_ready)
      Serial.print("data not ready, no new measurement available\n");
    else
      break;
    delay(100); /* retry in 100ms */
  } while (1);

  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("error reading measurement\n");
  } else {
    Serial.print("PM  1.0: ");
    Serial.println(m.mc_1p0);
    Serial.print("PM  2.5: ");
    Serial.println(m.mc_2p5);
    Serial.print("PM  4.0: ");
    Serial.println(m.mc_4p0);
    Serial.print("PM 10.0: ");
    Serial.println(m.mc_10p0);
    Serial.println();

    client.publish(pm1_topic, String(m.mc_1p0).c_str(), true);
    client.publish(pm25_topic, String(m.mc_2p5).c_str(), true);
    client.publish(pm10_topic, String(m.mc_10p0).c_str(), true);

  }

  delay(1000);
}
