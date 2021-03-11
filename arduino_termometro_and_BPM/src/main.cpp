#include <Arduino.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "BluetoothSerial.h"
#include <Adafruit_MLX90614.h>
#include <EEPROM.h>

// No other Address options.
#define DEF_ADDR 0x55
#define TERM_ADDR 0x5A

#define LED_PIN 5

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



SparkFun_Bio_Sensor_Hub bioHub(4, 15); // Takes address, reset pin, and MFIO pin.
bioData body;

BluetoothSerial SerialBT;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

char str_aux[100];
uint8_t sensor_sel;

void setup(){
  // Initilize hardware:
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  EEPROM.begin(1);

  sensor_sel = EEPROM.read(0);
  EEPROM.write(0, !sensor_sel);
  EEPROM.commit();

  Wire.begin();
  
  if(sensor_sel){
    int result = bioHub.begin();
    if (!result)
      Serial.println("Sensor started!");
    else
      Serial.println("Could not communicate with the sensor!!!");

    Serial.println("Configuring Sensor....");
    int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings.
    if(!error){
      Serial.println("Sensor configured.");
    }
    else {
      Serial.println("Error configuring sensor.");
      Serial.print("Error: ");
      Serial.println(error);
    }
  }else{
    mlx.begin();
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up.
  delay(4000);

}

void loop(){

  if(sensor_sel){
    body = bioHub.readBpm();
    sprintf(str_aux, "{'HR': %d, 'Confidence': %d, 'SpO2': %d, 'Status': %d}" , body.heartRate, body.confidence, body.oxygen, body.status);
  }else{
    double temp_obj = mlx.readObjectTempC();
    double temp_amb = mlx.readAmbientTempC();

    sprintf(str_aux, "{'Object Temperature': %.1f, 'Ambient Temperature': %.1f}", temp_obj, temp_amb);
  }

  /*sprintf(str_aux, "{'HR': %d, 'Confidence': %d, 'SpO2': %d, 'Status': %d, 'Object Temperature': %.1f, 'Ambient Temperature': %.1f}"
  , body.heartRate, body.confidence, body.oxygen, body.status, temp_obj, temp_amb);*/

  Serial.println(str_aux);
  SerialBT.write((uint8_t*)str_aux, strlen(str_aux)+1);
  delay(500); // Slowing it down, we don't need to break our necks here.

}