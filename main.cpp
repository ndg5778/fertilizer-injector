#include "main_header.h"

void setup()
{
  Serial.begin(115200);
  ec.begin();
  ph.begin();
  dht.begin();
  lcd.init();          
  lcd.backlight();

  xTaskCreate(taskPH_EC, "PH_EC_Task", STACK_SIZE_PH_EC, NULL, 1, &phEcTaskHandle);
  xTaskCreate(taskDHT, "DHT_Task", STACK_SIZE_DHT, NULL, 1, &dhtTaskHandle);
  xTaskCreate(taskStandardValuePH, "Standard_Value_PH_Task", STACK_SIZE_VALUE, NULL, 1, &standardValuePHTaskHandle);
  xTaskCreate(taskStandardValueEC, "Standard_Value_EC_Task", STACK_SIZE_VALUE, NULL, 1, &standardValueECTaskHandle);
  xTaskCreate(taskSetupValue, "Setup_Value_Task", STACK_SIZE_SETUP, NULL, 1, &setupValueTaskHandle);
  xTaskCreate(taskPrintValue, "Print_Value_Task", STACK_SIZE_PRINT, NULL, 1, &printValueTaskHandle);
}

void loop()
{
  // Not Used
}

void taskPrintValue(void* pvParameters) {
  while (1) {
    lcd.setCursor(0,0);
    lcd.print("PH:"); 
    lcd.setCursor(0,1);
    lcd.print("EC:"); 

    // PH
    lcd.setCursor(4, 0);
    lcd.print(phValue, 2);
    lcd.setCursor(9, 0);
    lcd.print("/");
    lcd.setCursor(11, 0);
    lcd.print(standardPH, 2);

    // EC
    lcd.setCursor(4, 1);
    lcd.print(ecValue, 2);
    lcd.setCursor(9, 1);
    lcd.print("/");
    lcd.setCursor(11, 1);
    lcd.print(standardEC, 2);

    vTaskDelay(PRINT_DELAY);
  }
}

// button으로 기준값 변경
void taskSetupValue(void* pvParameters) {
  pinMode(PH_UP_BTN, INPUT_PULLUP);
  pinMode(PH_DOWN_BTN, INPUT_PULLUP);
  pinMode(EC_UP_BTN, INPUT_PULLUP);
  pinMode(EC_DOWN_BTN, INPUT_PULLUP);
  
  // EEPROM.get(EEPROM_PH, standardPH);
  // EEPROM.get(EEPROM_EC, standardEC);

  // if(isinf(standardPH)) {
  //   standardPH = 6.0;
  // }
  // if(isinf(standardEC)) {
  //   standardEC = 1.2;
  // }

  bool lastPHUpBtnState = LOW;
  bool lastPHDownBtnState = LOW;
  bool lastECUpBtnState = LOW;
  bool lastECDownBtnState = LOW;

  while (1) {
    bool phUpBtnState = digitalRead(PH_UP_BTN);
    bool phDownBtnState = digitalRead(PH_DOWN_BTN);
    bool ecUpBtnState = digitalRead(EC_UP_BTN);
    bool ecDownBtnState = digitalRead(EC_DOWN_BTN);

    if(phUpBtnState != lastPHUpBtnState) {
      if (phUpBtnState == HIGH) {
        standardPH += 0.1;
        // EEPROM.put(EEPROM_PH, standardPH);
        Serial.print("PH standard : ");
        Serial.println(standardPH);
      } 
    }

    if(phDownBtnState != lastPHDownBtnState) {
      if (phDownBtnState == HIGH) {
        standardPH -= 0.1;
        // EEPROM.put(EEPROM_PH, standardPH);
        Serial.print("PH standard : ");
        Serial.println(standardPH);
      } 
    }
    
    if(ecUpBtnState != lastECUpBtnState) {
      if (ecUpBtnState == HIGH) {
        standardEC += 0.1;
        // EEPROM.put(EEPROM_EC, standardEC);
        Serial.print("EC standard : ");
        Serial.println(standardEC);
      } 
    }

    if(ecDownBtnState != lastECDownBtnState) {
      if (ecDownBtnState == HIGH) {
        standardEC -= 0.1;
        // EEPROM.put(EEPROM_EC, standardEC);
        Serial.print("EC standard : ");
        Serial.println(standardEC);
      } 
    }

    lastPHUpBtnState = phUpBtnState;
    lastPHDownBtnState = phDownBtnState;
    lastECUpBtnState = ecUpBtnState;
    lastECDownBtnState = ecDownBtnState;

    ph.calibration(voltagePH,temperature);
    ec.calibration(voltageEC,temperature);
    vTaskDelay(BTN_DELAY);
  }
}

// PH의 측정값이 설정값 이하면 motor 실행
void taskStandardValuePH(void* pvParameters) {
  pinMode(LED_PH_PIN, OUTPUT);
  pinMode(PHMotor_Relay, OUTPUT);
  pinMode(Pump_Motor_Relay, OUTPUT);

  while (1) {
    if (phValue < standardPH) {
      // motor on
      digitalWrite(PHMotor_Relay, MotorOn);
      digitalWrite(Pump_Motor_Relay, PumpMotorOn);
      digitalWrite(LED_PH_PIN, HIGH);
    }
    else {
      // motor off
      digitalWrite(PHMotor_Relay, MotorOff);
      digitalWrite(Pump_Motor_Relay, PumpMotorOff);
      digitalWrite(LED_PH_PIN, LOW);
    }

    vTaskDelay(MOTOR_DELAY);
  }
}

// EC의 측정값이 설정값 이하면 motor 실행
void taskStandardValueEC(void* pvParameters) {
  pinMode(LED_EC_PIN, OUTPUT);
  pinMode(ECMotor_Relay, OUTPUT);

  while (1) {
    if (ecValue < standardEC) {
      // motor on
      digitalWrite(ECMotor_Relay, MotorOn);
      digitalWrite(LED_EC_PIN, HIGH);
    }
    else {
      // motor off
      digitalWrite(ECMotor_Relay, MotorOff);
      digitalWrite(LED_EC_PIN, LOW);
    }

    vTaskDelay(MOTOR_DELAY);
  }
}

// temp, ph, ec값 받아오기
void taskPH_EC(void* pvParameters) {
  while (1) {
    // float temp = readTemperature();
    // temperature = (temp == (-1)) ? 25 : temp;
    // Serial.print("temperature:");
    // Serial.print(temperature, 2);

    voltagePH = analogRead(PH_PIN) / 1024.0 * 5000;
    phValue = ph.readPH(voltagePH, temperature);
    Serial.print("pH:");
    Serial.println(phValue, 2);

    vTaskDelay(PH_EC_DELAY);

    voltageEC = analogRead(EC_PIN) / 1024.0 * 5000;
    ecValue = ec.readEC(voltageEC, temperature);
    Serial.print("EC:");
    Serial.print(ecValue, 2);
    Serial.println("ms/cm");

    vTaskDelay(PH_EC_DELAY);
  }
}

// 온도값 return하기
void taskDHT(void* pvParameters) {
  // (void)pvParameters;
  while (1)
  {
    float temp = readTemperature();
    temperature = (temp == (-1)) ? 25 : temp;
    vTaskDelay(DHT_DELAY);
  }
}

// DHT11값 받아오기
float readTemperature() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    // Serial.println(F("Failed to read from DHT sensor!"));
    return -1;
  }


  return t;
}
