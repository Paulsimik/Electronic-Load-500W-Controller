//##############################//
//      STM Electronic Load     //
//          Version 1.5         //
//            By Paul           //
//##############################//

//  1.0
//  - Release  
//  1.1
//  - Added average temperature
//  - Added value Fan PWM delay
//  - Improvements
//  - Added timer while output ON
//  1.2
//  - Added check limits
//  1.3
//  - LCD write optimalization
//  - Fan run optimalization
//  1.4
//  - Rewrite encoder library
//  - Added Menu Main for select functions
//  - Added Battery Load mode
//  1.5
//  - Added Setup Menu
//  - Prepare calibration menu
//  - Voltage calibration
//  - Current Calibration

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <OneButton.h>
#include <RTClib.h>
#include <Buzzer.h>
#include <Encoder.h>
#include <FanControl.h>

#define UPDATE_CAPACITY         1000
#define UPDATE_VALUES           350
#define STATUS_INTERVAL         3000
#define VOLTAGE_MAX_LIMIT       99
#define CURRENT_MAX_LIMIT       50
#define POWER_MAX_LIMIT         500
#define TEMPERATURE_MAX_LIMIT   60

//=======================================================================//
//                             DEFINITION                                //
//=======================================================================//

#define LED_FAULT         PB10
#define LED_ENABLE        PB11
#define ENABLE_PIN        PB12
#define BTN_ENABLE        PB13
#define BUZZER_PIN        PB14
#define REFERENCE_PIN     PA8
#define FAN_PWM           PA9
#define FAN_FB            PA10
#define BTN_ENCODER       PA15

Adafruit_ADS1115 ads;
hd44780_I2Cexp lcd;
RTC_DS1307 rtc;
OneButton enBtn(BTN_ENABLE);
OneButton encBtn(BTN_ENCODER);
Encoder encoder;
Buzzer buzzer; 

enum Status
{
  NORMAL_STATUS,
  FAULT,
  VOLTAGE_LIMIT,
  CURRENT_LIMIT,
  POWER_LIMIT,
  TEMPERATURE_LIMIT,
  SELF_TEST_ERROR
};

enum Status currentStatus = NORMAL_STATUS;

//=======================================================================//
//                                 MENU                                  //
//=======================================================================//

const unsigned char bitmap_arrow [] PROGMEM = {
0x00, 0x08, 0x04, 0x7e, 0x04, 0x08, 0x00
};

enum MenuType
{
  MENU_MAIN,
  MENU_LOAD,
  MENU_BATERRY,
  MENU_SETUP,
  MENU_VOLTAGE_CALIBRATION,
  MENU_CURRENT_CALIBRATION,
  MENU_TIME,
  MENU_NULL
};

enum MenuType currMenu = MENU_LOAD;

void Menu_Main(void);
void Menu_Load(void);
void Menu_Baterry(void);
void Menu_Setup(void);
void Menu_Voltage_Calibration(void);
void Menu_Current_Calibration(void);
void Menu_Time(void);

void DoPointerNavigation(void);
void CaptureButtonDownStates(void);

//=======================================================================//
//                               VARIABLES                               //
//=======================================================================//

DateTime currentTime;
DateTime startTime;
TimeSpan ts;
float voltage = 0.0f, current = 0.0f, capacity = 0.0f, temp1 = 0.0f, temp2 = 0.0f, averageTemp = 0.0f, offVoltage = 0.0f;
uint8_t fanPercentPower = 0;
uint16_t power = 0, fanSetPwm = 0, fanCurrentPwm = 0, actualCapacity = 0;
int encPos = 1, currMenuPos = 1;
char timeBuffer[9], timeElapsedBuffer[9], voltageBuffer[8], currentBuffer[8], powerBuffer[8], capacityBuffer[10] = "    0mAh", offVoltageBuffer[8];

bool enableBtnClick = false, encBtnClick = false, enableBtnLongPress = false, encBtnLongPress = false;
bool updateValues = true, disableOutputState = true, updateMenu = true;

unsigned long updateTime = 0, fanPwmChangeMillis = 0, statusMillis = 0, updateTimeCapacity = 0;

//=======================================================================//
//                           CALIBRATION DATA                            //
//=======================================================================//

float voltageLSB = 0.003125;
float voltageOffset = 0.08;

float currentLSB = 0.003125;
float currentOffset = 0.202;

//=======================================================================//
//                                 VOID                                  //
//=======================================================================//

void GetVoltage(char* buffer);
void GetCurrent(char* buffer);
void GetPower(char* buffer);
void GetCapacity(char* buffer);
void GetOffVoltage(char* buffer);
String GetAverageTemp();
String GetTime();
String ElapsedTime();

void DisableOutput(bool en);
void FanControl(void);
void FanPWM(uint16_t power);
void LCDInit(void);
void SelfTest(void);
void SetupPins(void);
void EnableButtonClick(void);
void EncButtonClick(void);
void EnableButtonLongPress(void);
void EncButtonLongPress(void);
void ControlLoop(void);
void EncoderLoop(bool values);
void CheckLimits(void);
uint16_t mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup()
{
  Wire.begin();
  Wire.setClock(1000000);
  rtc.begin();
  SetupPins();
  DisableOutput(disableOutputState);
  LCDInit();

  //rtc.adjust(DateTime(2025, 6, 11, 19, 26, 0));
  ads.setGain(GAIN_FOUR);
  ads.begin();

  enBtn.attachClick(EnableButtonClick);
  encBtn.attachClick(EncButtonClick);
  encBtn.attachLongPressStart(EncButtonLongPress);

  buzzer.InitBuzzer(BUZZER_PIN);
}

//=======================================================================//
//                                  LOOP                                 //
//=======================================================================//

void loop()
{
  switch (currMenu)
  {
    case MENU_MAIN:
      Menu_Main();
      break;
    case MENU_LOAD:
      Menu_Load();
      break;
    case MENU_BATERRY:
      Menu_Baterry();
      break;
    case MENU_SETUP:
      Menu_Setup();
      break;
    case MENU_VOLTAGE_CALIBRATION:
      Menu_Voltage_Calibration();
      break;
    case MENU_CURRENT_CALIBRATION:
      Menu_Current_Calibration();
      break;
    case MENU_TIME:
      Menu_Time();
      break;
    default:
      break;
  }
}

//=======================================================================//
//                                  MAIN                                 //
//=======================================================================//

void Menu_Main()
{
  lcd.clear();
  updateMenu = true;

  while (1)
  {
    ControlLoop();
    EncoderLoop(false);

    if(currMenuPos != encPos)
    {
      currMenuPos = encPos;
      updateMenu = true;
    }

    if(updateMenu)
    {
      updateMenu = false;
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Electronic Load");
      lcd.setCursor(1, 1);
      lcd.print("Baterry Load");
      lcd.setCursor(1, 2);
      lcd.print("Setup");

      switch (currMenuPos)
      {
        case 1:
          lcd.setCursor(0, 0);
          lcd.write(0);
          break;
        case 2:
          lcd.setCursor(0, 1);
          lcd.write(0);
          break;
        case 3:
          lcd.setCursor(0, 2);
          lcd.write(0);
          break;
      
      default:
        break;
      }
    }

    if(encBtnClick)
    {
      encBtnClick = false;
      switch (currMenuPos)
      {
        case 1:
          currMenu = MENU_LOAD;
          return;
        case 2:
          currMenu = MENU_BATERRY;
          return;
        case 3:
          currMenu = MENU_SETUP;
          return;
      
        default:
          return;
      }
    }

    if(encBtnLongPress)
    {
      encBtnLongPress = false;
    }
  }
}

//=======================================================================//
//                            ELECTRONIC LOAD                            //
//=======================================================================//

void Menu_Load()
{
  lcd.clear();
  encoder.encoderPos = 1;

  while (1)
  {
    if(statusMillis - millis() > STATUS_INTERVAL)
    {
      CheckLimits();
      statusMillis = millis();
    }

    ControlLoop();
    FanControl();

    if(enableBtnClick)
    {
      enableBtnClick = false;
      DisableOutput(disableOutputState = !disableOutputState);
      startTime = currentTime;
    }

    if(encBtnLongPress)
    {
      encBtnLongPress = false;
      currMenu = MENU_MAIN;
      disableOutputState = true;
      updateMenu = true;
      DisableOutput(true);
      return;
    }

    if(updateValues)
    {
      GetVoltage(voltageBuffer);
      GetCurrent(currentBuffer);
      GetPower(powerBuffer);

      lcd.setCursor(0, 0);
      lcd.write(voltageBuffer);
      lcd.setCursor(0, 1);
      lcd.write(currentBuffer);
      lcd.setCursor(0, 2);
      lcd.write(powerBuffer);
      lcd.setCursor(0, 3);
      lcd.print(GetAverageTemp());

      lcd.setCursor(12, 0);
      lcd.print(GetTime());

      lcd.setCursor(12, 1);
      lcd.print(ElapsedTime());

      lcd.setCursor(17, 2);
      lcd.write(disableOutputState ? "OFF" : " ON");

      int charCount = String(fanPercentPower).length();
      lcd.setCursor(18 - charCount, 3);
      lcd.print(" " + String(fanPercentPower) + "%");

      updateValues = false;
    }

    if(millis() - updateTime >= UPDATE_VALUES)
    {
      updateValues = true;
      updateTime = millis();
    }
  }
}

//=======================================================================//
//                              BATERRY LOAD                             //
//=======================================================================//

void Menu_Baterry()
{
  lcd.clear();
  encoder.encoderPos = 0;

  while (1)
  {
    if(offVoltage != 0 && !disableOutputState && voltage <= offVoltage)
    {
      buzzer.Long();
      disableOutputState = true;
      updateValues = true;
      DisableOutput(true);
    }

    if(statusMillis - millis() > STATUS_INTERVAL)
    {
      CheckLimits();
      statusMillis = millis();
    }

    ControlLoop();
    EncoderLoop(true);
    FanControl();

    if(enableBtnClick)
    {
      enableBtnClick = false;
      DisableOutput(disableOutputState = !disableOutputState);
      startTime = currentTime;
      
      if(!disableOutputState)
        capacity = 0;
    }

    if(encBtnLongPress)
    {
      encBtnLongPress = false;
      currMenu = MENU_MAIN;
      disableOutputState = true;
      updateMenu = true;
      encoder.encoderPos = 2;
      DisableOutput(true);
      return;
    }

    if(updateValues)
    {
      GetVoltage(voltageBuffer);
      GetCurrent(currentBuffer);
      GetPower(powerBuffer);
      GetOffVoltage(offVoltageBuffer);

      lcd.setCursor(0, 0);
      lcd.print(String(voltageBuffer) + "   " + String(currentBuffer) + "  " + String(powerBuffer));
      lcd.setCursor(0, 1);
      lcd.write(capacityBuffer);
      lcd.setCursor(0, 3);
      lcd.print(GetAverageTemp());
      lcd.setCursor(10, 3);
      lcd.write(offVoltageBuffer);

      lcd.setCursor(0, 2);
      lcd.print(GetTime());

      lcd.setCursor(12, 1);
      lcd.print(ElapsedTime());

      lcd.setCursor(17, 2);
      lcd.write(disableOutputState ? "OFF" : " ON");

      int charCount = String(fanPercentPower).length();
      lcd.setCursor(18 - charCount, 3);
      lcd.print(" " + String(fanPercentPower) + "%");

      updateValues = false;
    }

    if(millis() - updateTime >= UPDATE_VALUES)
    {
      updateValues = true;
      updateTime = millis();
    }

    if(!disableOutputState && millis() - updateTimeCapacity >= UPDATE_CAPACITY)
    {
      GetCapacity(capacityBuffer);
      updateTimeCapacity = millis();
    }
  }
}

//=======================================================================//
//                                 SETUP                                 //
//=======================================================================//

void Menu_Setup()
{
  lcd.clear();
  updateMenu = true;
  encoder.encoderPos = 1;

  while (1)
  {
    ControlLoop();
    EncoderLoop(false);

    if(currMenuPos != encPos)
    {
      currMenuPos = encPos;
      updateMenu = true;
    } 

    if(updateMenu)
    {
      updateMenu = false;
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Voltage Calibration");
      lcd.setCursor(1, 1);
      lcd.print("Current Calibration");
      lcd.setCursor(1, 2);
      lcd.print("Back");

      switch (currMenuPos)
      {
        case 1:
          lcd.setCursor(0, 0);
          lcd.write(0);
          break;
        case 2:
          lcd.setCursor(0, 1);
          lcd.write(0);
          break;
        case 3:
          lcd.setCursor(0, 2);
          lcd.write(0);
          break;
      
      default:
        break;
      }
    }

    if(encBtnClick)
    {
      encBtnClick = false;
      switch (currMenuPos)
      {
        case 1:
          currMenu = MENU_VOLTAGE_CALIBRATION;
          return;
        case 2:
          currMenu = MENU_CURRENT_CALIBRATION;
          return;
        case 3:
          currMenu = MENU_MAIN;
          return;
      
        default:
          return;
      }
    }

    if(encBtnLongPress)
    {
      encBtnLongPress = false;
    }
  } 
}

//=======================================================================//
//                          VOLTAGE CALIBRATION                          //
//=======================================================================//

void Menu_Voltage_Calibration()
{
  lcd.clear();
  updateMenu = true;

  while (1)
  {
    ControlLoop();
    
    if(encBtnLongPress)
    {
      encBtnLongPress = false;
      currMenu = MENU_SETUP;
      return;
    }

    if(updateValues)
    {
      updateValues = false;

      int16_t val = ads.readADC_SingleEnded(3);
      float val1 = val * voltageLSB;
      float val2 = val1 * voltageOffset;
      float val3 = val1 - val2;
      char buffer[10];
      dtostrf(voltageLSB, 1, 6, buffer);
      lcd.setCursor(0, 0);
      lcd.print("ADC Input: " + String(val) + " ");
      lcd.setCursor(0, 1);
      lcd.print("Value: " + String(val1) + " ");
      lcd.setCursor(0, 2);
      lcd.print("LSB: " + String(buffer));
      lcd.setCursor(0, 3);
      lcd.print("Offset: " + String(voltageOffset));
    }

    if(millis() - updateTime >= 200)
    {
      updateValues = true;
      updateTime = millis();
    }
  } 
}

//=======================================================================//
//                          CURRENT CALIBRATION                          //
//=======================================================================//

void Menu_Current_Calibration()
{
  lcd.clear();
  updateMenu = true;
  DisableOutput(false);

  while (1)
  {
    ControlLoop();
    
    if(encBtnLongPress)
    {
      encBtnLongPress = false;
      currMenu = MENU_SETUP;
      DisableOutput(true);
      return;
    }

    if(updateValues)
    {
      updateValues = false;

      int16_t val = ads.readADC_SingleEnded(2);
      float val1 = (float)val / currentOffset;
      float val2 = val1 / 1000;
      char buffer[10];
      dtostrf(currentOffset, 1, 3, buffer);
      lcd.setCursor(0, 0);
      lcd.print("ADC Input: " + String(val) + " ");
      lcd.setCursor(0, 1);
      lcd.print("Value: " + String(val1) + " ");
      lcd.setCursor(0, 2);
      lcd.print("Current: " + String(val2));
      lcd.setCursor(0, 3);
      lcd.print("Offset: " + String(buffer));
    }

    if(millis() - updateTime >= 200)
    {
      updateValues = true;
      updateTime = millis();
    }
  } 
}

//=======================================================================//
//                                  TIME                                 //
//=======================================================================//

void Menu_Time()
{
  lcd.clear();

  while (1)
  {

  }
}

void GetVoltage(char* buffer)
{
  int16_t val = ads.readADC_SingleEnded(3);
  float val1 = val * voltageLSB;
  float val2 = val1 * voltageOffset;
  voltage = val1 - val2;

  if(voltage < 0)
    voltage = 0;

  dtostrf(voltage, 4, 1, buffer);
  strcat(buffer, "V");
}

void GetCurrent(char* buffer)
{
  int16_t val = ads.readADC_SingleEnded(2);
  float val1 = (float)val / currentOffset;
  current = val1 / 1000;

  if(current < 0)
    current = 0;

  dtostrf(current, 4, 1, buffer);
  strcat(buffer, "A");
}

void GetPower(char* buffer)
{
  power = voltage * current;
  dtostrf(power, 4, 0, buffer);
  strcat(buffer, "W");
}

void GetOffVoltage(char* buffer)
{
  dtostrf(offVoltage, 4, 1, buffer);
  strcat(buffer, "V");
}

String GetAverageTemp()
{
  int16_t val = ads.readADC_SingleEnded(0);
  temp1 = (val * 0.03125) / 10;

  int16_t val2 = ads.readADC_SingleEnded(1);
  temp2 = (val2 * 0.03125) / 10;

  averageTemp = (temp1 + temp2) / 2;
  return String(averageTemp).substring(0, 4) + (char)223 + "C";
}

String GetTime()
{
  currentTime = rtc.now();
  sprintf(timeBuffer, "%02d:%02d:%02d", currentTime.hour(), currentTime.minute(), currentTime.second());
  return timeBuffer;
}

String ElapsedTime()
{
  if(!disableOutputState)
    ts = currentTime - startTime;

  sprintf(timeElapsedBuffer, "%02d:%02d:%02d", ts.hours(), ts.minutes(), ts.seconds());
  return timeElapsedBuffer;
}

void GetCapacity(char* buffer)
{
  capacity += current / 3.6;
  dtostrf(capacity, 5, 0, buffer);
  strcat(buffer, "mAh");
}

void DisableOutput(bool en)
{
  digitalWrite(ENABLE_PIN, en);
  digitalWrite(LED_ENABLE, !en);
}

void FanControl()   // 0 - 65535
{
  if(temp1 < 30 && temp2 < 30)
  {
    FanPWM(0);
    fanPercentPower = 0;
    return;
  }

  if(millis() - fanPwmChangeMillis < 500)
    return;
  
  if((temp1 >= 30 || temp2 >= 30) && (temp1 < 45 || temp2 <= 45))
  {
    fanSetPwm = mapFloat(averageTemp, 30.0, 45.0, 5000.0, 60000.0);
    fanPercentPower = map(fanSetPwm, 5000, 60000, 0, 99);
  }
  else if(temp1 >= 45 || temp2 >= 45)
  {
    fanSetPwm = 65535;
    fanPercentPower = 100;
  }

  if(fanCurrentPwm != fanSetPwm)
  {
    FanPWM(fanSetPwm);
    fanPwmChangeMillis = millis();
  }
}

uint16_t mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void FanPWM(uint16_t fanPower)   // 0 - 65535
{
  analogWrite(FAN_PWM, fanPower);
  fanCurrentPwm = fanPower;
}

void EnableButtonClick()
{
  enableBtnClick = true;
  buzzer.Short();
}

void EncButtonClick()
{
  encBtnClick = true;
  buzzer.Short();
}

void EnableButtonLongPress()
{
  enableBtnLongPress = true;
  buzzer.Long();
}

void EncButtonLongPress()
{
  encBtnLongPress = true;
  buzzer.Long();
}

void LCDInit()
{
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.createChar(0, bitmap_arrow);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("  Electronic Load   ");
  lcd.setCursor(0, 2);
  lcd.print("        v1.5        ");
  delay(500);
}

void EncoderLoop(bool values)
{
  long newPos = encoder.encoderPos;
  if(newPos != encPos)
  {
    if(!values)
    {
      if(newPos < 1)
      {
        newPos = 3;
        encoder.encoderPos = 3;
      }

      if(newPos > 3)
      {
        newPos = 1;
        encoder.encoderPos = 1;
      }
    }
    else
    {
      if(newPos <= 0)
      {
        newPos = 0;
        encoder.encoderPos = 0;
      }

      offVoltage = (float)newPos * 0.1;
    }

    buzzer.Short();
    encPos = newPos;
  }
}

void SetupPins()
{
  pinMode(LED_FAULT, OUTPUT);
  pinMode(LED_ENABLE, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(REFERENCE_PIN, INPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(FAN_FB, INPUT);

  digitalWrite(LED_FAULT, LOW);
  digitalWrite(LED_ENABLE, LOW);
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWriteResolution(16);
  FanPWM(0);
}

void ControlLoop()
{
  enBtn.tick();
  encBtn.tick();
  buzzer.Tick();
}

void CheckLimits()
{
  if(voltage > VOLTAGE_MAX_LIMIT)
  {
    currentStatus = VOLTAGE_LIMIT;
    lcd.clear();
  }
  else if(current > CURRENT_MAX_LIMIT)
  {
    currentStatus = CURRENT_LIMIT;
    lcd.clear();
  }
  else if(power > POWER_MAX_LIMIT)
  {
    currentStatus = POWER_LIMIT;
    lcd.clear();
  }
  else if(averageTemp > TEMPERATURE_MAX_LIMIT)
  {
    currentStatus = TEMPERATURE_LIMIT;
    lcd.clear();
  }

  while (currentStatus != NORMAL_STATUS)
  {
    currMenu = MENU_NULL;
    encBtn.tick();
    digitalWrite(LED_FAULT, 1);
    lcd.setCursor(0, 1);

    if(encBtnLongPress)
    {
      encBtnLongPress = false;
      currentStatus = NORMAL_STATUS;
      digitalWrite(LED_FAULT, 0);
      digitalWrite(BUZZER_PIN, 0);
      currMenu = MENU_MAIN;
      lcd.clear();
      return;
    }

    switch (currentStatus)
    {
      case FAULT:
        break;
      case  VOLTAGE_LIMIT:
        lcd.print("INPUT VOLTAGE LIMIT!");
        digitalWrite(BUZZER_PIN, 1);
        DisableOutput(true);
        disableOutputState = true;
        break;
      case  CURRENT_LIMIT:
        lcd.print("INPUT CURRENT LIMIT!");
        digitalWrite(BUZZER_PIN, 1);
        DisableOutput(true);
        disableOutputState = true;
        break;
      case  POWER_LIMIT:
        lcd.print("INPUT POWER LIMIT! ");
        digitalWrite(BUZZER_PIN, 1);
        DisableOutput(true);
        disableOutputState = true;
        break;
      case  TEMPERATURE_LIMIT:
        lcd.print("INPUT TEMPERATURE! ");
        digitalWrite(BUZZER_PIN, 1);
        DisableOutput(true);
        disableOutputState = true;
        analogWrite(FAN_PWM, 255);
        break;
      case  SELF_TEST_ERROR:
        break;  
    }
  }
}