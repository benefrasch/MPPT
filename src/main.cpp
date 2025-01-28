#include <Arduino.h>
#include <Adafruit_ADS1x15.h>

Adafruit_ADS1115 ads;

int16_t lastPwmValue = 0;
int16_t currentPwmValue = 20;

const uint16_t PWM_RANGE = 1000;
const uint8_t PWM_STEP = 6;
const float voltage_divider = 0.0448901624;

float lastOutVoltage = 0.0f;
float currentOutVoltage = 0.0f;

float currentInVoltage = 0.0f;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting MPPT");


  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  // pwm on pin D6
  pinMode(D6, OUTPUT);
  analogWriteFreq(8000);
  analogWriteRange(PWM_RANGE);
  analogWrite(D6, currentPwmValue);
}

void loop()
{

  // read voltage on out pin
  lastOutVoltage = currentOutVoltage;
  
  uint16_t adc0 = ads.readADC_SingleEnded(0);
  currentOutVoltage = ads.computeVolts(adc0) / voltage_divider;
  
  uint16_t adc1 = ads.readADC_SingleEnded(1);
  currentInVoltage = ads.computeVolts(adc1) / voltage_divider;

  float diff = currentOutVoltage - lastOutVoltage;


  Serial.print(">pwmValue:"); Serial.println(currentPwmValue); 
  Serial.print(">outVoltage:"); Serial.println(currentOutVoltage); 
  Serial.print(">inVoltage:"); Serial.println(currentInVoltage);
  Serial.print(">outputPower(appr):"); Serial.println(currentOutVoltage * currentOutVoltage/30);
  Serial.print(">diff:"); Serial.println(diff);
  
  // determine direction for new pwm value
  bool pptIncreased = currentOutVoltage > lastOutVoltage;
  bool pwmIncreased = currentPwmValue > lastPwmValue;

  // Serial.print(">pptincreased:"); Serial.println(pptIncreased); 
  // Serial.print(">pwmincreased:"); Serial.println(pwmIncreased); 

  uint8_t pwmSave = currentPwmValue;
  if (pptIncreased)
  {
    // continue in the same direction
    if (pwmIncreased)
      currentPwmValue += PWM_STEP;
    else
      currentPwmValue -= PWM_STEP;
  }
  else
  {
    // go back
    if (pwmIncreased)
      currentPwmValue -= PWM_STEP;
    else
      currentPwmValue += PWM_STEP;
  }

  
  // limit
  if (currentPwmValue > 900)
    currentPwmValue = 900;
  if (currentPwmValue < 1)
    currentPwmValue = 1;
  if(currentInVoltage < 0.1)
    currentPwmValue = 2;
  lastPwmValue = pwmSave;

  // set new pwm value and wait
  analogWrite(D6, currentPwmValue);
  delay(1000);
  

}