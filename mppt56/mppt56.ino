#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define pins and constants
const int VIN = A0;             // Voltage input pin
const int ANALOG_IN_PIN = A1;   // Current sensor pin
const int Relay_pin = 8;        // Relay pin
const int pwmpin = 9;           // PWM output pin

// Voltage measurement constants
const float VCC = 5.04;         // Supply voltage
const float R1 = 30000.0;       // Resistor R1 (in ohms)
const float R2 = 7500.0;        // Resistor R2 (in ohms)
const float ref_voltage = 5.0;  // Reference voltage
float zerocal = 1.10;           // Zero calibration value

// Current measurement variables
float current;

// MPPT variables
float Power_now = 0, Power_anc = 0, voltage_anc = 0;
float pwm = 128;
float Dmax = 0.95;
float Dmin = 0.0;
float Dinit = 0.95;
float DelD = 0.0001;
float Vold = 0;
float Pold = 0;
float Dold = Dinit;

// ADC values and calculated values
int adc_value = 0;
float adc_voltage = 0.0;
float in_voltage = 0.0;

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize pins
  pinMode(Relay_pin, OUTPUT);
  pinMode(pwmpin, OUTPUT);

  // Initialize LCD
  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();

  // Set PWM frequency to 25 kHz
  setPwmFrequency25kHz();
}

void loop() {
  // Voltage measurement
  adc_value = analogRead(VIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage * (R1 + R2) / R2 - zerocal;

  // Current measurement
  long rawCurrent = 0;
  for (int i = 0; i < 200; i++) {
    int c = analogRead(ANALOG_IN_PIN);
    rawCurrent += c;
    delay(1);
  }
  rawCurrent = (rawCurrent / 200 - 514);  // Offset correction
  if (rawCurrent < 1) rawCurrent = 0;
  current = rawCurrent * 7.0 / 1000.0; // Convert to Amps with calibration factor

  // Calculate power
  Power_now = in_voltage * current;

  // MPPT P&O Algorithm
  float P = Power_now;
  float dV = in_voltage - Vold;
  float dP = P - Pold;

  if (dP != 0) {
    if (dP < 0) {
      if (dV < 0) {
        Dold -= DelD;  // Decrease duty cycle
      } else {
        Dold += DelD;  // Increase duty cycle
      }
    } else {
      if (dV < 0) {
        Dold += DelD;  // Increase duty cycle
      } else {
        Dold -= DelD;  // Decrease duty cycle
      }
    }
  }

  // Bound the duty cycle between Dmin and Dmax
  if (Dold >= Dmax) Dold = Dmax;
  if (Dold <= Dmin) Dold = Dmin;

  // Set PWM output for the buck converter
  analogWrite(pwmpin, Dold * 255);

  // Update old values for next iteration
  Vold = in_voltage;
  Pold = P;

  // Control relay for low voltage condition
  if (in_voltage > 12.58) {
    digitalWrite(Relay_pin, LOW);
  } else {
    digitalWrite(Relay_pin, HIGH);
  }

  // Display values on LCD
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(in_voltage, 2);
  lcd.print("V P: ");
  lcd.print(Power_now, 2);

  lcd.setCursor(0, 1);
  lcd.print("I: ");
  lcd.print(current, 3);
  lcd.print("A PWM: ");
  lcd.print(Dold * 100, 1);
  lcd.print("%");

  // Delay for the next iteration
  delay(1500);
}

void setPwmFrequency25kHz() {
  // Configure Timer1 for 25 kHz PWM
  TCCR1A = 0;                   // Clear Timer1 control register A
  TCCR1B = 0;                   // Clear Timer1 control register B
  TCNT1 = 0;                    // Reset Timer1 counter

  // Set Timer1 to Phase Correct PWM mode with ICR1 as top
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);

  ICR1 = 320;                   // Set top value for 25 kHz (16 MHz / (2 * 25 kHz))
  OCR1A = 160;                  // Set compare match value for 50% duty cycle (adjust as needed)

  // Enable PWM output on pin 9 (OC1A)
  TCCR1A |= (1 << COM1A1);
}
