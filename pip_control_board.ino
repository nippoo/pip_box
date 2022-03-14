#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#define TARGET_TEMP_C 21

// Pin definitions
#define ONE_WIRE_BUS 2
#define PELTIER_OUT_PIN 3
#define FLIPPER_ENABLE_PIN 4
#define FLIPPER_OUT_PIN 5
#define LED_STATUS_PIN 8
#define LED_WARNING_PIN 9

// Setup a oneWire instance to communicate with temperature sensor
OneWire oneWire(ONE_WIRE_BUS);  
DallasTemperature sensors(&oneWire);

// PID variables for Peltier
double TargetTemp, CurrTemp, PeltierPWM;

// PID parameters for Peltier (P = 5, I = 5, D = 1 as a guess)
PID myPID(&CurrTemp, &PeltierPWM, &TargetTemp,5,5,1, REVERSE);

void setup(void)
{
  sensors.begin();  // Start up the library
  Serial.begin(9600);
  sensors.requestTemperatures();
  TargetTemp = TARGET_TEMP_C;
  CurrTemp = sensors.getTempCByIndex(0);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2000);
  
  pinMode(PELTIER_OUT_PIN, OUTPUT);
  pinMode(FLIPPER_ENABLE_PIN, INPUT);
  pinMode(FLIPPER_OUT_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_WARNING_PIN, OUTPUT);

  digitalWrite(FLIPPER_OUT_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_WARNING_PIN, LOW);

  cli();//stop interrupts

//set timer1 interrupt at 100Hz for flipper
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1550;// = (16*10^6) / (10*1024) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts

}

ISR(TIMER1_COMPA_vect){ // flipper code, runs at 100Hz
    if(digitalRead(FLIPPER_ENABLE_PIN)){
      int flipperState = random(0,2);
      digitalWrite(FLIPPER_OUT_PIN, flipperState);
      digitalWrite(LED_STATUS_PIN, flipperState);
    }  
   else{
    digitalWrite(FLIPPER_OUT_PIN,LOW);
    digitalWrite(LED_STATUS_PIN, LOW);
    }
}
  

void loop(void) // update Peltier PWM value and serial print every 2s
{ 

//  Serial.print("Rail 5 voltage: ");
//  Serial.print(analogRead(4)*0.03086392334);
//  Serial.println("V");
//  Serial.print("Rail 6 voltage: ");
//  Serial.print(analogRead(5)*0.03086392334);
//  Serial.println("V");

  sensors.requestTemperatures();

  //print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.println("C");

  //update Peltier status
  
  CurrTemp = sensors.getTempCByIndex(0);
  myPID.Compute();
  
  analogWrite(PELTIER_OUT_PIN,PeltierPWM);
  Serial.print("PWM output: ");
  Serial.println(PeltierPWM);

  delay(2000);
}
