#include <avr/io.h>
#include <util/delay.h>
#include "Arduino.h"
#include <avr/interrupt.h>
/*
MCU --> ATMega328P on a arduinonano
Peripheral --> KY-028 Digital Temperature Sensor Module (specifically using the A0 analog signal pin)


we are using portC, the analog read in on PC5 
1. We need to enable the ADEN bit within the ADCSRA register
2. Once the ADC has calculated a value, it is then stored within the ADCH and ADCL registers. That is Analog-to-Digital-Convertor-HIGH/LOW. The data is presetned 
"right adjusted".
3. When we access the data within ADCH, this will then re-enable access to ADCH and ADCL from the ADC.
4. We need to start the conversion by disabling the power reduction ADC bit, PRADC, by writing a 0 and wirting a 1 to the start conversion bit, ADSC. 
5. (OPTIONAL) we can automatically trigger a conversion by setting the auto triggering but, ADATE, within ADCSRA. For this we will also need to specify the trigger
and we do this by setting the ADC trigger select bits, ADTS, in ADCSRB. 
6. When we trigger a snap of the input signal for convserion, ADIF flag will be raised. If we put the pin on auto trigger mode, then we also need to write a logical 1 
to ADSC in ADCSRA. 

We need to manually write to the ADSC in ADCSRA a 1 to enable the auto triggering. This will continue on its own in free-running mode. 


*/

int reg1 = 0x79;
int reg2 = 0x78;
short offset = 0.035;

void setupADC();
void startConversion();
float grabAns();


void startConversion(){
  ADCSRA |= _BV(ADSC);
}

void setupADC(){
  ADMUX = 0b00000101;
  ADCSRA = _BV(ADEN) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2) | _BV(ADATE);
  ADCSRB = 0x00;
  DIDR0 |= (1 << ADC5D);


  startConversion();

}

float grabAns(){
  float avg = 0.0;
  for(int i=0; i < 5; i++)
  { 
  
    uint8_t low  = ADCL;
    uint8_t high = ADCH;


    uint16_t adc = (high << 8) | low;		// 0<= result <=1023
    float adcVoltage = ((5.0 * adc) / 1024.0);
    avg += adcVoltage;
  }
 
  return avg / 5.0;

}

void setup(){
  DDRC &= ~(1 << DDC5);
  Serial.begin(115200);
}

int main(void){
  setup();
  setupADC();
  while(1){
    float ans = grabAns();
    Serial.print("Measured voltage: "); //cheating a bit here. This is the packaging and sending of the information over to the computer.
    Serial.println(ans - 0.09);
    _delay_ms(250);
    
  }

}