/* Project
                            --------
D5(A0)/RESET/ADC0/PB5 -- 1  /      / 8 -------   Vcc   ---------
D3(A3)/XTAL1/ADC3/PB3 -- 2  /      / 7 -- PB2/SCL/SCK/------- D2(A1)
D4(A2)/XTAL2/ADC2/PB4 -- 3  /      / 6 -- PB1/MISO/---------- D1
-----   GND   ---------- 4  /      / 5 -- PB0/MOSI/SDA/AREF/- D0 
                             --------
                             
Pin Pin Name    I/O Type  Description
1   PB5 (OC1A)  Output    PWM output channel A
2   PB3 (OC2A)  Output    PWM output channel B
3   PB4         Input     General-purpose digital I/O pin
4   GND –       Ground
5   PB0 (MOSI)  In/Out    SPI master-out, slave-in or general-purpose digital I/O pin
6   PB1 (MISO)  In/Out    SPI master-in, slave-out or general-purpose digital I/O pin
7   PB2 (SCK)   In/Out    SPI clock or general-purpose digital I/O pin
8   VCC –       Positive supply voltage

upload example 11.ArduinoISP
then select tools->board-> attiny85
programmer Arduino as ISP
*/

#include <avr/sleep.h>  //Needed for sleep_mode
#include <avr/wdt.h>   //Needed to enable/disable watch dog timer
#include <Arduino.h>
#include <U8x8lib.h>

uint16_t adc_value;
int Motor_pin        = 0;  // Pin 5 (motor)
int watchdog_counter = 0;  // add +1 at each awake of the processor..
int counterA         = 0;  
long result          = 0;

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock */ 1, /* data */ 2, /* reset */ U8X8_PIN_NONE);
// oled screen  - SDA on PB1 (pin 6) - SCL on PB2 (pin 7)

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
   watchdog_counter++;
  //Don't do anything. This is just here so that we wake up.
}

void setup() {

  // ADC Left Shift Result
     ADMUX |= 1<<ADLAR;
  // ADC Voltage Referrence
  // ADMUX |= 0<<REFS2|0<<REFS1|0<<REFS0;           // Vcc  is used as Voltage Referrence
     ADMUX |= 1<<REFS2|1<<REFS1|0<<REFS0;          // 2.56 is used as Voltage Referrence
  // ADC Input Channel/s
     ADMUX |= 0<<MUX3|0<<MUX2|1<<MUX1|0<<MUX0;   // Use ADC3 on pin PB3 as ADC input (0011)
  // ADC clock prescaler
     ADCSRA |= 0<<ADPS2|1<<ADPS1|1<<ADPS0;     // Use a divide by 8 Prescaler.
                                              // Sysclock = 8MHz, ADCclk = 1MHz (max)
     ADCSRA |= 1<<ADEN;                      // Enable the ADC

// Oled Screen setup
  u8x8.begin();                                  //
  u8x8.setPowerSave(0);                         // oled on
  u8x8.setFont(u8x8_font_chroma48medium8_r);   //
  u8x8.setCursor(1, 1);                       //
  u8x8.print("OriginalStefrid");             //
  u8x8.setPowerSave(1);                     // shut down the oled
       
  pinMode(PWM_pin, OUTPUT);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   //Power down everything, wake up from WDT
  sleep_enable();
}

void loop() 
  {
    ADCSRA &= ~(1<<ADEN); //Disable ADC, saves ~230uA
    
    goToSleep(9); // sleep for 8 seconds
    
    //setup_watchdog(9);   //Sets the watchdog timer to wake us up, but not reset
                          //0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
                         //6=1sec, 7=2sec, 8=4sec, 9=8sec
    //sleep_mode();    //Go to sleep! Wake up xx sec latter
    
    // wake up !
    
    if (watchdog_counter > 6)  { // > 9 (= 5) so 10 x 9 sec about 90 secondes
      watchdog_counter = 0;
      ADCSRA |= 1<<ADEN;                 // Enable the ADC
      ADCSRA |= 1<<ADSC;                // Start the ADC conversion
      while(ADCSRA &(1<<ADSC) == 1);   // wait until conversion is finished.
                                      // a 0 on ADCSRA means conversion is complete
  //  adc_value = ADCL|(ADCH << 8);  // ADC value is left justified. Use operand precedence rules
                                    // left to right precedence, so ADCL is read first
      adc_value = ADCH;

      u8x8.setPowerSave(0); // awake the oled
      //u8x8.setFont(u8x8_font_chroma48medium8_r);
      //u8x8.setFont(u8g2_font_u8glib_4_tr);
      u8x8.setCursor(1, 3);
      u8x8.print("Start > 100");
        u8x8.setCursor(1, 4);
        u8x8.print("Adc:     ");
        u8x8.setCursor(1, 4);
        u8x8.print("Adc: ");
        u8x8.print(adc_value);
      u8x8.setCursor(1, 5);
      u8x8.print("Vcc:     ");
      u8x8.setCursor(1, 5);
      u8x8.print("Vcc: ");
      u8x8.print(result);

   // simple counter for testing the loop
      u8x8.setCursor(1, 7);
      u8x8.print("loop #");
      u8x8.print(counterA);
      u8x8.setPowerSave(1);
    //u8x8.drawString(11, 7, String(counterB).c_str());
      counterA++;
      result++;
      
        if (adc_value > 120)  // after ADC reading, test if the value is above, if so, run the motor
         {
          for (int i = 0; i<10; i++)  // from #IG benoit - this act like a pwm. 
          {
            digitalWrite(Motor_pin, HIGH);
            goToSleep(0);              // 16ms                                      
            digitalWrite(Motor_pin, LOW);
            goToSleep(2);             //  64ms
          }
          
          watchdog_counter = 0;  // the "goToSleep" awake the prcessor ten time (loop for i)
                                // so need the set counter at 0
                               //this counter is trigged at each awake !
         }
      }
  }       // end of loop



// ***** ***** ***** ***** ***** ****** ****** ***** *****
void goToSleep(int tim) {   // IG @BenoitPaquinDk
                             // disable ADC, Sleep, enable ADC
  //ADCSRA &= ~(1<<ADEN);   //Disable ADC, saves ~230uA
  sleep_enable();
  setup_watchdog(tim);
  sleep_mode();
  //ADCSRA |= 1<<ADEN;      // Enable the ADC
  }


// ***** ***** ***** ***** ***** ****** ****** ***** *****
//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/

void setup_watchdog(int timerPrescaler) {
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings
  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
                                       //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF);                //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE);     //Set WD_change enable, set WD enable
  WDTCR = bb;                       //Set new watchdog timeout value
  WDTCR |= _BV(WDIE);              //Set the interrupt enable, this will keep unit from resetting after each int
}
//
