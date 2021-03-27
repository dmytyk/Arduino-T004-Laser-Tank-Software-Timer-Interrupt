// PIN ASSIGNMENTS
#ifndef GREEN
    // Green LED - 1 Sec
    #define GREEN 0
#endif
#ifndef YELLOW
    // Yellow LED - 5 Sec
    #define YELLOW 1
#endif
#ifndef RED
    // Green LED - 10 Sec
    #define RED 2
#endif

// ISR
// call ISR - TC4_Handler 20000 times per second
// an interrupt is called every 50 microseconds so to get:
// count 1 interrupt = 50us
// count 20 interrupts = 1ms
// count 2000 interrupts = 100ms
// count 20000 interrupts = 1s
// count 100000 interrupts = 5s
// count 200000 interrupts = 10s
#ifndef ISR_1MSECS
    // 1 millisecond
    #define ISR_1MSECS 20
#endif
#ifndef ISR_100MSECS
    // 100 millisecond
    #define ISR_100MSECS 2000
#endif
#ifndef ISR_1SECS
    // 1 Second
    #define ISR_1SECS 20000
#endif
#ifndef ISR_5SECS
    // 5 Seconds
    #define ISR_5SECS 100000
#endif
#ifndef ISR_10SECS
    // 10 Seconds
    #define ISR_10SECS 200000
#endif

// ISR vars
volatile int ledState = LOW;            // ledState used to toggle the state of the built in LED
volatile int ISR_DelayCounter = 0;      // counter used by out version of delay to indicate 1 millisecond has passed
volatile int ISR_Builtin = 0;           // counter used to tell us when it is time to toggle the built in LED, used to show the ISR is running
volatile int ISR_Green = 0;             // counter used to toggle the green led in the background second
volatile int ISR_Yellow = 0;            // counter used to toggle the yellow led in the background 5 seconds
volatile int ISR_Red = 0;               // counter used to toggle the red led in the background every 10 seconds

// Start MKR1010 software timer interrupt function **********
void setup_timer4()
{
    // Set up the generic clock (GCLK4) used to clock timers
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);              // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |           // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |     // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);            // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Feed GCLK4 to TC4 and TC5
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |       // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;       // Feed the GCLK4 to TC4 and TC5
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;          // Set the counter to 8-bit mode
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

    REG_TC4_COUNT8_CC0 = 150;                       // Set the TC4 CC0 value calculated for 50usec = 20Khz
                                                    // 3Mhz / 20Khz = 150, (3,000,000 / 20,000 = 150)                                                 
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

    NVIC_SetPriority(TC4_IRQn, 0);                  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
    NVIC_EnableIRQ(TC4_IRQn);                       // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

    REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
    REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts

    // value needed for 50usec ISR
    uint16_t prescale=TC_CTRLA_PRESCALER(4);        // 48Mhz (main clock) / 16 (prescaler) = 3Mhz
    Serial.println("prescale " + (String)prescale);

    REG_TC4_CTRLA |= prescale | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;  // Enable TC4
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);                              // Wait for synchronization
}

// Interrupt Service Routine (ISR)
void TC4_Handler()
{
  // check for overflow (OVF) interrupt
  if (TC4->COUNT8.INTFLAG.bit.OVF && TC4->COUNT8.INTENSET.bit.OVF)
  {
    // this is our delay counter
    if(ISR_DelayCounter != 0) {
      ISR_DelayCounter--;
    }
    
    // Flash the Built in LED
    // see if it's time to change the built in led
    if(ISR_Builtin == ISR_100MSECS) {
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
  
      // set the LED with the ledState of the variable:
      digitalWrite(LED_BUILTIN, ledState); 
      ISR_Builtin = 0; 
    } else {
      ISR_Builtin++;  
    }
    
    // This is the ISR Code
    // see if it is time to change the green led - every 1 second(s)
    // if so tell the background to do it
    if(ISR_Green < ISR_1SECS) {
      ISR_Green++;
    }
    
    // see if it is time to change the yellow led - every 3 second(s)
    // if so tell the background to do it
    if(ISR_Yellow < ISR_5SECS) {
      ISR_Yellow++;
    }
    
    // see if it is time to change the red led - every 5 second(s)
    // if so tell the background to do it
    if(ISR_Red < ISR_10SECS) {
      ISR_Red++;
    }

    // clear interrupt - clear the MC1 interrupt flag
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;
  }
}
// End MKR1010 software timer interrupt function **********

// this is our version of the Arduino delay function
void ISRDelay(int delayTime)
{
  ISR_DelayCounter = delayTime * ISR_1MSECS; 
  while(ISR_DelayCounter != 0); 
}

void setup()
{
    // we use this led to show the background is running
    // flashed on and off every time we go through the background loop
    pinMode(LED_BUILTIN, OUTPUT);
    
    pinMode(GREEN, OUTPUT);
    pinMode(YELLOW, OUTPUT);
    pinMode(RED, OUTPUT);
    
    // done initialization so start the interrupts
    // call ISR - TC4_Handler 20000 times per second
    // an interrupt is called every 50 microseconds
    setup_timer4();
}

void loop()
{
    // Background Process 1
    // see if it's time to change the green led
    if(ISR_Green == ISR_1SECS) {
      ISR_Green = 0; 
      digitalWrite(GREEN, HIGH);  
      ISRDelay(50);
      digitalWrite(GREEN, LOW); 
    }

    // Background Process 2
    // see if it's time to change the yellow led
    if(ISR_Yellow == ISR_5SECS) {
      ISR_Yellow = 0; 
      digitalWrite(YELLOW, HIGH);  
      ISRDelay(50);
      digitalWrite(YELLOW, LOW); 
    }

    // Background Process 3
    // see if it's time to change the red led
    if(ISR_Red == ISR_10SECS) {
      ISR_Red = 0; 
      digitalWrite(RED, HIGH);  
      ISRDelay(50);
      digitalWrite(RED, LOW); 
    }
}
