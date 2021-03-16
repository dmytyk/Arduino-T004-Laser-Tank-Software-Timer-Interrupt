# Arduino-T004-Laser-Tank-Software-Timer-Interrupt
This is tutorial number 4 for my 5.5W Laser Tank (LT) project.  It demonstrates how to setup and configure an MKR1010 hardware timer to create an Interrupt Service Routine. The Hardware Timers, using Interrupt, still work even if other functions are blocking. Moreover, they are much more precise (certainly depending on clock frequency accuracy) than other software timers using millis() or micros(). That's mandatory if you need to measure some data requiring better accuracy.
The most important feature is they're ISR-based timers. Therefore, their executions are not blocked by bad-behaving functions / tasks.

### What are we Shooting For
I needed a 50uSec (microseconds) every 5mSec (milliseconds) Pulse Width Modulation signal to simulate what the controller for the Laser Module was generating.  

### What do need to do

We need to identify two parameters and then configure the timer.  To identify the two parameters we can do some easy math:

- The Math
    - (Clock Source / Pre-Scaler) / Target Frequency = Counter Value
      - Start with the MKR1010's clock source of 48MHz which is a 0.020833333333333uSec period
      - Using a Pre-Scaler of 16 we get 48MHz / 16 = 3MHz which is a 0.33333333333333uSec period
      - We need a 50uSec pulse which is a 20kHz frequency so 3MHz / 20kHz = 150 for our counter value 
      - ((48,000,000 / 16) / 20,000) = 150

This is the LT with the laser installed, and the test setup used for this tutorial. 

![Test Setup](/Images/TestSetup.JPG)

Here is the code for the MKR1010.  This is a subset of the code for my LT, it has been modified a bit for this tutorial.

[Arduino Code](/T004_LaserTankTimerinterrupt.ino)

- Pin Assignments
    - Assign the pins we are using to show the different timing intervals created in the Interrupt Service Routine (ISR).
```sh
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
```
- Constants
    - Create the constants we will need to count the correct number of interrupts for the defined times we are looking for.  For example as noted above we have a 50uSec interrupt so counting 20 interrupts would be .00005 * 20 or .001 or 1 millisecond.
```sh
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
```
- Vars
    - Create the ISR vars (Note: volatile declaration - directs the compiler to load the variable from RAM and not from a storage register)
```sh 
    // ISR vars
    volatile int ledState = LOW;            // ledState used to toggle the state of the built in LED
    volatile int ISR_DelayCounter = 0;      // counter used by out version of delay to indicate 1 millisecond has passed  
    volatile int ISR_Builtin = 0;           // counter used to tell us when it is time to toggle the built in LED, used to show the ISR is running 
    volatile int ISR_Green = 0;             // counter used to toggle the green led in the background second
    volatile int ISR_Yellow = 0;            // counter used to toggle the yellow led in the background 5 seconds
    volatile int ISR_Red = 0;               // counter used to toggle the red led in the background every 10 seconds
```
- Setup Timer 4 to trigger the TC4_Handler() ISR routine
    - Setup timer 4 to count 50uSec interrupts - Select Generic Clock (GCLK) 4, set it to 50/50 duty cycle, connect GCLK to Timer Counter 4 (TC4), Set it to 8 bit mode since we only need to count 150 (less the 255 max count for 8 bits), set the predetermined count to 150 to get the 50 uSec period, set it to the highest priority, connect TC4 to Nested Vector Interrupt Controller, set the pre-scalar to 16 to get 3 MHz and finally enable the counter.
```sh 
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
```
- Build the ISR routine
    - This code is the actual ISR, this is where come every 50 uSec.  First make sure we have a valid interrupt (i.e. the correct interval has occurred), if our delay counter is set process it, flash the builtin LED every 100 milliseconds to show we are coming here, process our 3 background timers, the clear the interrupt, so we can start again
```sh 
// Interrupt Service Routine (ISR)
void TC4_Handler()
{
  // check for overflow (OVF) interrupt
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)
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

    // clear interrupt - clear the MC1 interrupt flag so we can start the next cycle
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;
  }
}
```
- Setup
    - Setup our hardware pins, then call setup_timer4() to configure the timer interrupt to occur every 50 uSecs.
```sh 
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
```
- Loop
    - Process the three background processes (flash the different color LED's) when the individual timed events occur, basically once the correct number of interrupts are counted for each background task, we reset the count and process the task.
```sh 
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
```
## Next Up
> - A bottle rocket launcher attachment for a Drone
> - A Paratrooper dropping device an RC airplane / Drone
> - A cool GEO cache device (my grandkids love to find them)
> - Several cool Arduino tutorials for Background Processing, Board to Board communication and Fun stuff TBD

Please follow me on [YouTube](https://www.youtube.com/channel/UClwcP7ByE6Ia9DmKfP0C-UQ) to catch the "Next Up Stuff" and thanks for Hanging out at the Shack!


