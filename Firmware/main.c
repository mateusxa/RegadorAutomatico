/*
 * File:   main.c
 * Author: Mateus
 *
 * Created on May 26, 2021, 4:00 PM
 */

// watering tome nao passa de 40 s
// wake up time nao funciona quando muda a hora
// watering time nao funciona quando demora um tempo para acionar


#include "xc8.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>



#define WaterPumpTRIS   TRISCbits
#define WaterPumpPORT   PORTCbits
#define WaterPumpPIN    RC2

#define BuzzerTRIS      TRISCbits
#define BuzzerPORT      PORTCbits
#define BuzzerPIN       RC0

#define ValveTRIS       TRISCbits
#define ValvePORT       PORTCbits
#define ValvePIN        RC5

#define BUTTON_DELAY    50


// MODES
#define SETTING_CURRENT_MINUTES             0
#define SETTING_CURRENT_HOURS               1
#define IDLE_MODE                           2
#define MAIN_LOOP_HANDLER                   3
#define SETTING_TARGET_MINUTES              4
#define SETTING_TARGET_HOURS                5
#define SETTING_WATERING_SECONDS            6
#define SETTING_WATERING_MINUTES            7
#define SETTING_VALVE_SECONDS               8
#define SETTING_VALVE_MINUTES               9
#define WATERING_MODE                       10
#define VALVE_MODE                          11

void ChangeTime(uint8_t Address, uint8_t Time);
uint8_t ReadTime(uint8_t Address);
char AddBCD(char Number1, char Number2);
char SubBCD(char Number1, char Number2);
void WakeUp(void);
void UpdateCurrentTime(void);
void Buzz(void);
void BlinkDigit(char index);
void StartPWM(void);

void Display(char Hours, char Minutes);

void GPIOconfig(void);
void I2Cconfig(void);
void TIMERconfig(void);
void INTconfig(void);
void PWMconfig(void);
void MCUconfig(void);
void MCUinit(void);
void TMR0interruption (void);
void TMR1interruption (void);
void TMR3interruption (void);
void INT0interruption (void);
void INT1interruption (void);
void INT2interruption (void);

char CurrentTime[3];
char TargetTime[3] = {0x00, 0x00, 0x12};

char Blink_First_Digit = 0;
char Blink_Second_Digit = 0;
char Blink = 0;

char State = 0;

char UpdateTimeFlag = 0;
char WakeUpTime[3] = {0,0,0};

char WaterEnable = 0;
char WaterEnableLock = 0;
char WateringTime[3] = {0,0,0};
char CurrentWateringTime[2] = {0,0};

char ValveEnable = 0;
char ValveEnableLock = 0;
char ValveTime[3] = {0,0,0};
char CurrentValveTime[2] = {0,0};

char UpdateTimePrescale = 0;

char BuzzerFlag = 0;

char LastSecond = 0;


void main(void) {
    
    MCUconfig();
    MCUinit();
    
    while(1){
        
        __delay_ms(1);                                     // Time to not overflow
        
        switch(State){
            case SETTING_CURRENT_MINUTES:                                         // Setting Hours mode
                
                Display(CurrentTime[2], CurrentTime[1]);    // Display Current time
                BlinkDigit(1);
                
                break;
            
            case SETTING_CURRENT_HOURS:                                         // Setting Minutes mode
                
                Display(CurrentTime[2], CurrentTime[1]);    // Display Minutes
                BlinkDigit(2);
                
                break;
                
            case IDLE_MODE:                                         // Sleep mode
                
                Sleep();
                
                break;
                
            case MAIN_LOOP_HANDLER:                                         // main loop mode
                
                if(UpdateTimeFlag){
                   
                    UpdateCurrentTime();   
                    
                    if(CurrentTime[2] == TargetTime[2] && CurrentTime[1] == TargetTime[1]) {    // 2 - Hours | 1 - Minutes
                        WaterEnable = 1;                                                        // Enable
                    }else{
                        WaterEnable = 0;                                                        // Disable
                        WaterEnableLock = 0;                                                    // Disable
                    }

                    UpdateTimeFlag = 0;                                                         // Disable
                }

                Display(CurrentTime[2], CurrentTime[1]);                                        // 2 - Hours | 1 - Minutes
                
                BlinkDigit(0);
                
                if(WaterEnable == 1 && WaterEnableLock == 0) State = WATERING_MODE;                        // 
                else if(WakeUpTime[2] == CurrentTime[2] && WakeUpTime[1] == CurrentTime[1]) State = IDLE_MODE;
                
                break;
                
            case SETTING_TARGET_MINUTES:                                                     // Setting Target Hours mode
                Display(TargetTime[2], TargetTime[1]);                  // Display Target time
                BlinkDigit(1);
                break;
                
            case SETTING_TARGET_HOURS:                                                     // Setting Target Minutes mode
                Display(TargetTime[2], TargetTime[1]);                  // Display Target time
                BlinkDigit(2);
                break;
                
            case SETTING_WATERING_SECONDS:
                Display(WateringTime[1], WateringTime[0]);                  // Display Target time
                BlinkDigit(1);
                break;
                
            case SETTING_WATERING_MINUTES:
                Display(WateringTime[1], WateringTime[0]);                  // Display Target time
                BlinkDigit(2);
                break;
                
            case SETTING_VALVE_SECONDS:
                Display(ValveTime[1], ValveTime[0]);                  // Display Target time
                BlinkDigit(1);
                break;
                
            case SETTING_VALVE_MINUTES:
                Display(ValveTime[1], ValveTime[0]);                  // Display Target time
                BlinkDigit(2);
                break;
                
            case WATERING_MODE:
                if(UpdateTimeFlag) {
                    UpdateCurrentTime();
                    if(CurrentTime[0] != LastSecond){
                        
                        if(AddBCD(CurrentWateringTime[0], 0x01) > 0x59){
                            CurrentWateringTime[1] = AddBCD(CurrentWateringTime[1], 0x01);
                            CurrentWateringTime[0] = 0x00;
                        }else CurrentWateringTime[0] = AddBCD(CurrentWateringTime[0], 0x01);
                        
                        LastSecond = CurrentTime[0];
                    }
                    
                    UpdateTimeFlag = 0;
                }
                PIE2bits.TMR3IE = 0;                                    // Disable Buzzer Timer
                PIE1bits.TMR1IE = 0;                                    // Disable 
                INTCONbits.INT0IE = 0;
                INTCON3bits.INT1IE = 0;
                INTCON3bits.INT2IE = 0;
                BuzzerPORT.BuzzerPIN = 0;
                
                if(CurrentWateringTime[1] == 0x00 && CurrentWateringTime[0] == 0x01){
                    CCP1CONbits.DC1B0 = 0;                              // 0.5 duty cycle
                    CCP1CONbits.DC1B1 = 0;
                    CCPR1L = 0x19;
                }else if(CurrentWateringTime[1] == 0x00 && CurrentWateringTime[0] == 0x02){
                    CCP1CONbits.DC1B0 = 0;                              // 0.75 duty cycle
                    CCP1CONbits.DC1B1 = 1;
                    CCPR1L = 0x25;
                }else if(CurrentWateringTime[1] > 0x00 || CurrentWateringTime[0] >= 0x03){
                    CCP1CONbits.DC1B0 = 0;                              // 1 duty cycle
                    CCP1CONbits.DC1B1 = 0;
                    CCPR1L = 0x32;
                }
                BlinkDigit(0);
                Display(CurrentWateringTime[1], CurrentWateringTime[0]);
                
                if(CurrentWateringTime[1] == WateringTime[1] && CurrentWateringTime[0] == WateringTime[0]){
                    CCP1CONbits.DC1B0 = 0;
                    CCP1CONbits.DC1B1 = 0;
                    CCPR1L = 0x00;                                     
                    CurrentWateringTime[0] = 0x00;
                    CurrentWateringTime[1] = 0x00;
                    LastSecond = 0x00;
                    State = VALVE_MODE;
                }
                
                break;
            case VALVE_MODE:
                if(UpdateTimeFlag) {
                    UpdateCurrentTime();
                    if(CurrentTime[0] != LastSecond){
                        
                        if(AddBCD(CurrentValveTime[0], 0x01) > 0x59){
                            CurrentValveTime[1] = AddBCD(CurrentValveTime[1], 0x01);
                            CurrentValveTime[0] = 0x00;
                        }else CurrentValveTime[0] = AddBCD(CurrentValveTime[0], 0x01);
                        
                        LastSecond = CurrentTime[0];
                    }
                    
                    UpdateTimeFlag = 0;
                }

                BuzzerPORT.BuzzerPIN = 0;
                
                // VALVE PORT ON
                ValvePORT.ValvePIN = 1;
                
                Display(CurrentValveTime[1], CurrentValveTime[0]);
                
                if(CurrentValveTime[1] == ValveTime[1] && CurrentValveTime[0] == ValveTime[0]){
                    
                    // VALVE PORT OFF
                    ValvePORT.ValvePIN = 0;
                    PIE2bits.TMR3IE = 1;                                    // Disable Buzzer Timer
                    PIE1bits.TMR1IE = 1;                                    // Disable 
                    INTCONbits.INT0IE = 1;
                    INTCON3bits.INT1IE = 1;
                    INTCON3bits.INT2IE = 1;   
                                                      
                    WaterEnable = 0;
                    WaterEnableLock = 1;
                    WakeUp();
                    CurrentValveTime[0] = 0x00;
                    CurrentValveTime[1] = 0x00;
                    LastSecond = 0x00;
                    State = MAIN_LOOP_HANDLER;
                }
                
                break;
        }
        
        
        
    }
    
    
    return;
}

void __interrupt () my_isr_routine (void) {
    // TIMERS INTERRUPTIONS
    TMR0interruption();
    TMR1interruption();
    TMR3interruption();

    // EXTERNAL INTERRUPTIONS
    INT0interruption();
    INT1interruption();
    INT2interruption();
}

void GPIOconfig(void){

    TRISC = 0;
    
    TRISC |= 0x18;          // Setting RC3(SCL) and RC4(SDA) as input
    
    TRISA = 0;              // Setting A Pins as outputs
    
    TRISB = 0x07;           // Setting External Interrupts as inputs
    
    BuzzerTRIS.BuzzerPIN = 0;
    BuzzerPORT.BuzzerPIN = 1;
    
    WaterPumpTRIS.WaterPumpPIN = 0;
    WaterPumpPORT.WaterPumpPIN = 1;
    
    ValveTRIS.ValvePIN = 0;
    ValvePORT.ValvePIN = 0;
}
void I2Cconfig(void){
    
    SSPADD = 9;             // 100kHz I2C Clock with Fosc = 4MHz
                            // I2C Clock = Fosc / (4*(SSPADD+1)
    SSPCON1 = 0x28;         // SSPEN = 1 and SSPM<3:0> = 1000
}
void TIMERconfig(void){
    T0CON = 0x83;           // TMR0ON = 1 - Enables Timer 0
                            // T08BIT = 0 - Timer 0 is a 8 bit timer
                            // PSA = 0    - Prescaler assigned
                            // T0PS<2:0> = 011 - 1:16 Prescale value
    
    T1CON = 0xB1;           // RD16 = 1 - ReadWrite 16 Timer
                            // TMR1ON = 1 - Timer 1 Enabled
                            // T1CKPS<1:0> = 11 - 1:8 Prescale
    
    T3CON = 0x91;           // RD16 = 1 - Enables register read/write of Timer3 in one bit 16-bit operation
                            // T3CKPS<1:0> = 01 - 1:2 Prescale
                            // TMR3ON = 1 - Enables Timer3
}
void INTconfig(void){
    INTCON = 0xF0;          // GIE = 1 - Enables global interrupts
                            // PEIE = 1 - Enable Peripheral Interrupts
                            // TMR0IE = 1 - Enables timer 0 interrupts
                            // INT0IE = 1 - External Interrupt Enabled
    

    
    PIE2 = 0x02;            // TMR3IE = 1 - TMR3 Overflow Interrupt Enable bit;
    
    
    PIE1 = 0x01;            // TMR1IE = 1 - Timer 1 Interrupt Enabled
    
    INTCON2 &= ~(0xF0);     // INTEDG<0:2> = 000 - Interrupt on falling edge
    
    INTCON3 = 0x18;         // INT2IE = 1 - INT2 enabled
                            // INT1IE = 1 - INT1 enabled
}
void PWMconfig(void){
    // 1 - Set the PWM period by writing to the PR2 register.
    PR2 = 49;                // (1/5*10^3) = [PR2 + 1]*4*(1/4*10^6)*4    -   Period of 0.2 ms
                            // PWM Period = [PR2 + 1]*4*TOSC*TMR2_PRESCALE
    
                            // PWM Duty Cycle = (CCPRXL:CCPXCON<5:4>)*TOSC*(TMR2 Prescale Value)
    
    // 2 - Set the PWM duty cycle by writing to the CCPRxL register and CCPxCON<5:4> bits.
    CCP1CON = 0x00;         // CCPR1L:CCP1CON<5:4> = 100;    -   Period of 0.1 ms (Half of PWM Period)
    CCPR1L = 0x19;          // CCPR1L:CCP1CON<5:4> = 0x64; 0X19
                            // PWM Duty Cycle = (CCPR1L:CCP1CON<5:4>)*TOSC*TMR2_PRESCALE
                            // 0.1*10^-3 = CCPR1L:CCP1CON<5:4>*(1/4*10^-6)*4
    
    // 3 - Make the CCPx pin an output by clearing the appropriate TRIS bit.
    TRISCbits.RC2 = 0;
    
    // 4 - Set the TMR2 prescale value, then enable Timer2 by writing to T2CON.
    T2CON = 0x01;
    
    T2CONbits.TMR2ON = 1;
    
    // 5 - Configure the CCPx module for PWM operation.
    CCP1CON = 0x0C;
    
    __delay_us(1);
    
    CCPR1L = 0x00;
}
void MCUconfig(void) {
    OSCCON |= 0xE2;         // IRCF<2:0> = 110 - 4 MHz
                            // SCS<1:0> = 1x - Internal Oscillator Block
    GPIOconfig();
    
    
    I2Cconfig();
    
    TIMERconfig();
    
    INTconfig();
    
    PWMconfig();
}
void TMR0interruption (void){
    //////////// TIMER 0 INTERRUPT /////////////////
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF){
        
        switch(State){
            case IDLE_MODE:
                UpdateCurrentTime();

                if(CurrentTime[2] == TargetTime[2] && CurrentTime[1] == TargetTime[1]) {
                    WaterEnable = 1;
                    //WakeUp();
                    State = MAIN_LOOP_HANDLER;
                }else{
                    WaterEnable = 0;
                    WaterEnableLock = 0;
                }
                break;
            default:
                UpdateTimeFlag = 1;
                break;
        }
        INTCONbits.TMR0IF = 0;
    }
}
void TMR1interruption (void){
    //////////// TIMER 1 INTERRUPT /////////////////
    if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1){
        
        Blink = ~Blink;
        
        PIR1bits.TMR1IF = 0;
    }
}
void TMR3interruption (void){
    //////////// TIMER 3 INTERRUPT /////////////////
    if(PIE2bits.TMR3IE == 1 && PIR2bits.TMR3IF == 1){
        
        if(BuzzerFlag){
            BuzzerPORT.BuzzerPIN = 0;
            PIE2bits.TMR3IE = 0;
            BuzzerFlag = 0;
        }else{
            BuzzerPORT.BuzzerPIN = 1;
            BuzzerFlag = 1;
        }
        

        PIR2bits.TMR3IF = 0;
    }
}
void INT0interruption (void){
    //////////// EXTERNAL INT 0 ////////////////////////
    if(INTCONbits.INT0IE == 1 && INTCONbits.INT0IF == 1){
        
        Buzz();                         // Buzzing
        //__delay_ms(BUTTON_DELAY);
        switch(State){
            
            
            case SETTING_CURRENT_MINUTES:
                
                State = SETTING_CURRENT_HOURS;
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                
                break;
            
            case SETTING_CURRENT_HOURS:
                State = MAIN_LOOP_HANDLER;
                WakeUp();
                T0CONbits.TMR0ON = 1;    // Disable Timer 0
                T1CONbits.TMR1ON = 0;    // Enable Timer 1
                break;
            
            case IDLE_MODE:
                
                WakeUp();
                State = MAIN_LOOP_HANDLER;
                
                break;
                
            case MAIN_LOOP_HANDLER:                     // colocar nome dos estados
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_TARGET_MINUTES;
                break;
                
            case SETTING_TARGET_MINUTES:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_TARGET_HOURS;
                break;
            case SETTING_TARGET_HOURS:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_WATERING_SECONDS;
                break;
            case SETTING_WATERING_SECONDS:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_WATERING_MINUTES;
                break;
            case SETTING_WATERING_MINUTES:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_VALVE_SECONDS;
                break;
            case SETTING_VALVE_SECONDS:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = SETTING_VALVE_MINUTES;
                break;
            case SETTING_VALVE_MINUTES:
                T0CONbits.TMR0ON = 1;    // Enable Timer 0
                T1CONbits.TMR1ON = 0;    // Disable Timer 1
                WakeUp();
                State = MAIN_LOOP_HANDLER;
                break;
        }
        
        INTCONbits.INT0F = 0;
    }
}
void INT1interruption (void){
    //////////// EXTERNAL INT 1 ////////////////////////
    if(INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1){
        
        Buzz();                         // Buzzing
        //__delay_ms(BUTTON_DELAY);
        
        switch(State){
            case SETTING_CURRENT_MINUTES:
                if(CurrentTime[1] == 0x59) CurrentTime[1] = 0x00;       // Reset Minutes
                else CurrentTime[1] = AddBCD(CurrentTime[1], 0x01);           // Increment Minutes
                
                ChangeTime(0x00, 0x00);
                ChangeTime(0x01, CurrentTime[1]);
                break;
            case SETTING_CURRENT_HOURS:
                if(CurrentTime[2] == 0x23) CurrentTime[2] = 0x00;           // Reset Hours
                else CurrentTime[2] = AddBCD(CurrentTime[2], 0x01);               // Increment Hours
                
                ChangeTime(0x00, 0x00);
                ChangeTime(0x02, CurrentTime[2]);
                break;
                
            case IDLE_MODE:
                WakeUp();
                State = MAIN_LOOP_HANDLER;
                break;
                
            case MAIN_LOOP_HANDLER:
                //Beep
                break;
                
            case SETTING_TARGET_MINUTES:                                                     // Minutes Mode
                if(TargetTime[1] == 0x59) TargetTime[1] = 0x00;       // Reset Minutes
                else TargetTime[1] = AddBCD(TargetTime[1], 0x01);           // Increment Minutes
                break;
                
            case SETTING_TARGET_HOURS:                                                     // Hours Mode
                if(TargetTime[2] == 0x23) TargetTime[2] = 0x00;           // Reset Hours
                else TargetTime[2] = AddBCD(TargetTime[2], 0x01);               // Increment Hours
                break;
            case SETTING_WATERING_SECONDS:
                if(WateringTime[0] == 0x59) WateringTime[0] = 0x00;       // Reset Minutes
                else WateringTime[0] = AddBCD(WateringTime[0], 0x01);           // Increment Minutes
                break;
            case SETTING_WATERING_MINUTES:
                if(WateringTime[1] == 0x59) WateringTime[1] = 0x00;           // Reset Hours
                else WateringTime[1] = AddBCD(WateringTime[1], 0x01);               // Increment Hours
                break;
            case SETTING_VALVE_SECONDS:
                if(ValveTime[0] == 0x59) ValveTime[0] = 0x00;       // Reset Minutes
                else ValveTime[0] = AddBCD(ValveTime[0], 0x01);           // Increment Minutes
                break;
            case SETTING_VALVE_MINUTES:
                if(ValveTime[1] == 0x59) ValveTime[1] = 0x00;           // Reset Hours
                else ValveTime[1] = AddBCD(ValveTime[1], 0x01);               // Increment Hours
                break;
        }
        
        INTCON3bits.INT1IF = 0;
    }
}
void INT2interruption (void){
    //////////// EXTERNAL INT 2 ////////////////////////
    if(INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1){
        
        Buzz();                         // Buzzing
        //__delay_ms(BUTTON_DELAY);
        
        switch(State){
            
            case SETTING_CURRENT_MINUTES:
                if(CurrentTime[1] == 0x00) CurrentTime[1] = 0x59;       // Reset Minutes
                else CurrentTime[1] = SubBCD(CurrentTime[1], 0x01);           // Increment Minutes
                
                ChangeTime(0x00, 0x00);
                ChangeTime(0x01, CurrentTime[1]);
                break;
            case SETTING_CURRENT_HOURS:
                if(CurrentTime[2] == 0x00) CurrentTime[2] = 0x23;           // Reset Hours
                else CurrentTime[2] = SubBCD(CurrentTime[2], 0x01);               // Increment Hours
                
                ChangeTime(0x00, 0x00);
                ChangeTime(0x02, CurrentTime[2]);
                break;
            
            case IDLE_MODE:
                
                WakeUp();

                State = MAIN_LOOP_HANDLER;
                
                break;
                
            case MAIN_LOOP_HANDLER:
                //Beep
                break;

            case SETTING_TARGET_MINUTES:                                                     // Minutes Mode
                if(TargetTime[1] == 0x00) TargetTime[1] = 0x59;       // Reset Minutes
                else TargetTime[1] = SubBCD(TargetTime[1], 0x01);           // Decrement Minutes
                break;
                
            case SETTING_TARGET_HOURS:                                                     // Hours Mode
                if(TargetTime[2] == 0x00) TargetTime[2] = 0x23;           // Reset Hours
                else TargetTime[2] = SubBCD(TargetTime[2], 0x01);               // Decrement Hours
                break;
            case SETTING_WATERING_SECONDS:                                                     // Minutes Mode
                if(WateringTime[0] == 0x00) WateringTime[0] = 0x59;       // Reset Minutes
                else WateringTime[0] = SubBCD(WateringTime[0], 0x01);           // Decrement Minutes
                break;
                
            case SETTING_WATERING_MINUTES:                                                     // Hours Mode
                if(WateringTime[1] == 0x00) WateringTime[1] = 0x59;           // Reset Hours
                else WateringTime[1] = SubBCD(WateringTime[1], 0x01);               // Decrement Hours
                break;
                
            case SETTING_VALVE_SECONDS:                                                     // Minutes Mode
                if(ValveTime[0] == 0x00) ValveTime[0] = 0x59;       // Reset Minutes
                else ValveTime[0] = SubBCD(ValveTime[0], 0x01);           // Decrement Minutes
                break;
                
            case SETTING_VALVE_MINUTES:                                                     // Hours Mode
                if(ValveTime[1] == 0x00) ValveTime[1] = 0x59;           // Reset Hours
                else ValveTime[1] = SubBCD(ValveTime[1], 0x01);               // Decrement Hours
                break;
        }
        
        INTCON3bits.INT2IF = 0;
    }
}


void MCUinit(void){
    Display(0x88, 0x88);
    
}

void WakeUp(void){
    UpdateCurrentTime();

    if (AddBCD(CurrentTime[1], 0x02) > 0x59) {
        if(AddBCD(CurrentTime[1], 0x01) > 0x23){
            WakeUpTime[2] = 0x00;
            WakeUpTime[1] = AddBCD(CurrentTime[1], 0x02);
            WakeUpTime[1] = SubBCD(CurrentTime[1], 0x60);
        }else{
            WakeUpTime[2] = AddBCD(CurrentTime[2], 0x01);
            WakeUpTime[1] = AddBCD(CurrentTime[1], 0x02);
            WakeUpTime[1] = SubBCD(CurrentTime[1], 0x60);
        }
    }else{
        WakeUpTime[2] = CurrentTime[2];
        WakeUpTime[1] = AddBCD(CurrentTime[1], 0x02);
    } 
    UpdateTimeFlag = 1;
}

void BlinkDigit(char index){
    switch(index){
        case 0:
            Blink_First_Digit = 0;                            // Disable Blink Hours
            Blink_Second_Digit = 0;                          // Enable Blink Minutes
        break;

        case 1:
            Blink_First_Digit = 0;                            // Disable Blink Hours
            Blink_Second_Digit = 1;                          // Enable Blink Minutes
        break;

        case 2:
            Blink_First_Digit = 1;                            // Disable Blink Hours
            Blink_Second_Digit = 0;                          // Enable Blink Minutes
        break;
    }
    
}

char AddBCD(char Number1, char Number2){
    char temp = Number1 + Number2;
    
    if ((temp & 0x0F) > 9) return temp + 0x06;
    else return temp;
}
char SubBCD(char Number1, char Number2){
    char temp = Number1 - Number2;
    
    if ((temp & 0x0F) > 9) return temp - 0x06;
    else return temp;
}
void Display(char Hours, char Minutes){
    char Upper_Hours = ((Hours & 0xF0) >> 4);       // Extracting only upper nimble
    char Lower_Hours = Hours & 0x0F;                // Extracting only lower nimble
    
    char Upper_Minutes = ((Minutes & 0xF0) >> 4);   // Extracting only upper nimble
    char Lower_Minutes = Minutes & 0x0F;            // Extracting only lower nimble
    
    if(Blink_First_Digit && Blink) {
        LATA |= 0x00;         // Blinking Hours
        __delay_us(800);
    }
    else {
        LATA = Upper_Hours;                     // Displaying Upper Hours on 1 digit
        LATB |= 0x10;
        __delay_us(800);
        LATB &= ~(0x10);
    }
    
    if(Blink_First_Digit && Blink) {
        LATA |= 0x00;         // Blinking Hours
        __delay_us(800);
    }
    else { 
        LATA = Lower_Hours;                     // Displaying Lower Hours on 2 digit
        LATB |= 0x20;
        __delay_us(800);
        LATB &= ~(0x20);
    }
    
    if(Blink_Second_Digit && Blink) {
        LATA |= 0x00;         // Blinking Hours
        __delay_us(800);
    }
    else {
        LATA = Upper_Minutes;                   // Displaying Upper Minutes on 3 digit
        LATB |= 0x40;
        __delay_us(800);
        LATB &= ~(0x40);
    }
    
    if(Blink_Second_Digit && Blink) {
        LATA |= 0x00;         // Blinking Hours
        __delay_us(800);
    }else {
        LATA = Lower_Minutes;                   // Displaying Lower Minutes on 4 digit
        LATB |= 0x80;
        __delay_us(800);
        LATB &= ~(0x80);
    }
}
void Buzz(void){
    
    PIE2bits.TMR3IE = 1;
    TMR3 = 0;
}
void ChangeTime(uint8_t Address, uint8_t Time){
    
    SSPCON2bits.SEN = 1;                            // Start Enable bit
    while(SEN);                                     // wait start condition to finish
    PIR1bits.SSPIF = 0;                             // Clear SSP interrupt flag
    
    SSPBUF = 0xD0;                                  // Send Slave Address + Write
    while(!SSPIF);                                  // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;                             // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){                       // Check if ACK was received
        SSPCON2bits.PEN = 1;                        // Initiates a Stop condition
        while(PEN);                                 // Wait for stop to finish
        return;
    }
    
    SSPBUF = Address;         // Send Register Address
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return;
    }
    
    SSPBUF = Time;         // Send Minutes
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag

    
    SSPCON2bits.PEN = 1;  // Initiates a Stop condition
    while(PEN);
}
uint8_t ReadTime(uint8_t Address){
    char TempData;
    
    SSPCON2bits.SEN = 1;   // Start Enable bit
    while(SEN);            // wait start condition to finish
    PIR1bits.SSPIF = 0;    // Clear SSP interrupt flag
    
    SSPBUF = 0xD0;         // Send Slave Address + Write
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return(0xFF);
    }
    
    SSPBUF = Address;         // Send Register Address
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return(0xFF);
    }
    
    SSPCON2bits.RSEN = 1;   // Restart Condition
    while(SSPCON2bits.RSEN); // Wait for restart condition to finish
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    
    SSPBUF = 0xD1;         // Send Slave Address + Write
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return(0xFF);
    }
    
    SSPCON2bits.RCEN = 1;   // Receive Enable bit
    while(!SSPSTATbits.BF); // Wait for Buffer to become full
    TempData = SSPBUF;      // Save the Data Bytes
    
    SSPCON2bits.ACKDT = 1;  // Send ACK bit
    SSPCON2bits.ACKEN = 1;  // Initiate NACK
    while(ACKEN);           // Wait for NACK
    
    SSPCON2bits.PEN = 1;    // Stop Condition
    while(PEN);             // Wait for Stop condition
    
    return TempData;        // Return the Data Byte value
     
}
void UpdateCurrentTime(void){
    SSPCON2bits.SEN = 1;   // Start Enable bit
    while(SEN);            // wait start condition to finish
    PIR1bits.SSPIF = 0;    // Clear SSP interrupt flag
    
    SSPBUF = 0xD0;         // Send Slave Address + Write
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return;
    }
    
    SSPBUF = 0;         // Send Register Address
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return;
    }
    
    SSPCON2bits.RSEN = 1;   // Restart Condition
    while(SSPCON2bits.RSEN); // Wait for restart condition to finish
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    
    SSPBUF = 0xD1;         // Send Slave Address + Write
    while(!SSPIF);         // Wait for Ack SSIF is set every 9th clock cycle
    PIR1bits.SSPIF = 0;    // Clear SSP Interrupt Flag
    if (SSPCON2bits.ACKSTAT){ // Check if ACK was received
        SSPCON2bits.PEN = 1;  // Initiates a Stop condition
        while(PEN);           // Wait for stop to finish
        return;
    }
    
    SSPCON2bits.RCEN = 1;   // Receive Enable bit
    while(!SSPSTATbits.BF); // Wait for Buffer to become full
    CurrentTime[0] = SSPBUF;      // Save the Data Bytes
    
    SSPCON2bits.ACKDT = 0;  // Send ACK bit
    SSPCON2bits.ACKEN = 1;  // Initiate NACK
    while(ACKEN);           // Wait for NACK
    
    SSPCON2bits.RCEN = 1;   // Receive Enable bit
    while(!SSPSTATbits.BF); // Wait for Buffer to become full
    CurrentTime[1] = SSPBUF;      // Save the Data Bytes
    
    SSPCON2bits.ACKDT = 0;  // Send ACK bit
    SSPCON2bits.ACKEN = 1;  // Initiate NACK
    while(ACKEN);           // Wait for NACK
    
    SSPCON2bits.RCEN = 1;   // Receive Enable bit
    while(!SSPSTATbits.BF); // Wait for Buffer to become full
    CurrentTime[2] = SSPBUF;      // Save the Data Bytes
    
    SSPCON2bits.ACKDT = 1;  // Send ~ACK bit
    SSPCON2bits.ACKEN = 1;  // Initiate NACK
    while(ACKEN);           // Wait for NACK
    
    SSPCON2bits.PEN = 1;    // Stop Condition
    while(PEN);             // Wait for Stop condition     
}
