/*
 
/*
 * File:   main.c
 * Author: Mateus
 *
 * Created on May 26, 2021, 4:00 PM
 */
 /*
#include "xc8.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


void ChangeTime(uint8_t Address, uint8_t Time);
uint8_t ReadTime(uint8_t Address);
char AddBCD(char Number);
char SubBCD(char Number);

void Display(char Hours, char Minutes);

char Current_Hours;

char Current_Minutes;

char Target_Hours = 0x12;

char Target_Minutes = 0x00;

char Blink_Hours = 0;

char Blink_Minutes = 0;

char Blink = 0;

char State = 0;

int Value = 0;

void main(void) {
    
    OSCCON |= 0X62;         // IRCF<2:0> = 110 - 4 MHz
                            // SCS<1:0> = 1x - Internal Oscillator Block
    
    TRISC |= 0xFF;          // Setting RC3(SCL) and RC4(SDA) as input
    
    SSPADD = 9;             // 100kHz I2C Clock with Fosc = 4MHz
                            // I2C Clock = Fosc / (4*(SSPADD+1)
    SSPCON1 = 0x28;         // SSPEN = 1 and SSPM<3:0> = 1000
    
    T0CON = 0x82;           // TMR0ON = 1 - Enables Timer 0
                            // T08BIT = 0 - Timer 0 is a 8 bit timer
                            // PSA = 0    - Prescaler assigned
                            // T0PS<2:0> = 100 - 1:32 Prescale value
    
    T1CON = 0XB1;           // RD16 = 1 - ReadWrite 16 Timer
                            // TMR1ON = 1 - Timer 1 Enabled
                            // T1CKPS<1:0> = 11 - 1:8 Prescale
    
    INTCON = 0xF0;          // GIE = 1 - Enables global interrupts
                            // PEIE = 1 - Enable Peripheral Interrupts
                            // TMR0IE = 1 - Enables timer 0 interrupts
                            // INT0IE = 1 - External Interrupt Enabled
    
    PIE1 = 0x01;            // TMR1IE = 1 - Timer 1 Interrupt Enabled
    
    INTCON2 &= ~(0xF0);     // INTEDG<0:2> = 000 - Interrupt on falling edge
    
    INTCON3 = 0x18;         // INT2IE = 1 - INT2 enabled
                            // INT1IE = 1 - INT1 enabled
    
    ChangeTime(0x00, 0x00);
    ChangeTime(0x01, 0x00);
    ChangeTime(0x02, 0x12);
        
    TRISA = 0;              // Setting A Pins as outputs
    
    TRISB = 0x07;           // Setting External Interrupts as inputs
    
    
    while(1){
        
        switch(State){
            case 0:
                //Display(Current_Hours, Current_Minutes);
                
                PORTA = Value;
                PORTA |= 0xF0;
                __delay_us(5);
                Blink_Hours = 0;
                Blink_Minutes = 0;
                break;
                
            case 1:
                Display(Target_Hours, Target_Minutes);
                Blink_Minutes = 1;
                Blink_Hours = 0;
                break;
                
            case 2:
                Display(Target_Hours, Target_Minutes);
                Blink_Hours = 1;
                Blink_Minutes = 0;
                break;
        }
        
        
        
    }
    
    
    return;
}

void __interrupt () my_isr_routine (void) {
    
    //////////// TIMER 0 INTERRUPT /////////////////
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF){
        //Current_Hours = ReadTime(0x02);
        
        //Current_Minutes = ReadTime(0x01);
        
        Value = 0x0F & ReadTime(0x01);
        //if(Current_Hours == Target_Hours && Current_Minutes == Target_Minutes) PORTBbits.RB3 = 1;
        //else PORTBbits.RB3 = 0;
        
        INTCONbits.TMR0IF = 0;
    }
    
    //////////// TIMER 1 INTERRUPT /////////////////
    if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1){
        
        Blink = ~Blink;
        
        PIR1bits.TMR1IF = 0;
    }
    
    //////////// EXTERNAL INT 0 ////////////////////////
    if(INTCONbits.INT0IE == 1 && INTCONbits.INT0IF == 1){
        
        switch(State){
            case 0:
                State = 1;
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                break;
                
            case 1:
                T0CONbits.TMR0ON = 0;    // Disable Timer 0
                T1CONbits.TMR1ON = 1;    // Enable Timer 1
                State = 2;
                break;
                
            case 2:
                T0CONbits.TMR0ON = 1;    // Enable Timer 0
                T1CONbits.TMR1ON = 0;    // Disable Timer 1
                State = 0;
                break;
        }
        
        INTCONbits.INT0F = 0;
    }
    
    //////////// EXTERNAL INT 1 ////////////////////////
    if(INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1){
        
        switch(State){
            case 0:
                //Beep
                break;
            case 1:                                                     // Minutes Mode
                if(Target_Minutes == 0x59) Target_Minutes = 0x00;       // Reset Minutes
                else Target_Minutes = AddBCD(Target_Minutes);           // Increment Minutes
                break;
            case 2:                                                     // Hours Mode
                if(Target_Hours == 0x23) Target_Hours = 0x00;           // Reset Hours
                else Target_Hours = AddBCD(Target_Hours);               // Increment Hours
                break;
        }
        
        INTCON3bits.INT1IF = 0;
    }
    
    //////////// EXTERNAL INT 2 ////////////////////////
    if(INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1){
        
        switch(State){
            case 0:
                //Beep
                break;
            case 1:                                                     // Minutes Mode
                if(Target_Minutes == 0x00) Target_Minutes = 0x59;       // Reset Minutes
                else Target_Minutes = SubBCD(Target_Minutes);           // Decrement Minutes
                break;
            case 2:                                                     // Hours Mode
                if(Target_Hours == 0x00) Target_Hours = 0x23;           // Reset Hours
                else Target_Hours = SubBCD(Target_Hours);               // Decrement Hours
                break;
        }
        
        INTCON3bits.INT2IF = 0;
    }


}

char AddBCD(char Number){
    char Upper_Number = ((Number & 0xF0) >> 4);       // Extracting only upper nimble
    char Lower_Number = Number & 0x0F;                // Extracting only lower nimble
    
    if(Lower_Number < 9) Lower_Number++;
    else{
        Lower_Number = 0;
        Upper_Number++;
    }
    return ((Upper_Number << 4) | Lower_Number);
}

char SubBCD(char Number){
    char Upper_Number = ((Number & 0xF0) >> 4);       // Extracting only upper nimble
    char Lower_Number = Number & 0x0F;                // Extracting only lower nimble
    
    if(Lower_Number > 0) Lower_Number--;
    else{
        Lower_Number = 9;
        Upper_Number--;
    }
    return ((Upper_Number << 4) | Lower_Number);
}

void Display(char Hours, char Minutes){
    char Upper_Hours = ((Hours & 0xF0) >> 4);       // Extracting only upper nimble
    char Lower_Hours = Hours & 0x0F;                // Extracting only lower nimble
    
    char Upper_Minutes = ((Minutes & 0xF0) >> 4);   // Extracting only upper nimble
    char Lower_Minutes = Minutes & 0x0F;            // Extracting only lower nimble
    
    if(Blink_Hours && Blink) PORTA |= 0x00;         // Blinking Hours
    else PORTA = 0x80 | Upper_Hours;                     // Displaying Upper Hours on 1 digit
    
    if(Blink_Hours && Blink) PORTA |= 0x00;         // Blinking Hours
    else PORTA = 0x40 | Lower_Hours;                     // Displaying Lower Hours on 2 digit
    
    if(Blink_Minutes && Blink) PORTA |= 0x00;         // Blinking Minutes
    else PORTA = 0x20 | Upper_Minutes;                   // Displaying Upper Minutes on 3 digit
    
    if(Blink_Minutes && Blink) PORTA |= 0x00;         // Blinking Minutes
    else PORTA = 0x10 | Lower_Minutes;                   // Displaying Lower Minutes on 4 digit
    
    __delay_us(1);                                 // Time to prevent overflow
    
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

 * 
 */