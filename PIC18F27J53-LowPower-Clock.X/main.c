/*
 * Title:   PIC18F27J53 Low Power Clock
 * Date:	2016.09.12
 * Author:	kerikun11
 */

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 2       // PLL Prescaler Selection (Divide by 2 (8 MHz oscillator input))
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration Bit (PLL Disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset (Disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)
// CONFIG2L
#pragma config OSC = INTOSC     // Oscillator (INTOSC)
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = OFF     // EC Clock Out Enable Bit  (CLKO output disabled on the RA6 pin)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)
// CONFIG2H
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024)
// CONFIG3L
#pragma config DSWDTOSC = T1OSCREF// DSWDT Clock Select (DSWDT uses T1OSC/T1CKI)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer (Disabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))
// CONFIG3H
#pragma config IOL1WAY = OFF    // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set and cleared as needed)
#pragma config ADCSEL = BIT12   // ADC 10 or 12 Bit Select (12 - Bit ADC Enabled)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)
// CONFIG4L
#pragma config WPFP = PAGE_127  // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 127)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region  (Configuration Words page not erase/write-protected)
// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<6:0>/WPEND region ignored)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select bit (valid when WPDIS = 0) (Pages WPFP<6:0> through Configuration Words erase/write protected)
#pragma config LS48MHZ = SYS48X8// Low Speed USB mode with 48 MHz system clock bit (System clock at 48 MHz USB CLKEN divide-by is set to 8)

#include <xc.h>
#include <stdint.h>
#include "7seg.h"

#define _XTAL_FREQ 4000000

void dinamic(void) {
    static uint8_t dinamic_counter = 1;
    static uint8_t sec, min, hou, wkd;
    switch (dinamic_counter) {
        case 1:
            print_7seg(4, '0' + (min & 0x0F));
            TMR1L += 0xF0;
            break;
        case 2:
            print_7seg(3, '0' + (min >> 4));
            TMR1L += 0xF0;
            break;
        case 3:
            print_7seg(2, '0' + (hou & 0x0F));
            TMR1L += 0xF0;
            break;
        case 4:
            if (hou & 0xF0) print_7seg(1, '0' + (hou >> 4));
            else print_7seg(1, ' ');
            TMR1L += 0xF0;
            break;
        case 5:
            print_7seg(5, RTCCFGbits.HALFSEC ? ' ' : ':');
            TMR1L += 0xF0;
            break;
        default:
            print_7seg(-1, ' ');
            RTCCFGbits.RTCPTR0 = 1;
            RTCCFGbits.RTCPTR1 = 0;
            while (RTCCFGbits.RTCSYNC);
            hou = RTCVALL;
            wkd = RTCVALH;
            sec = RTCVALL;
            min = RTCVALH;
            dinamic_counter = 0;
            break;
    }
    dinamic_counter++;
}

void interrupt ISR(void) {
    if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
        INTCONbits.GIE = 0;
        print_7seg(-1, ':');
        DSCONHbits.DSEN = 1;
        SLEEP();
    }
    if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
        if (PORTAbits.RA5 == 0) {
            print_7seg(-1, ' ');
            RTCCFGbits.RTCPTR0 = 0;
            RTCCFGbits.RTCPTR1 = 0;
            while (RTCCFGbits.RTCSYNC);
            uint8_t val = RTCVALH;
            val = ((val >> 4)*10 + (val & 0x0F) + 1) % 60;
            RTCCFGbits.RTCPTR0 = 0;
            RTCCFGbits.RTCPTR1 = 0;
            RTCVALH = ((val / 10) << 4) + ((val % 10)&0x0F);
            RTCVALL = 0;
        }
    }
    if (INTCON3bits.INT2IE && INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF = 0;
        if (PORTCbits.RC2 == 0) {
            print_7seg(-1, ' ');
            RTCCFGbits.RTCPTR0 = 1;
            RTCCFGbits.RTCPTR1 = 0;
            while (RTCCFGbits.RTCSYNC);
            uint8_t val = RTCVALL;
            val = ((val >> 4)*10 + (val & 0x0F) + 1) % 24;
            RTCCFGbits.RTCPTR0 = 1;
            RTCCFGbits.RTCPTR1 = 0;
            RTCVALL = ((val / 10) << 4) + ((val % 10)&0x0F);
        }
    }
    if (PIE1bits.TMR1IE && PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        TMR1H = 0xFF;
        dinamic();
    }
}

int main(void) {
    OSCCONbits.SCS = 0;
    DSCONLbits.RELEASE = 0;
    TRISA = 0b01110001; // RA7, RA6, RA5, Vcap, RA3, RA2, RA1,  RA0
    TRISB = 0b00000001; // RB7, RB6, SDA, SCL,  RB3, RB2, RB1,  RB0
    TRISC = 0b00111110; // RXD, TXD, D+,  D-,   Vusb,RC2, T1OSI,T1OSO
    ANCON0 = 0b11111111; // x,x,x,AN4(RA5),AN3(RA3),AN2(RA2),AN1(RA1),AN0(RA0)
    ANCON1 = 0b00011111; // VBG,x,x,AN12(RB0),AN11(RC2),AN10(RB1),AN9(RB3),AN8(RB2)

    // Timer1
    T1CON = 0b10001101;
    PIE1bits.TMR1IE = 1;

    // RTC
    EECON2 = 0x55;
    EECON2 = 0xAA;
    RTCCFGbits.RTCWREN = 1;
    RTCCFG = 0b10100000;
    RTCCAL = 0x00;

    // INT0
    INTCON2bits.INTEDG0 = 0;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;

    // INT1
    RPINR1 = 2; // INT1 -> RP2
    INTCON2bits.INTEDG1 = 0;
    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT1IE = 1;

    // INT2
    RPINR2 = 13; // INT2 -> RP13
    INTCON2bits.INTEDG2 = 0;
    INTCON3bits.INT2IF = 0;
    INTCON3bits.INT2IE = 1;

    // Enable Interrupts
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    while (1) {
        SLEEP();
        NOP();
    }
    return 0;
}