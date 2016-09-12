#include "7seg.h"

#define SEG_1   LATA1
#define SEG_2   LATA2
#define SEG_3   LATC7
#define SEG_4   LATC6
#define SEG_5   LATA3

static void print_column(uint8_t col) {
    SEG_1 = (col == 1);
    SEG_2 = (col == 2);
    SEG_3 = (col == 3);
    SEG_4 = (col == 4);
    SEG_5 = (col == 5);
}

static void print_pattern(uint8_t pat) {
    LATB = ~pat;
}

void print_7seg(uint8_t col, char c) {
    // BGACDEHF
    static uint8_t pattern[10] = {
        0b10111110, //0
        0b10010000, //1
        0b11101100, //2
        0b11111000, //3
        0b11010010, //4
        0b01111010, //5
        0b01111110, //6
        0b10110010, //7
        0b11111110, //8
        0b11111010, //9
    };
    uint8_t pat;
    switch (c) {
        case ' ':
            pat = 0b00000000;
            break;
        case ':':
            pat = 0b10100000;
            break;
        case '-':
            pat = 0b01000000;
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            pat = pattern[c - '0'];
            break;
        default:
            pat = 0b00000010;
            break;
    }
    print_column(0);
    print_pattern(pat);
    print_column(col);
}
