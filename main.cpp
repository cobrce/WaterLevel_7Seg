/* #define TEST // uncomment to enable test mode for simulation */
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>
/* #include "ShiftRegister.h" */
#include "calibrate.h"

// Pins definition
#define HB_LED PD7 // heartbeat LED
#define PIN_UART_TX PD1
#define PIN_I2C_SDA PC4
#define PIN_I2C_SCL PC5
// VL53L0X sensor library
#include "vl53l0x-non-arduino/VL53L0X.h"
#include "vl53l0x-non-arduino/util/i2cmaster.h"
#include "vl53l0x-non-arduino/util/debugPrint.h"
#include "vl53l0x-non-arduino/util/millis.h"

#define __COMPILING_AVR_LIBC__ 1 // just to avoid some intellisence annoying messages

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif // !FALSE

// some constants
#define SENSOR_HEIGHT 30
#define FULL_WATER 190
#define TOO_FAR_HEIGHT (FULL_WATER + SENSOR_HEIGHT + 10)
#define TOO_CLOSE_HEIGHT 10
#define HYSTERESIS 5

void pushError(uint8_t errorValue);

uint16_t EEMEM EE_FullHeight = FULL_WATER + SENSOR_HEIGHT; // eeprom variable in case we calibrate and save the actual values
volatile uint16_t FullHeight = 0;
volatile uint16_t distance;       // decalred as global for debug
volatile uint16_t levelInPercent; // same as above, but this is a claculated percentage of water level

// calculate a bargraph representation of a value then shift it to fit the shift register
// the bargraph contains 10 leds, the percentage is rounded down to 10s, example : 51 is displayed as 5leds, 0 is 0 led, 34 is 3 leds..etc
uint32_t calculateBarGraph(uint16_t valueInPercent, uint8_t shift)
{

    if (valueInPercent > 100) // check limits
        valueInPercent = 100;
    uint32_t result = 0;

    uint8_t valueInTens = ((valueInPercent / 10)) % 11; // round up the percents to tens

    while (valueInTens--) // for each "1" in tens (i.e for each 10 in percents) light an led
        result = (result << 1) | 1;

    return result << shift; // shift the final result to the desired bit position (see the call lcoation for more info)
}

// swap the order of bits, due to the way the bargraph is wired in the PCB
uint8_t swapBits(uint8_t input)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++)
        result = (result << 1) | ((input >> i) & 1);
    return result;
}



uint8_t SevenSegNumbers[] = {
    // seven segments coded numbers
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
};

// extract a digit form a number at a given index, then return its seven segment reperesentation
uint8_t extractSevenSegDigit(uint32_t value, uint8_t index, uint8_t reverse=false)
{
    for (int i = 0; i < index; i++)
        value /= 10;
    value %= 10;
    uint8_t result = SevenSegNumbers[value]; // convert number into 7 seg representation
    if (reverse)
        result = ~result;
    return result;
}

void displayInPins(uint8_t value, volatile uint8_t* port, char* pins, uint8_t number_of_pins)
{
    for (uint8_t i = 0; i < number_of_pins;i++)
    {
        uint8_t pinValue = (value >> i) & 1;
        if (pinValue)
            *port |= _BV(pins[i]);
        else
            *port &= ~_BV(pins[i]);
    }

}
// the first shift register has the data of the last digit of seven segment display and the the first bit of the bargraph
// the second shift register has the data of the second digit of seven segments
// the third ...
// the fourth shift register has 8 bits of the data of the bargraph swapped (because of schematic)
// the 10th bit of the bargraph is conencted to pin PD4 of the micro controller
uint32_t bargraphValue = 0; // set as global for debugging
volatile uint8_t topBargraphValue = 0; // updated by TIMER1
void display()
{
    // if water level is above 95 then use a solid led light, otherwise blink it (the blink is done by TIMER1)
    topBargraphValue = levelInPercent > 95;
    bargraphValue = calculateBarGraph(levelInPercent, 0);

    char portdPins[] = {PD0,PD2,PD3};
    char portcPins[] = {PC2,PC3,PC4,PC5,PC6,PC7};
    displayInPins(bargraphValue & 0b111111, &PORTC, portcPins, sizeof(portcPins));
    displayInPins(bargraphValue >> 6, &PORTD, portdPins, sizeof(portdPins));

    uint8_t tempLevel = levelInPercent;
    if (tempLevel > 99)
        tempLevel = 99;
    
    PORTA = extractSevenSegDigit(tempLevel, 1,true); // reverse is set to true because of common anode
    PORTB = extractSevenSegDigit(tempLevel, 0,true);

    /* UpdateRegister(); // latch */
}

uint16_t measureDistance() // in cm
{
    statInfo_t xTraStats;

    distance = readRangeSingleMillimeters(&xTraStats) / 10;

    debug_dec(distance);
    debug_str("cm ");
    return distance;
}

#define N_SAMPLES 10
uint16_t data[N_SAMPLES] = {0};

uint16_t calculateMean(uint16_t *data, uint16_t len)
{
    uint16_t mean = 0;
    for (int i = 0; i < len; i++)
        mean += data[i];
    return mean / len;
}

uint16_t measureMeanDistance()
{
    // take several samples
    for (int i = 0; i < N_SAMPLES;)
    {
        uint16_t distance = measureDistance();
        if ((distance | 1) == 8191) // is it an error code?
        {
            continue;
        }
        else
        {
            data[i] = distance;
            i++;
        }
    }
    // calculate the mean of the values
    uint32_t mean = calculateMean(data, N_SAMPLES);

    // order the samples based on how far from the mean value
    for (int i = 0; i < N_SAMPLES; i++)
        for (int j = i + 1; j < N_SAMPLES; j++)
            if (fabs(mean - data[i]) > fabs(mean - data[j]))
            {
                uint16_t temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
    // calculate the mean of half the data that are closer to the average
    return calculateMean(data, N_SAMPLES / 2);
}


// write the duty cycle of the heart beat led
void pwmWrite(uint8_t value)
{
    OCR2 = value;
}

// initialize the timer1 for 10ms interrupt
void initTimer1()
{
    //--------------------------------------------------
    // Timer1 for the heartbeat update
    //--------------------------------------------------
    TCCR1A = 0; // no pin output
    TCCR1B = (1 << WGM12)|(1 << CS11) | (1 << CS10);// 64 prescaler, Clear Timer on Compare Match (Mode 4),  TOP=OCR1A
    OCR1A = ((F_CPU / 64) / 100);                      // every 10ms
    TIMSK |= (1 << OCIE1A);
}

// init the pwm pin
void pwmInit()
{
    DDRD |= _BV(HB_LED);
    TCCR2 = _BV(COM21) | _BV(WGM21) | _BV(WGM20); // Fast pwm mode, output in OC2 pin (PD7)
    TCCR2 |= _BV(CS20);                             // no prescaling for the timer
    pwmWrite(0);                                    // initially set the duty cycle at 0
}

// initialization function, to init several peripherals, GPIOs and timers
void init()
{
    debugInit();
    //--------------------------------------------------
    // GPIOs
    //--------------------------------------------------
    UCSRB &= ~_BV(RXEN); // Disable UART RX
    /* DDRD = _BV(PIN_UART_TX); */
    /* DDRD |= _BV(PD4); // PD4 as output for A9 of bargraph */

    DDRC = 255;
    DDRD = 255;
    DDRA = 255;
    DDRB = 255;

    i2c_init();
    initMillis();
    initTimer1();
    pwmInit();
    /* InitShiftRegister(); */

    sei();
}

volatile uint8_t heartBeatValue = 20; // the initial value of the hearbeat pwm
volatile int8_t heartBeatDelta = 1;   // the step to progress the pwm value
volatile uint8_t heartBeatDelay = 0;
void heartBeat()
{
    heartBeatDelay++;
    if (heartBeatDelay < 2)
        return;
    
    heartBeatDelay = 0;

    if ((heartBeatValue > 60) | (heartBeatValue < 10)) // every time a max/min value is reached change the direction of the pwm
        heartBeatDelta = -heartBeatDelta;

    heartBeatValue += heartBeatDelta; // increase (or decrease) the pwm value
    pwmWrite(heartBeatValue);
}

// interrupt handler for the timer1 compare match that ticks every 10ms
volatile uint8_t topBargraphBlinkCounter = 0;
volatile uint8_t topBargraphBlinkState = 0;
ISR(TIMER1_COMPA_vect)
{
    heartBeat();              // update the heartbeat pwm
    
    // blink the top bargraph led
    topBargraphBlinkCounter++;
    if (topBargraphBlinkCounter > 50)
    {
        topBargraphBlinkState = 1 - topBargraphBlinkState;
        topBargraphBlinkCounter = 0;
    }
    char topBarGraphPin[] = {PD4};
    const auto topBargraphPort = &PORTD;
    // either blink or write a solid value on bargraph top led (depends on the value calculated in display)
    displayInPins(topBargraphBlinkState | topBargraphValue,topBargraphPort,topBarGraphPin,1);

}

volatile uint32_t now; 
int main(void)
{
    init();

// this code is used for tests in simulator, so the measurments are generated and the timers are replaced by delays
#ifdef TEST
    levelInPercent = 88;
    int8_t step = 0;
    /* int8_t errorCounter = 20; */
    volatile uint32_t prevUpdate = 0;
    while (1)
    {
        now = millis();
        if ((now - prevUpdate) >= 1000)
        {
            prevUpdate = now;

            /* if (levelInPercent > 90) */
            /*     step = -6; */
            /* else if (levelInPercent < 11) */
            /*     step = 6; */
            /* levelInPercent += step; */

            debug_dec(levelInPercent);
            debug_putc(' ');
        }
        display();
    }
#endif

    initVL53L0X(0);

    // configure the VLC53L0X sensor
    setSignalRateLimit(0.1);
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(500 * 1000UL);

    // read the fullheight from EEPROM, it's eventually updated if calibration is enabled
    FullHeight = eeprom_read_word(&EE_FullHeight);

    calibrate(); // do the calibration (if enable)

    while (1) // main loop
    {
        uint16_t distance = measureMeanDistance(); // start the distance measurement and return the result

        levelInPercent = ((FullHeight - distance) * 100 / FULL_WATER);

        if (levelInPercent > 100) // maximum 100%
        {
            levelInPercent = 100;
            // FullHeight = FULL_WATER + mean_distance; // update fullheight
        }
        debug_dec(levelInPercent); // send value through serial
        debug_str("% ");

        display(); 

        _delay_ms(200);
    }
}
