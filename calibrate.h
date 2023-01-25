#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#ifdef CALIBRATE
inline uint16_t calibrateFullHeight()
{
    distance = measureDistance();

    if (distance > TOO_FAR_HEIGHT)
        return CALBIRATION_SENSOR_TOO_FAR;
    else if (distance < TOO_CLOSE_HEIGHT)
        return CALBIRATION_SENSOR_TOO_CLOSE;

    FullHeight = FULL_WATER + distance;
    eeprom_write_word(&EE_FullHeight, FullHeight);
    return 0;
}
#endif

inline void calibrate()
{
#ifdef CALIBRATE
    uint16_t error_code = calibrateFullHeight();
    if (error_code)
        displayError(error_code); // display error code infinitly (until reset)

    flashValue(distance);   // display distance for 1 sec then clear sceen
    flashValue(fullheight); // display full height for 1 sec then clear sceen
#endif
}

#endif
