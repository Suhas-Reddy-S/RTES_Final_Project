#include <stdint.h>
void init_i2c0(void);
void write_to_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                            uint8_t nargs, ...);
uint32_t read_from_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                                 uint8_t reg);
