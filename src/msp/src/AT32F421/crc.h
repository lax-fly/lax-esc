#include "msp.h"

class Crc : public CrcIf
{
private:
    uint8_t poly;
    uint8_t start;

public:
    virtual void set_start(uint8_t val);
    virtual void set_poly(uint8_t poly);
    virtual uint8_t calc(uint8_t *data, uint32_t sz);
    virtual uint32_t calc(uint8_t data);
    static CrcIf *singleton(); // do not try to delete the returned object
};