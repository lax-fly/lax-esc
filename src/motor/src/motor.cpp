#include "motor.h"
#include "bldc.h"
#include "assert.h"

static Bldc* bldc;

MotorIf *MotorIf::singleton(Type type)
{
    switch (type)
    {
    case BLDC:
        if (bldc)
            return bldc;
        return bldc = new Bldc();
        break;
    case PMSM:  // not yet
    case BDCM:  // not yet
    default:
        assert(false);
        break;
    }
    return nullptr;
}