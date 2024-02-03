#include "protocol.h"
#include "serial.h"
#include "dshot.h"

#include <assert.h>

static Serial *serial = nullptr;
static Dshot *dshot = nullptr;

Protocol *Protocol::singleton(Type type)
{
    switch (type)
    {
    case SERIAL:
        if (serial)
            return serial;
        return serial = new Serial();
        break;
    case DSHOT:
        if (dshot)
            return dshot;
        return dshot = new Dshot();

    default:
        assert(false);
        break;
    }
    assert(false);
    return nullptr;
}