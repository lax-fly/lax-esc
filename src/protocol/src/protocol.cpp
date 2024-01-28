#include "protocol.h"
#include "serial.h"

#include <assert.h>

static Serial *serial = nullptr;

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
        /* code */

    default:
        assert(false);
        break;
    }
    assert(false);
    return nullptr;
}