#pragma once
#include "msp.h"
#include "protocol.h"

// Dshot must be singleton
class Dshot : public Protocol
{
private:
public:
    Dshot();
    virtual ~Dshot();
    virtual void poll(void);
    virtual bool signal_lost();

    void bind(Pin pin);
    void release(void);
};