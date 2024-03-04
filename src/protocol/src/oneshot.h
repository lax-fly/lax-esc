#include "protocol.h"

class Oneshot : public Protocol
{
public:
    Oneshot();
    ~Oneshot();
    virtual void poll(void);
    virtual bool signal_lost(); // return if the input signal is lost or mismatch
    
    void bind(Pin pin);
    void release(void);
};