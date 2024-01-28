#include "string.h"
#include "assert.h"
#include "msp.h"

extern UsartIf* debug_usart;

extern "C"
{
#include "stdio.h"
#include "unistd.h"
#include <sys/types.h>
#include <sys/stat.h>

    // refer to https://blog.csdn.net/CooCox_UP_Team/article/details/8465143

    extern int _end; // the heap region, defined in ld script

    caddr_t _sbrk(int incr)
    {
        static unsigned char *heap = NULL;
        unsigned char *prev_heap;
        if (heap == NULL)
        {
            heap = (unsigned char *)&_end;
        }
        prev_heap = heap;
        heap += incr;
        return (caddr_t)prev_heap;
    }

    int _close(int file)
    {
        return 0;
    }
    int _fstat(int file, struct stat *st)
    {
        st->st_mode = S_IFCHR; // char device
        return 0;
    }
    int _isatty(int file)
    {
        return 1; // terminal device
    }
    int _lseek(int file, int ptr, int dir)
    {
        return 0;
    }
    int _read(int file, char *ptr, int len)
    {
        return debug_usart->async_recv((uint8_t *)ptr, len);
        return 0;
    }
    int _write(int file, char *ptr, int len)
    {
        debug_usart->async_send((uint8_t *)ptr, len);
        return len;
    }
    void abort(void)
    {
        /* Abort called */
        while (1)
            ;
    }
}
