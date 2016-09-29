#ifndef DELAYLINE_H
#define DELAYLINE_H

#include "../common/basic.h"
#include "../common/log_messages.h"

/* this is a classical ringbuffer, implemented
 * as an usual array with travelling indexpointer */

class delayline
{
    public:
        delayline(unsigned int length);         // init delay line with a cerain length
        ~delayline();
        void input(double sample);              // manually input next sample
        void connect(double *signal);     // connects the delayline to a signal variable
        void shift(void);                       // automatically clocks the new value from connected signal
        double get(unsigned int index);         // get value from a certain index
        double operator[](unsigned int index);   // same as before
        void flush(void);

    private:
        unsigned int L; // length of delayline
        unsigned int l; // index pointer
        double *z;      // delay line array
        double *s;
};

#endif // DELAYLINE_H
