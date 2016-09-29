
#include "delayline.h"

delayline::delayline(unsigned int length)
{
    L = length;
    z = init_double(L);
    l = 0;
}

delayline::~delayline()
{
    free(z);
    z = NULL;
}

void
delayline::connect(double *signal)
{
    if (NULL == signal) err_msg(__FILE__, __LINE__, "Cannot connect signal to NULL pointer.");
    s = signal;
}

double
delayline::get(unsigned int index)
{
    if (index >= L) err_msg(__FILE__, __LINE__, "Warning: Index out of bounds in delayline."); // TODO: remove
    index = (index + l) % L;
    return z[index];
}

double
delayline::operator[](unsigned int index)
{
    if (index >= L) err_msg(__FILE__, __LINE__, "Warning: Index out of bounds in delayline."); // TODO: remove
    index = (index+l) % L;
    return z[index];
}

void
delayline::input(double sample)
{
    if (0 == l) l = L-1;
    else l--;
    z[l] = sample;
}

void
delayline::shift(void)
{
    if (0 == l) l = L-1;
    else l--;
    z[l] = *s;
}

void delayline::flush(void)
{
    l = 0; // reset index pointer
    for (unsigned int i = 0; i < L; i++)
        z[i] = .0; // clear values
}
