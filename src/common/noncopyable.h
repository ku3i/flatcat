#ifndef NONCOPYABLE_H_INCLUDED
#define NONCOPYABLE_H_INCLUDED

class noncopyable
{
protected:
    noncopyable() {}
    //~noncopyable() {}
    virtual ~noncopyable() {}
private:
    noncopyable(const noncopyable&);
    const noncopyable& operator=(const noncopyable&);
};

#endif // NONCOPYABLE_H_INCLUDED
