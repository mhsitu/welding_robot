#pragma once

#include "stackObject.h"

class CStackNumber : public CStackObject
{
public:
    CStackNumber(double n);
    virtual ~CStackNumber();

    std::string toString() const;

    CStackObject* copyYourself();

    float getFloatValue();
    int getIntValue();
    long getLongValue();
    double getValue();
    void setFloatValue(float n);
    void setIntValue(int n);
    void setLongValue(long n);
    void setValue(double n);

protected:
    double _value;
};
