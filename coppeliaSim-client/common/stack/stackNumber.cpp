#include "stackNumber.h"
#include <sstream>

CStackNumber::CStackNumber(double n)
{
    _objectType=STACK_NUMBER;
    _value=n;
}

CStackNumber::~CStackNumber()
{
}

std::string CStackNumber::toString() const
{
    std::stringstream ss;
    ss << _value;
    return(ss.str());
}

float CStackNumber::getFloatValue()
{
    return((float)_value);
}

int CStackNumber::getIntValue()
{
    return((int)_value);
}

long CStackNumber::getLongValue()
{
    return((long)_value);
}

double CStackNumber::getValue()
{
    return(_value);
}

void CStackNumber::setFloatValue(float n)
{
    _value=(double)n;
}

void CStackNumber::setIntValue(int n)
{
    _value=(int)n;
}

void CStackNumber::setLongValue(long n)
{
    _value=(long)n;
}

void CStackNumber::setValue(double n)
{
    _value=n;
}

CStackObject* CStackNumber::copyYourself()
{
    CStackNumber* retVal=new CStackNumber(_value);
    return(retVal);
}
