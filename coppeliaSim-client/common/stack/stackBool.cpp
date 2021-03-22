#include "stackBool.h"
#include <sstream>

CStackBool::CStackBool(bool theValue)
{
    _objectType=STACK_BOOL;
    _value=theValue;
}

CStackBool::~CStackBool()
{
}

std::string CStackBool::toString() const
{
    std::string strVal = _value ? "true" : "false";
    return(strVal);
}

bool CStackBool::getValue()
{
    return(_value);
}

void CStackBool::setValue(bool theValue)
{
    _value=theValue;
}

CStackObject* CStackBool::copyYourself()
{
    CStackBool* retVal=new CStackBool(_value);
    return(retVal);
}
