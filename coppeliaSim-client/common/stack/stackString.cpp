#include "stackString.h"
#include <sstream>

CStackString::CStackString(const char* str,int l)
{
    _objectType=STACK_STRING;
    if (l>0)
        _value.assign(str,str+l);
}

CStackString::~CStackString()
{
}

std::string CStackString::toString() const
{
    std::stringstream ss;
    ss << "\"";
    for(size_t i = 0; i < _value.size(); ++i)
    {
        char c = _value[i];
        if(c == '"') ss << "\\\"";
        else if(c == '\\') ss << "\\\\";
        else if(c == '\n') ss << "\\n";
        else if(c == '\r') ss << "\\r";
        else if(c == '\t') ss << "\\t";
        else ss << c;
    }
    ss << "\"";
    return(ss.str());
}

std::string CStackString::getValue()
{
    return(_value);
}

void CStackString::setValue(const char* str,int l)
{
    if (l>0)
        _value.assign(str,str+l);
    else
        _value.clear();
}

CStackObject* CStackString::copyYourself()
{
    CStackString* retVal=new CStackString(_value.c_str(),int(_value.length()));
    return(retVal);
}
