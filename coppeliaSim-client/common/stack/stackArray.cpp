#include "stackArray.h"
#include "stackNull.h"
#include "stackNumber.h"
#include "stackBool.h"
#include "stackString.h"
#include "stackMap.h"
#include <sstream>

CStackArray::CStackArray()
{
    _objectType=STACK_ARRAY;
    _circularRef=false;
}

CStackArray::~CStackArray()
{
    for (size_t i=0;i<_objectValues.size();i++)
        delete _objectValues[i];
}

std::string CStackArray::toString() const
{
    if(_circularRef) return "...";
    std::stringstream ss;
    ss << "{";
    std::string sep = "";
    for(double num : _doubleValues)
    {
        ss << sep << num;
        sep = ", ";
    }
    for(CStackObject* obj : _objectValues)
    {
        ss << sep << obj->toString();
        sep = ", ";
    }
    ss << "}";
    return(ss.str());
}

void CStackArray::buildOntoStack(int stackId)
{
    simPopStackItem(stackId,0);
    if (_doubleValues.size()>0)
    {
       for (size_t i=0;i<_doubleValues.size();i++)
           simPushDoubleOntoStack(stackId,_doubleValues[i]);
    }
    else if (_objectValues.size()>0)
    {
        for (size_t i=0;i<_objectValues.size();i++)
            CStackObject::buildItemOntoStack(stackId,_objectValues[i]);
    }
}

bool CStackArray::buildFromStack(int stackId)
{
    if (_objectValues.size()+_doubleValues.size()+_intValues.size()==0)
    {
        int s=simGetStackSize(stackId);
        // Check first if we have only numbers:
        bool allNumbers=true;
        double dummyD;
        for (int i=0;i<s;i++)
        {
            simMoveStackItemToTop(stackId,0);
            if (1!=simGetStackDoubleValue(stackId,&dummyD))
                allNumbers=false;
        }
        // Now read out the stack:
        if (allNumbers)
        {
            _doubleValues.resize(s);
            for (int i=0;i<s;i++)
            {
                simMoveStackItemToTop(stackId,0);
                simGetStackDoubleValue(stackId,&dummyD);
                _doubleValues[i]=dummyD;
                simPopStackItem(stackId,1);
            }
        }
        else
        {
            for (int i=0;i<s;i++)
            {
                simMoveStackItemToTop(stackId,0);
                appendTopStackItem(stackId);
            }
        }
        return(true);
    }
    return(false);
}

void CStackArray::setCircularRef()
{
    _circularRef=true;
}

bool CStackArray::isCircularRef()
{
    return(_circularRef);
}

void CStackArray::appendTopStackItem(int stackId)
{ // this also clears the item from the stack
    if (_doubleValues.size()+_intValues.size()==0)
        _objectValues.push_back(CStackObject::buildItemFromTopStackPosition(stackId));
}

bool CStackArray::isNull(size_t index)
{
    if (index<_objectValues.size())
        return(_objectValues[index]->getObjectType()==STACK_NULL);
    return(false);
}

bool CStackArray::isBool(size_t index)
{
    if (index<_objectValues.size())
        return(_objectValues[index]->getObjectType()==STACK_BOOL);
    return(false);
}

bool CStackArray::isNumber(size_t index)
{
    if (index<_objectValues.size())
        return(_objectValues[index]->getObjectType()==STACK_NUMBER);
    return(index<_doubleValues.size());
}

bool CStackArray::isString(size_t index)
{
    if (index<_objectValues.size())
        return(_objectValues[index]->getObjectType()==STACK_STRING);
    return(false);
}

bool CStackArray::isArray(size_t index,size_t minSize/*=0*/)
{
    if (index<_objectValues.size())
    {
        if (_objectValues[index]->getObjectType()==STACK_ARRAY)
            return(((CStackArray*)_objectValues[index])->getSize()>=minSize);
    }
    return(false);
}

bool CStackArray::isMap(size_t index)
{
    if (index<_objectValues.size())
        return(_objectValues[index]->getObjectType()==STACK_MAP);
    return(false);
}

bool CStackArray::getBool(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_BOOL) )
        return(((CStackBool*)_objectValues[index])->getValue());
    return(false);
}

float CStackArray::getFloat(size_t index)
{
    return((float)getDouble(index));
}

double CStackArray::getDouble(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_NUMBER) )
        return(((CStackNumber*)_objectValues[index])->getValue());
    if (index<_doubleValues.size())
        return(_doubleValues[index]);
    return(0.0);
}

int CStackArray::getInt(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_NUMBER) )
        return((int)((CStackNumber*)_objectValues[index])->getValue());
    if (index<_doubleValues.size())
        return((int)_doubleValues[index]);
    return(0);
}

const double* CStackArray::getDoublePointer()
{
    if (_doubleValues.size()>0)
        return(&_doubleValues[0]);
    return(NULL);
}

const int* CStackArray::getIntPointer()
{
    if (_intValues.size()==0)
    {
        for (size_t i=0;i<_doubleValues.size();i++)
            _intValues.push_back(int(_doubleValues[i]));
    }
    if (_intValues.size()>0)
        return(&_intValues[0]);
    return(NULL);
}

std::string CStackArray::getString(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_STRING) )
        return(((CStackString*)_objectValues[index])->getValue());
    return("");
}

CStackArray* CStackArray::getArray(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_ARRAY) )
        return((CStackArray*)_objectValues[index]);
    return(NULL);
}

CStackMap* CStackArray::getMap(size_t index)
{
    if ( (index<_objectValues.size())&&(_objectValues[index]->getObjectType()==STACK_MAP) )
        return((CStackMap*)_objectValues[index]);
    return(NULL);
}




bool CStackArray::isNumberArray()
{
    return(_objectValues.size()==0);
}

size_t CStackArray::getSize()
{
    if (_objectValues.size()>0)
        return(_objectValues.size());
    return(_doubleValues.size());
}

const std::vector<CStackObject*>* CStackArray::getObjects()
{
    return(&_objectValues);
}

const std::vector<double>* CStackArray::getDoubles()
{
    return(&_doubleValues);
}

const std::vector<int>* CStackArray::getInts()
{
    if (_intValues.size()==0)
    {
        for (size_t i=0;i<_doubleValues.size();i++)
            _intValues.push_back(int(_doubleValues[i]));
    }
    return(&_intValues);
}

bool CStackArray::pushNull()
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackNull());
        return(true);
    }
    return(false);
}

bool CStackArray::pushBool(bool d)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackBool(d));
        return(true);
    }
    return(false);
}

bool CStackArray::pushFloat(float d)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackNumber(double(d)));
        return(true);
    }
    return(false);
}

bool CStackArray::pushDouble(double d)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackNumber(d));
        return(true);
    }
    return(false);
}

bool CStackArray::pushInt(int d)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackNumber((double)d));
        return(true);
    }
    return(false);
}

bool CStackArray::pushString(const std::string& d)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackString(d.c_str(),int(d.length())));
        return(true);
    }
    return(false);
}

bool CStackArray::pushString(const char* d,size_t bufferLength)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(new CStackString(d,int(bufferLength)));
        return(true);
    }
    return(false);
}

bool CStackArray::pushArray(CStackArray* arr)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(arr);
        return(true);
    }
    return(false);
}

bool CStackArray::pushMap(CStackMap* map)
{
    if (_doubleValues.size()+_intValues.size()==0)
    {
        _objectValues.push_back(map);
        return(true);
    }
    return(false);
}

bool CStackArray::setDoubleArray(const double* d,size_t l)
{
    if (_doubleValues.size()+_intValues.size()+_objectValues.size()==0)
    {
        for (size_t i=0;i<l;i++)
            _doubleValues.push_back(d[i]);
        return(true);
    }
    return(false);
}

bool CStackArray::setIntArray(const int* d,size_t l)
{
    if (_doubleValues.size()+_intValues.size()+_objectValues.size()==0)
    {
        for (size_t i=0;i<l;i++)
            _doubleValues.push_back((double)d[i]);
        return(true);
    }
    return(false);
}


CStackObject* CStackArray::copyYourself()
{
    CStackArray* retVal=new CStackArray();
    retVal->_intValues.assign(_intValues.begin(),_intValues.end());
    retVal->_doubleValues.assign(_doubleValues.begin(),_doubleValues.end());
    for (size_t i=0;i<_objectValues.size();i++)
        retVal->_objectValues.push_back(_objectValues[i]->copyYourself());
    return(retVal);
}

