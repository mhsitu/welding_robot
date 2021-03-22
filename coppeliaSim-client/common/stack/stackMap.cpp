#include "stackMap.h"
#include "stackArray.h"
#include "stackNull.h"
#include "stackNumber.h"
#include "stackBool.h"
#include "stackString.h"
#include <sstream>

CStackMap::CStackMap()
{
    _objectType=STACK_MAP;
}

CStackMap::~CStackMap()
{
    for (std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.begin();it!=_objectValuesKStr.end();it++)
        delete it->second;
    for (std::map<int,CStackObject*>::iterator it=_objectValuesKInt.begin();it!=_objectValuesKInt.end();it++)
        delete it->second;
    for (std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.begin();it!=_objectValuesKBool.end();it++)
        delete it->second;
}

std::string CStackMap::toString() const
{
    std::stringstream ss;
    ss << "{";
    std::string sep = "";
    for(const auto &item : _objectValuesKInt)
    {
        ss << sep << item.first << "=" << item.second->toString();
        sep = ", ";
    }
    for(const auto &item : _objectValuesKBool)
    {
        ss << sep << item.first << "=" << item.second->toString();
        sep = ", ";
    }
    for(const auto &item : _objectValuesKStr)
    {
        ss << sep << item.first << "=" << item.second->toString();
        sep = ", ";
    }
    ss << "}";
    return(ss.str());
}

void CStackMap::_remove(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
    {
        delete it->second;
        _objectValuesKStr.erase(it);
    }
}

void CStackMap::_remove(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
    {
        delete it->second;
        _objectValuesKInt.erase(it);
    }
}

void CStackMap::_remove(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
    {
        delete it->second;
        _objectValuesKBool.erase(it);
    }
}

void CStackMap::appendTopStackItem(const char* key,int stackId)
{ // this also clears the item from the stack
    _remove(key);
    _objectValuesKStr[key]=CStackObject::buildItemFromTopStackPosition(stackId);
}

void CStackMap::appendTopStackItem(int key,int stackId)
{ // this also clears the item from the stack
    _remove(key);
    _objectValuesKInt[key]=CStackObject::buildItemFromTopStackPosition(stackId);
}

void CStackMap::appendTopStackItem(bool key,int stackId)
{ // this also clears the item from the stack
    _remove(key);
    _objectValuesKBool[key]=CStackObject::buildItemFromTopStackPosition(stackId);
}

std::map<std::string,CStackObject*>* CStackMap::getKeyValuePairs()
{
    return(&_objectValuesKStr);
}

std::map<std::string,CStackObject*>* CStackMap::getKeyValuePairsKStr()
{
    return(&_objectValuesKStr);
}

std::map<int,CStackObject*>* CStackMap::getKeyValuePairsKInt()
{
    return(&_objectValuesKInt);
}

std::map<bool,CStackObject*>* CStackMap::getKeyValuePairsKBool()
{
    return(&_objectValuesKBool);
}

bool CStackMap::isKeyPresent(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    return(it!=_objectValuesKStr.end());
}

bool CStackMap::isKeyPresent(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    return(it!=_objectValuesKInt.end());
}

bool CStackMap::isKeyPresent(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    return(it!=_objectValuesKBool.end());
}

bool CStackMap::isNull(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
        return(it->second->getObjectType()==STACK_NULL);
    return(false);
}

bool CStackMap::isBool(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
        return(it->second->getObjectType()==STACK_BOOL);
    return(false);
}

bool CStackMap::isNumber(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
        return(it->second->getObjectType()==STACK_NUMBER);
    return(false);
}

bool CStackMap::isString(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
        return(it->second->getObjectType()==STACK_STRING);
    return(false);
}

bool CStackMap::isArray(const char* key,size_t minSize/*=0*/)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
    {
        if (it->second->getObjectType()==STACK_ARRAY)
            return(((CStackArray*)it->second)->getSize()>=minSize);
    }
    return(false);
}

bool CStackMap::isMap(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if (it!=_objectValuesKStr.end())
        return(it->second->getObjectType()==STACK_MAP);
    return(false);
}

bool CStackMap::isNull(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
        return(it->second->getObjectType()==STACK_NULL);
    return(false);
}

bool CStackMap::isBool(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
        return(it->second->getObjectType()==STACK_BOOL);
    return(false);
}

bool CStackMap::isNumber(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
        return(it->second->getObjectType()==STACK_NUMBER);
    return(false);
}

bool CStackMap::isString(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
        return(it->second->getObjectType()==STACK_STRING);
    return(false);
}

bool CStackMap::isArray(int key,size_t minSize/*=0*/)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
    {
        if (it->second->getObjectType()==STACK_ARRAY)
            return(((CStackArray*)it->second)->getSize()>=minSize);
    }
    return(false);
}

bool CStackMap::isMap(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if (it!=_objectValuesKInt.end())
        return(it->second->getObjectType()==STACK_MAP);
    return(false);
}

bool CStackMap::isNull(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
        return(it->second->getObjectType()==STACK_NULL);
    return(false);
}

bool CStackMap::isBool(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
        return(it->second->getObjectType()==STACK_BOOL);
    return(false);
}

bool CStackMap::isNumber(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
        return(it->second->getObjectType()==STACK_NUMBER);
    return(false);
}

bool CStackMap::isString(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
        return(it->second->getObjectType()==STACK_STRING);
    return(false);
}

bool CStackMap::isArray(bool key,size_t minSize/*=0*/)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
    {
        if (it->second->getObjectType()==STACK_ARRAY)
            return(((CStackArray*)it->second)->getSize()>=minSize);
    }
    return(false);
}

bool CStackMap::isMap(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if (it!=_objectValuesKBool.end())
        return(it->second->getObjectType()==STACK_MAP);
    return(false);
}

bool CStackMap::getBool(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_BOOL) )
        return(((CStackBool*)it->second)->getValue());
    return(false);
}

float CStackMap::getFloat(const char* key)
{
    return((float)getDouble(key));
}

double CStackMap::getDouble(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(((CStackNumber*)it->second)->getValue());
    return(0.0);
}

int CStackMap::getInt(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(int(((CStackNumber*)it->second)->getValue()));
    return(0);
}

std::string CStackMap::getString(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_STRING) )
        return(((CStackString*)it->second)->getValue());
    return("");
}

CStackArray* CStackMap::getArray(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_ARRAY) )
        return((CStackArray*)it->second);
    return(NULL);
}

CStackMap* CStackMap::getMap(const char* key)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(it->second->getObjectType()==STACK_MAP) )
        return((CStackMap*)it->second);
    return(NULL);
}

bool CStackMap::getBool(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_BOOL) )
        return(((CStackBool*)it->second)->getValue());
    return(false);
}

float CStackMap::getFloat(int key)
{
    return((float)getDouble(key));
}

double CStackMap::getDouble(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(((CStackNumber*)it->second)->getValue());
    return(0.0);
}

int CStackMap::getInt(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(int(((CStackNumber*)it->second)->getValue()));
    return(0);
}

std::string CStackMap::getString(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_STRING) )
        return(((CStackString*)it->second)->getValue());
    return("");
}

CStackArray* CStackMap::getArray(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_ARRAY) )
        return((CStackArray*)it->second);
    return(NULL);
}

CStackMap* CStackMap::getMap(int key)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(it->second->getObjectType()==STACK_MAP) )
        return((CStackMap*)it->second);
    return(NULL);
}

bool CStackMap::getBool(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_BOOL) )
        return(((CStackBool*)it->second)->getValue());
    return(false);
}

float CStackMap::getFloat(bool key)
{
    return((float)getDouble(key));
}

double CStackMap::getDouble(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(((CStackNumber*)it->second)->getValue());
    return(0.0);
}

int CStackMap::getInt(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_NUMBER) )
        return(int(((CStackNumber*)it->second)->getValue()));
    return(0);
}

std::string CStackMap::getString(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_STRING) )
        return(((CStackString*)it->second)->getValue());
    return("");
}

CStackArray* CStackMap::getArray(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_ARRAY) )
        return((CStackArray*)it->second);
    return(NULL);
}

CStackMap* CStackMap::getMap(bool key)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(it->second->getObjectType()==STACK_MAP) )
        return((CStackMap*)it->second);
    return(NULL);
}

bool CStackMap::contains(const char* key,int theType/*=-1*/,size_t theMinSizeIfArray/*=0*/,bool onlyNumbersInArray/*=false*/)
{
    std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.find(key);
    if ( (it!=_objectValuesKStr.end())&&(theType==-1||it->second->getObjectType()==theType) )
    {
        if (theType!=STACK_ARRAY)
            return(true);
        CStackArray* arr=(CStackArray*)it->second;
        if (arr->getSize()>=theMinSizeIfArray)
            return((!onlyNumbersInArray)||arr->isNumberArray());
    }
    return(false);
}

bool CStackMap::contains(int key,int theType/*=-1*/,size_t theMinSizeIfArray/*=0*/,bool onlyNumbersInArray/*=false*/)
{
    std::map<int,CStackObject*>::iterator it=_objectValuesKInt.find(key);
    if ( (it!=_objectValuesKInt.end())&&(theType==-1||it->second->getObjectType()==theType) )
    {
        if (theType!=STACK_ARRAY)
            return(true);
        CStackArray* arr=(CStackArray*)it->second;
        if (arr->getSize()>=theMinSizeIfArray)
            return((!onlyNumbersInArray)||arr->isNumberArray());
    }
    return(false);
}

bool CStackMap::contains(bool key,int theType/*=-1*/,size_t theMinSizeIfArray/*=0*/,bool onlyNumbersInArray/*=false*/)
{
    std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.find(key);
    if ( (it!=_objectValuesKBool.end())&&(theType==-1||it->second->getObjectType()==theType) )
    {
        if (theType!=STACK_ARRAY)
            return(true);
        CStackArray* arr=(CStackArray*)it->second;
        if (arr->getSize()>=theMinSizeIfArray)
            return((!onlyNumbersInArray)||arr->isNumberArray());
    }
    return(false);
}

void CStackMap::setNull(const char* key)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackNull();
}

void CStackMap::setBool(const char* key,bool d)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackBool(d);
}

void CStackMap::setFloat(const char* key,float d)
{
    setDouble(key,(double)d);
}

void CStackMap::setDouble(const char* key,double d)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackNumber(d);
}

void CStackMap::setInt(const char* key,int d)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackNumber(double(d));
}

void CStackMap::setString(const char* key,const std::string& d)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackString(d.c_str(),int(d.length()));
}

void CStackMap::setString(const char* key,const char* d,size_t bufferLength)
{
    _remove(key);
    _objectValuesKStr[key]=new CStackString(d,int(bufferLength));
}

void CStackMap::setArray(const char* key,CStackArray* arr)
{
    _remove(key);
    _objectValuesKStr[key]=arr;
}

void CStackMap::setMap(const char* key,CStackMap* map)
{
    _remove(key);
    _objectValuesKStr[key]=map;
}

void CStackMap::setNull(int key)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackNull();
}

void CStackMap::setBool(int key,bool d)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackBool(d);
}

void CStackMap::setFloat(int key,float d)
{
    setDouble(key,(double)d);
}

void CStackMap::setDouble(int key,double d)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackNumber(d);
}

void CStackMap::setInt(int key,int d)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackNumber(double(d));
}

void CStackMap::setString(int key,const std::string& d)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackString(d.c_str(),int(d.length()));
}

void CStackMap::setString(int key,const char* d,size_t bufferLength)
{
    _remove(key);
    _objectValuesKInt[key]=new CStackString(d,int(bufferLength));
}

void CStackMap::setArray(int key,CStackArray* arr)
{
    _remove(key);
    _objectValuesKInt[key]=arr;
}

void CStackMap::setMap(int key,CStackMap* map)
{
    _remove(key);
    _objectValuesKInt[key]=map;
}

void CStackMap::setNull(bool key)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackNull();
}

void CStackMap::setBool(bool key,bool d)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackBool(d);
}

void CStackMap::setFloat(bool key,float d)
{
    setDouble(key,(double)d);
}

void CStackMap::setDouble(bool key,double d)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackNumber(d);
}

void CStackMap::setInt(bool key,int d)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackNumber(double(d));
}

void CStackMap::setString(bool key,const std::string& d)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackString(d.c_str(),int(d.length()));
}

void CStackMap::setString(bool key,const char* d,size_t bufferLength)
{
    _remove(key);
    _objectValuesKBool[key]=new CStackString(d,int(bufferLength));
}

void CStackMap::setArray(bool key,CStackArray* arr)
{
    _remove(key);
    _objectValuesKBool[key]=arr;
}

void CStackMap::setMap(bool key,CStackMap* map)
{
    _remove(key);
    _objectValuesKBool[key]=map;
}

CStackObject* CStackMap::copyYourself()
{
    CStackMap* retVal=new CStackMap();
    for (std::map<std::string,CStackObject*>::iterator it=_objectValuesKStr.begin();it!=_objectValuesKStr.end();it++)
        retVal->_objectValuesKStr[it->first]=it->second->copyYourself();
    for (std::map<int,CStackObject*>::iterator it=_objectValuesKInt.begin();it!=_objectValuesKInt.end();it++)
        retVal->_objectValuesKInt[it->first]=it->second->copyYourself();
    for (std::map<bool,CStackObject*>::iterator it=_objectValuesKBool.begin();it!=_objectValuesKBool.end();it++)
        retVal->_objectValuesKBool[it->first]=it->second->copyYourself();
    return(retVal);
}

