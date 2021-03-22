#pragma once
#include "simLib.h"
#include <string>

enum {  STACK_NULL=0,
        STACK_NUMBER,
        STACK_BOOL,
        STACK_STRING,
        STACK_ARRAY,
        STACK_MAP
};

class CStackNull;
class CStackNumber;
class CStackBool;
class CStackString;
class CStackArray;
class CStackMap;

class CStackObject
{
public:
    CStackObject();
    virtual ~CStackObject();

    virtual CStackObject* copyYourself();

    int getObjectType() const;

    static void buildItemOntoStack(int stackId,CStackObject* obj);
    static CStackObject* buildItemFromTopStackPosition(int stackId);

    CStackNull* asNull();
    CStackNumber* asNumber();
    CStackBool* asBool();
    CStackString* asString();
    CStackArray* asArray();
    CStackMap* asMap();

    virtual std::string toString() const = 0;
    std::string getObjectTypeString() const;

protected:
    int _objectType;
};
