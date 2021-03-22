#pragma once

#include <vector>
#include "stackObject.h"

class CStackMap;

class CStackArray : public CStackObject
{
public:
    CStackArray();
    virtual ~CStackArray();

    std::string toString() const;

    bool buildFromStack(int stackId);
    void buildOntoStack(int stackId);

    void appendTopStackItem(int stackId);

    CStackObject* copyYourself();

    bool pushNull();
    bool pushBool(bool d);
    bool pushFloat(float d);
    bool pushDouble(double d);
    bool pushInt(int d);
    bool pushString(const std::string& d);
    bool pushString(const char* d,size_t bufferLength);
    bool pushArray(CStackArray* arr);
    bool pushMap(CStackMap* map);
    bool setDoubleArray(const double* d,size_t l);
    bool setIntArray(const int* d,size_t l);

    bool isNumberArray();
    size_t getSize();

    bool isNull(size_t index);
    bool isBool(size_t index);
    bool isNumber(size_t index);
    bool isString(size_t index);
    bool isArray(size_t index,size_t minSize=0);
    bool isMap(size_t index);
    
    void setCircularRef();
    bool isCircularRef();

    bool getBool(size_t index);
    float getFloat(size_t index);
    double getDouble(size_t index);
    int getInt(size_t index);
    std::string getString(size_t index);
    CStackArray* getArray(size_t index);
    CStackMap* getMap(size_t index);

    const std::vector<CStackObject*>* getObjects();
    const std::vector<double>* getDoubles();
    const std::vector<int>* getInts();
    const double* getDoublePointer();
    const int* getIntPointer();

protected:
    std::vector<CStackObject*> _objectValues;
    std::vector<double> _doubleValues;
    std::vector<int> _intValues;
    bool _circularRef;
};
