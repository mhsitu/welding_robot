#pragma once

#include "stackObject.h"
#include <map>

class CStackArray;

class CStackMap : public CStackObject
{
public:
    CStackMap();
    virtual ~CStackMap();

    std::string toString() const;

    void appendTopStackItem(const char* key,int stackId);
    void appendTopStackItem(int key,int stackId);
    void appendTopStackItem(bool key,int stackId);

    CStackObject* copyYourself();

    void setNull(const char* key);
    void setBool(const char* key,bool d);
    void setFloat(const char* key,float d);
    void setDouble(const char* key,double d);
    void setInt(const char* key,int d);
    void setString(const char* key,const std::string& d);
    void setString(const char* key,const char* d,size_t bufferLength);
    void setArray(const char* key,CStackArray* arr);
    void setMap(const char* key,CStackMap* map);

    void setNull(int key);
    void setBool(int key,bool d);
    void setFloat(int key,float d);
    void setDouble(int key,double d);
    void setInt(int key,int d);
    void setString(int key,const std::string& d);
    void setString(int key,const char* d,size_t bufferLength);
    void setArray(int key,CStackArray* arr);
    void setMap(int key,CStackMap* map);

    void setNull(bool key);
    void setBool(bool key,bool d);
    void setFloat(bool key,float d);
    void setDouble(bool key,double d);
    void setInt(bool key,int d);
    void setString(bool key,const std::string& d);
    void setString(bool key,const char* d,size_t bufferLength);
    void setArray(bool key,CStackArray* arr);
    void setMap(bool key,CStackMap* map);

    bool isKeyPresent(const char* key);
    bool isNull(const char* key);
    bool isBool(const char* key);
    bool isNumber(const char* key);
    bool isString(const char* key);
    bool isArray(const char* key,size_t minSize=0);
    bool isMap(const char* key);

    bool isKeyPresent(int key);
    bool isNull(int key);
    bool isBool(int key);
    bool isNumber(int key);
    bool isString(int key);
    bool isArray(int key,size_t minSize=0);
    bool isMap(int key);

    bool isKeyPresent(bool key);
    bool isNull(bool key);
    bool isBool(bool key);
    bool isNumber(bool key);
    bool isString(bool key);
    bool isArray(bool key,size_t minSize=0);
    bool isMap(bool key);

    bool getBool(const char* key);
    float getFloat(const char* key);
    double getDouble(const char* key);
    int getInt(const char* key);
    std::string getString(const char* key);
    CStackArray* getArray(const char* key);
    CStackMap* getMap(const char* key);

    bool getBool(int key);
    float getFloat(int key);
    double getDouble(int key);
    int getInt(int key);
    std::string getString(int key);
    CStackArray* getArray(int key);
    CStackMap* getMap(int key);

    bool getBool(bool key);
    float getFloat(bool key);
    double getDouble(bool key);
    int getInt(bool key);
    std::string getString(bool key);
    CStackArray* getArray(bool key);
    CStackMap* getMap(bool key);

    bool contains(const char* key,int theType=-1,size_t theMinSizeIfArray=0,bool onlyNumbersInArray=false);
    bool contains(int key,int theType=-1,size_t theMinSizeIfArray=0,bool onlyNumbersInArray=false);
    bool contains(bool key,int theType=-1,size_t theMinSizeIfArray=0,bool onlyNumbersInArray=false);

    std::map<std::string,CStackObject*>* getKeyValuePairs();
    std::map<std::string,CStackObject*>* getKeyValuePairsKStr();
    std::map<int,CStackObject*>* getKeyValuePairsKInt();
    std::map<bool,CStackObject*>* getKeyValuePairsKBool();

protected:
    void _remove(const char* key);
    void _remove(int key);
    void _remove(bool key);

    std::map<std::string,CStackObject*> _objectValuesKStr;
    std::map<int,CStackObject*> _objectValuesKInt;
    std::map<bool,CStackObject*> _objectValuesKBool;
};
