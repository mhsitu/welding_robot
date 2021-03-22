#include "stackObject.h"
#include "stackNull.h"
#include "stackNumber.h"
#include "stackBool.h"
#include "stackString.h"
#include "stackArray.h"
#include "stackMap.h"

CStackObject::CStackObject()
{
}

CStackObject::~CStackObject()
{
}

int CStackObject::getObjectType() const
{
    return(_objectType);
}

CStackObject* CStackObject::copyYourself()
{
    return(NULL);
}

void CStackObject::buildItemOntoStack(int stackId,CStackObject* obj)
{
    if (obj->getObjectType()==STACK_NULL)
        simPushNullOntoStack(stackId); // NULL
    else if (obj->getObjectType()==STACK_BOOL)
        simPushBoolOntoStack(stackId,((CStackBool*)obj)->getValue()); // Bool
    else if (obj->getObjectType()==STACK_NUMBER)
        simPushDoubleOntoStack(stackId,((CStackNumber*)obj)->getValue()); // number
    else if (obj->getObjectType()==STACK_STRING)
    { // string
        std::string str(((CStackString*)obj)->getValue());
        simPushStringOntoStack(stackId,str.c_str(),int(str.length()));
    }
    else if (obj->getObjectType()==STACK_ARRAY)
    {
        CStackArray* arr=(CStackArray*)obj;
        if (arr->getSize()==0)
            simPushTableOntoStack(stackId); // empty array
        else if (arr->getSize()>0)
        {
            if (arr->isNumberArray())
               simPushDoubleTableOntoStack(stackId,&arr->getDoubles()->at(0),int(arr->getDoubles()->size())); // number array
            else
            { // mixed array
                simPushTableOntoStack(stackId);
                const std::vector<CStackObject*>* objs=arr->getObjects();
                for (size_t i=0;i<objs->size();i++)
                {
                    simPushInt32OntoStack(stackId,int(i+1)); // the key
                    CStackObject::buildItemOntoStack(stackId,objs->at(i)); // the value
                    simInsertDataIntoStackTable(stackId);
                }
            }
        }
    }
    else if (obj->getObjectType()==STACK_MAP)
    {
        CStackMap* map=(CStackMap*)obj;
        std::map<std::string,CStackObject*>* keyValues=map->getKeyValuePairs();
        simPushTableOntoStack(stackId);
        for (std::map<std::string,CStackObject*>::iterator it=keyValues->begin();it!=keyValues->end();it++)
        {
            simPushStringOntoStack(stackId,it->first.c_str(),0); // the key
            CStackObject::buildItemOntoStack(stackId,it->second); // the value
            simInsertDataIntoStackTable(stackId);
        }
    }
}

CStackObject* CStackObject::buildItemFromTopStackPosition(int stackId)
{ // this also clears the item from the stack
    CStackObject* retVal=NULL;
    simBool bv;
    double dv;
    if (1==simIsStackValueNull(stackId))
    { // NULL
        retVal=new CStackNull();
        simPopStackItem(stackId,1);
    }
    else if (1==simGetStackBoolValue(stackId,&bv))
    { // bool
        retVal=new CStackBool(bv!=0);
        simPopStackItem(stackId,1);
    }
    else if (1==simGetStackDoubleValue(stackId,&dv))
    { // number
        retVal=new CStackNumber(dv);
        simPopStackItem(stackId,1);
    }
    else
    {
        int sl;
        char* str=simGetStackStringValue(stackId,&sl);
        if (str!=NULL)
        { // string
            retVal=new CStackString(str,sl);
            simReleaseBuffer(str);
            simPopStackItem(stackId,1);
        }
        else
        { // the item is not a NULL, bool, number or string. So it must be an array or a map:
            int s=simGetStackTableInfo(stackId,0);
            if (s==sim_stack_table_circular_ref)
            { // A map/array in a circular reference
                CStackArray* arr=new CStackArray();
                arr->setCircularRef();
                retVal=arr;
                simPopStackItem(stackId,1);
            }
            else if (s==sim_stack_table_empty)
            { // Empty Array
                retVal=new CStackArray();
                simPopStackItem(stackId,1);
            }
            else if (s>0)
            { // Array
                if (1==simGetStackTableInfo(stackId,2))
                { // Array of numbers:
                    CStackArray* arr=new CStackArray();
                    std::vector<double> numbers;
                    numbers.resize(s);
                    simGetStackDoubleTable(stackId,&numbers[0],s);
                    arr->setDoubleArray(&numbers[0],s);
                    retVal=arr;
                    simPopStackItem(stackId,1);
                }
                else
                { // Array of items that are not all numbers
                    CStackArray* arr=new CStackArray();
                    int oi=simGetStackSize(stackId)-1;
                    simUnfoldStackTable(stackId); // This puts value-key pairs onto the stack
                    s=(simGetStackSize(stackId)-oi)/2;
                    for (int i=0;i<s;i++)
                    {
                        simMoveStackItemToTop(stackId,oi); // that's the key
                        simPopStackItem(stackId,1);
                        simMoveStackItemToTop(stackId,oi); // that's the value
                        arr->appendTopStackItem(stackId); // will also pop it
                    }
                    retVal=arr;
                }
            }
            else if (s==sim_stack_table_map)
            { // Map
                CStackMap* map=new CStackMap();
                int oi=simGetStackSize(stackId)-1;
                simUnfoldStackTable(stackId); // This puts value-key pairs onto the stack
                s=(simGetStackSize(stackId)-oi)/2;
                for (int i=0;i<s;i++)
                {
                    simMoveStackItemToTop(stackId,oi); // that's the key
                    char* str=simGetStackStringValue(stackId,NULL);
                    bool isbool=simGetStackBoolValue(stackId,&bv)==1;
                    bool isnum=simGetStackDoubleValue(stackId,&dv)==1;
                    simPopStackItem(stackId,1);
                    simMoveStackItemToTop(stackId,oi); // that's the value
                    if(isnum)
                        map->appendTopStackItem(int(dv),stackId); // will also pop it
                    else if(isbool)
                        map->appendTopStackItem(bv,stackId); // will also pop it
                    else
                        map->appendTopStackItem(str,stackId); // will also pop it
                    if(str) simReleaseBuffer(str);
                }
                retVal=map;
            }
        }
    }
    return(retVal);
}

CStackNull* CStackObject::asNull()
{
    if(_objectType==STACK_NULL) return(static_cast<CStackNull*>(this));
    else return(NULL);
}

CStackNumber* CStackObject::asNumber()
{
    if(_objectType==STACK_NUMBER) return(static_cast<CStackNumber*>(this));
    else return(NULL);
}

CStackBool* CStackObject::asBool()
{
    if(_objectType==STACK_BOOL) return(static_cast<CStackBool*>(this));
    else return(NULL);
}

CStackString* CStackObject::asString()
{
    if(_objectType==STACK_STRING) return(static_cast<CStackString*>(this));
    else return(NULL);
}

CStackArray* CStackObject::asArray()
{
    if(_objectType==STACK_ARRAY) return(static_cast<CStackArray*>(this));
    else return(NULL);
}

CStackMap* CStackObject::asMap()
{
    if(_objectType==STACK_MAP) return(static_cast<CStackMap*>(this));
    else return(NULL);
}

std::string CStackObject::getObjectTypeString() const
{
    switch(_objectType)
    {
    case STACK_NULL: return "null";
    case STACK_NUMBER: return "number";
    case STACK_BOOL: return "bool";
    case STACK_STRING: return "string";
    case STACK_ARRAY: return "array";
    case STACK_MAP: return "map";
    default: return "object";
    }
}

