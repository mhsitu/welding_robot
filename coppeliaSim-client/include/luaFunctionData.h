#pragma once

#include "luaFunctionDataItem.h"
#include "simLib.h"

#define SIM_LUA_ARG_NIL_ALLOWED (65536)

class CLuaFunctionData  
{
public:
    CLuaFunctionData();
    virtual ~CLuaFunctionData();

    //------------------------
    static void getInputDataForFunctionRegistration(const int* dat,std::vector<int>& outDat);
    bool readDataFromLua(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName);
    std::vector<CLuaFunctionDataItem>* getInDataPtr();
    void pushOutData(const CLuaFunctionDataItem& dataItem);
    void writeDataToLua(SLuaCallBack* p);
    //------------------------


    //------------------------
    void pushOutData_luaFunctionCall(const CLuaFunctionDataItem& dataItem);
    void writeDataToLua_luaFunctionCall(SLuaCallBack* p,const int* expectedArguments);
    bool readDataFromLua_luaFunctionCall(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName);
    std::vector<CLuaFunctionDataItem>* getOutDataPtr_luaFunctionCall();
    void releaseBuffers_luaFunctionCall(SLuaCallBack* p);
    //------------------------



protected:
    std::vector<CLuaFunctionDataItem> _inData;
    std::vector<CLuaFunctionDataItem> _outData;
};
