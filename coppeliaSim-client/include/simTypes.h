#if !defined(SIMTYPES_INCLUDED_)
#define SIMTYPES_INCLUDED_

// Various types used in the interface functions:
typedef unsigned char simBool;
typedef char simChar;
typedef int simInt;
typedef float simFloat;
typedef double simDouble;
typedef void simVoid;
typedef unsigned char simUChar;
typedef unsigned int simUInt;
typedef unsigned long long int simUInt64;

struct SScriptCallBack
{
    simInt objectID;
    simInt scriptID;
    simInt stackID;
    simChar waitUntilZero;
    simChar* raiseErrorWithMessage;
};

struct SShapeVizInfo
{
    simFloat* vertices;
    simInt verticesSize;
    simInt* indices;
    simInt indicesSize;
    simFloat shadingAngle;
    simFloat* normals;
    simFloat colors[9];
    simChar* texture; /*rgba*/
    simInt textureId;
    simInt textureRes[2];
    simFloat* textureCoords;
    simInt textureApplyMode;
    simInt textureOptions;
};

struct SLuaCallBack
{
    simInt objectID;
    simBool* inputBool;
    simInt* inputInt;
    simFloat* inputFloat;
    simChar* inputChar;
    simInt inputArgCount;
    simInt* inputArgTypeAndSize;
    simBool* outputBool;
    simInt* outputInt;
    simFloat* outputFloat;
    simChar* outputChar;
    simInt outputArgCount;
    simInt* outputArgTypeAndSize;
    simChar waitUntilZero;
    simChar* inputCharBuff;
    simChar* outputCharBuff;
    simInt scriptID;
    simDouble* inputDouble;
    simDouble* outputDouble;
};

typedef int (*contactCallback)(int,int,int,int*,float*);
typedef int (*jointCtrlCallback)(int,int,int,const int*,const float*,float*);

#endif // !defined(SIMTYPES_INCLUDED_)
