/**
  ******************************************************************************
  * Copyright (C) 2020 - ~, SCUT-RobotLab Development Team
  * @file    CoppelisSim.h
  * @author  Mentos Seetoo
  * @brief   Simple package based on 'Legacy remote API' of CoppeliaSim Robotics.
  * @version 1.3													 
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#pragma once
#include "extApi.h"
#include <string>
#include <vector>
#include <initializer_list>

/* Types define --------------------------------------------------------------*/
enum _clientStatus_Type
{
    Connect_Lost = 0U,
    Connected
};

enum _simObj_Type
{
    JOINT = 0,
    OTHER_OBJECT,
    VISION_SENSOR,
    FORCE_SENSOR,
};

enum _simSignal_Type
{
    SIM_FLOAT_SIGNAL,
    SIM_INTEGER_SIGNAL,
    SIM_STRING_SIGNAL
};

enum _simIO_Type
{
    CLIENT_RO = 1 << 4, /**< Read only*/
    CLIENT_WO = 2 << 4, /**< Write only*/
    CLIENT_RW = 3 << 4  /**< Read and Write*/
};

enum _simOP_Type
{
    SIM_POSITION = 0,
    SIM_QUATERNION,
    SIM_ORIENTATION,
    SIM_VELOCITY,
    SIM_FORCE,
    SIM_NORMAL_IMAGE,
    SIM_DEPTH_IMAGE,
    SIM_SIGNAL_OP      /**< For signal operation*/
};

/* Structure Definition ------------------------------------------------------*/
typedef struct
{
    simxInt operation;
    simxInt read_return_code;
    simxInt write_return_code;
} _simOPParam_Struct;

typedef struct
{
    simxFloat quaternion_4f[4],
        position_3f[3],
        orientation_3f[3],
        angVelocity_3f[3], //!< angle velocity(3-dimension)
        linVelocity_3f[3], //!< linear velocity(3-dimension)
        force_3f[3],       //!< force from force sensor(3-dimension)
        torque_3f[3],      //!< torque of force sensor (3-dimension)
        torque_f,          //!< torque of Joint
        angle_f,           //!< angle of joint
        angVelocity_f;     //!< angular velocity of joint
} _simOPData_Struct;

typedef struct
{
    std::string  obj_name;
    simxInt      obj_handle;
    _simObj_Type object_type;
    std::vector<_simOPParam_Struct> operation_list; //!< operation = _simOP_Type || _simIO_Type;
    _simOPData_Struct obj_Data = {0};               //!< Data to read
    _simOPData_Struct obj_Target = {0};             //!< Taeget to write
    simxUChar *RGB_buffer;                          //!< Normal image buffer of vision sensor
    simxFloat *depth_buffer;                        //!< Depth image buffer of vision sensor
    simxInt   resolutionRGB[2];                     //!< Resolution of RGB image
    simxInt   resolutionDepth[2];                   //!< Resolution of depth image
} _simObjectHandle_Type;

typedef struct
{
    std::string siganl_name;
    _simSignal_Type signal_type;
    std::vector<_simOPParam_Struct> operation_list; //!< operation = _simOP_Type || _simIO_Type;
    simxFloat data;
    simxFloat target;
    simxUChar *string_data;
    simxInt   string_len;
} _simSignalHandle_Type;

typedef struct
{
  std::string  collision_name;
  simxInt      collision_handle;
  simxUChar    collision_state;
}_simCollisHandle_Type;

/**
    @brief  Interface for simple coppeliaSIm client.
    @note   Meyers' Singleton is used.
*/
class CoppeliaSim_Client
{
public:
    static CoppeliaSim_Client &getInstance()
    {
        static CoppeliaSim_Client _instance;
        return _instance;
    }

    bool Start(const char* p_IP, int32_t p_conection_port, int commThreadCycleInMs, bool synchronousMode = false, bool multiClientMode = false);
    bool Start();
    bool Set_startParam(const char *p_IP, int32_t p_conection_port, int commThreadCycleInMs, bool synchronousMode = false, bool multiClientMode = false);
    _simObjectHandle_Type *Add_Object(std::string full_name, _simObj_Type type, std::initializer_list<simxInt> operation_ls);
    _simSignalHandle_Type *Add_Object(std::string full_name, _simSignal_Type type, std::initializer_list<simxInt> operation_ls);
    _simCollisHandle_Type* Add_Object(std::string full_name);
    bool Delete_Object(_simObjectHandle_Type *pObject);
    bool ComWithServer();
    _clientStatus_Type Is_Connected();
    uint32_t GetSimMilliSec() { return (uint32_t)simTick; }
    uint32_t GetSimMicroSec() { return (uint32_t)(simTick * 1000); }

private:
    CoppeliaSim_Client();
    ~CoppeliaSim_Client();
    std::vector<_simObjectHandle_Type *> Joint_list;
    std::vector<_simObjectHandle_Type *> ForceSensor_list;
    std::vector<_simObjectHandle_Type *> OtherObject_list;
    std::vector<_simObjectHandle_Type *> VisionSensor_list;
    std::vector<_simSignalHandle_Type *> Signal_list;
    std::vector<_simCollisHandle_Type *> Collision_list;
    int32_t clientID;
    char*   IP_addr;
    int32_t cycleInMs;
    int32_t port;
    bool    synchronous_mode;
    bool    multi_client_mode;
    int32_t ping_time;
    int32_t simTick;
    _clientStatus_Type state;
    _simSignalHandle_Type* iteration;
    _simOP_Type GetOP_Type(simxInt operation)
    {
        return (_simOP_Type)(operation & 0x0F);
    }

    _simIO_Type GetIO_Type(simxInt operation)
    {
        return (_simIO_Type)(operation & 0xF0);
    }
};

extern CoppeliaSim_Client* CoppeliaSim;
 /************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
