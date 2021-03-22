/**
  ******************************************************************************
  * Copyright (C) 2020 - ~, SCUT-RobotLab Development Team
  * @file    CoppelisSim.cpp
  * @author  Sigma Chen hyper_chen@163.com
  * @brief   Simple package based on 'Legacy remote API' of CoppeliaSim Robotics.
  * 		 This file provide a class for user to easily communicate with 
  * 		 CoppeliaSim simulation server side. If you don't find a suitable 
  * 		 function for your project in this file, please refer to manual:
  * 		 https://www.coppeliarobotics.com/helpFiles/en/remoteApiOverview.htm
  * @version 1.3
  * @par Change Log：
  * <table>
  * <tr><th>Date        	<th>Version  <th>Author    		  <th>Description
  * <tr><td>2020-04-07   	<td> 1.0     <td>Mentos Seetoo    <td>Creator
  * <tr><td>2020-04-17   	<td> 1.1     <td>Sigma			  <td>Add collision operation.
  * 															  Add multi-client.
  * <tr><td>2020-05-10   	<td> 1.2     <td>Mentos Seetoo	  <td>Fixed bug in reading images.
  * <tr><td>2020-05-17   	<td> 1.3     <td>Sigma			  <td>Add simulation timestamp function.
  * </table>  
  * 
  ==============================================================================
                               How to use this Lib
  ==============================================================================
    @note
		- Before all your operations, you need to call `Start()`.
		- When the connection is established, you can use `Add_Obj()` to add operations
			that attatched to an boject, and receive a handle of this object, e.g.
			`Body = CoppeliaSim->Add_Object("chassis_respondable", OTHER_OBJECT, 
			{ SIM_ORIENTATION | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });`
		- Call `comWithServer()` periodly to automatically do all operations that
			you have added.
      	- Detailed using instruction please press Seetoo.
    @attention
		- At least C++11 is required.	
		- Singleton pogram pattern is used, please make sure there is only one 
		  client in one process.
		//TODO Dynamically add variables.	
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
#include "CoppeliaSim.h"
#include <string.h>
#include <cassert>
/* Private defines -----------------------------------------------------------*/
CoppeliaSim_Client *CoppeliaSim = &CoppeliaSim_Client::getInstance();

/* Founctions ----------------------------------------------------------------*/
/**
* @brief  Config object param.
*/
CoppeliaSim_Client::CoppeliaSim_Client()
{
	/*Do nothing now*/
}

/**
* @brief  Start simulation in specificed mode.
* @param  p_IP:               the ip address of the server.
* @param  p_conection_port:   the port number where to connect.
*                             Specify a negative port number in order to use shared
*                             memory, instead of socket communication.
* @param  commThreadCycleInMs:indicates how often data packets are sent back and forth.
*                             A default value of 5 is recommended.
* @param  synchronousMode:If this value is set 1, synchronous mode will be enabled.
* @param  multiClientMode:If this value is set 1, synchronous mode of multi-client will be enabled.
* @note   To view more "Operation modes" please visit:
*         http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#operationModes
* @retval true: start simulation successfully.
* @retval false: start failed.
*/
bool CoppeliaSim_Client::Start(const char *p_IP, int32_t p_conection_port, int commThreadCycleInMs, bool synchronousMode, bool multiClientMode)
{
	/*Save parameters*/
	static char *temp_IP = new char[strlen(p_IP)];
	strcpy(temp_IP, p_IP);
	IP_addr = temp_IP;
	port = p_conection_port;
	cycleInMs = commThreadCycleInMs;
	synchronous_mode = synchronousMode;
	multi_client_mode = multiClientMode;

	if (state == Connect_Lost)
	{
		/* Initialize simulation */
		clientID = simxStart(p_IP, p_conection_port, true, true, 2000, commThreadCycleInMs);

		if (clientID != -1)
		{
			if (synchronous_mode)
				simxSynchronous(clientID, true);

			simxStartSimulation(clientID, simx_opmode_oneshot);
			if (synchronous_mode && multi_client_mode)
				iteration = CoppeliaSim->Add_Object("iteration", SIM_INTEGER_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});

			state = Connected;
			return true;
		}
		else
		{
			state = Connect_Lost;
			return false;
		}
	}
	else
		return true;
}

//TODO Independently start function need test.
bool CoppeliaSim_Client::Start()
{
	assert(IP_addr != NULL);
	assert(port != 0);
	assert(cycleInMs != 0);

	if (state == Connect_Lost)
	{
		/* Reconnect */
		clientID = simxStart(IP_addr, port, true, true, 5000, cycleInMs);
		return (bool)(clientID != -1);
	}
	else
		return true;
}

bool CoppeliaSim_Client::Set_startParam(const char *p_IP, int32_t p_conection_port, int commThreadCycleInMs, bool synchronousMode, bool multiClientMode)
{
	if (p_IP == NULL || p_conection_port == 0 || commThreadCycleInMs == 0)
		return false;

	static char *temp_IP = new char[strlen(p_IP)];
	strcpy(temp_IP, p_IP);
	IP_addr = temp_IP;
	port = p_conection_port;
	cycleInMs = commThreadCycleInMs;
	synchronous_mode = synchronousMode;
	multi_client_mode = multiClientMode;
	return true;
}

/**
* @brief  Functions to add objects(Joint, Vision sensor, Force sensor, Other Object in coppeliasim, Siganl).
*         What object means is different from coppeliasim in this module. The "object" is an abstract
*         structure of data in programming.
* @note   We don't check the parameters in this version, including whether the object name is repeated,
*         or the operation list is valid and so on.
* @param
* @return _simObjectHandle_Type* \ _simSignalHandle_Type* \ _simCollisHandle_Type*
* @author Mentos Seetoo
*/
_simObjectHandle_Type *CoppeliaSim_Client::Add_Object(std::string full_name, _simObj_Type type, std::initializer_list<simxInt> operation_ls)
{
	_simObjectHandle_Type *p_Obj = new _simObjectHandle_Type;

	/*Check the memory*/
	if (p_Obj == NULL)
		return p_Obj;

	/*Get object handle*/
	if (simxGetObjectHandle(clientID, full_name.data(), &p_Obj->obj_handle, simx_opmode_blocking) == simx_return_ok)
	{
		p_Obj->obj_name = full_name;

		/*Set the type.*/
		p_Obj->object_type = type;
		switch (type)
		{
		case JOINT:
			Joint_list.push_back(p_Obj);
			break;
		case FORCE_SENSOR:
			ForceSensor_list.push_back(p_Obj);
			break;
		case OTHER_OBJECT:
			OtherObject_list.push_back(p_Obj);
			break;
		case VISION_SENSOR:
			VisionSensor_list.push_back(p_Obj);
			break;
		}
		/*Set the operations.*/
		for (auto &&operation : operation_ls)
		{
			_simOPParam_Struct *p_OP = new _simOPParam_Struct{0};
			p_OP->operation = operation;
			p_Obj->operation_list.push_back(*p_OP);
			delete p_OP;
		}
	}
	else
	{
		/*Failed to get the object handle */
		delete p_Obj;
		p_Obj = NULL;
	}

	return p_Obj;
}

_simSignalHandle_Type *CoppeliaSim_Client::Add_Object(std::string full_name, _simSignal_Type type, std::initializer_list<simxInt> operation_ls)
{
	_simSignalHandle_Type *p_Signal = new _simSignalHandle_Type;

	/*Check the memory*/
	if (p_Signal == NULL)
		return p_Signal;

	p_Signal->siganl_name = full_name;
	p_Signal->signal_type = type;
	Signal_list.push_back(p_Signal);
	/*Set the operations.*/
	for (auto &operation : operation_ls)
	{
		_simOPParam_Struct *p_OP = new _simOPParam_Struct{0};
		p_OP->operation = operation;
		p_Signal->operation_list.push_back(*p_OP);
		delete p_OP;
	}

	return p_Signal;
}

_simCollisHandle_Type *CoppeliaSim_Client::Add_Object(std::string full_name)
{
	_simCollisHandle_Type *p_Collision = new _simCollisHandle_Type;

	/*Check the memory*/
	if (p_Collision == NULL)
		return p_Collision;

	/*Get collision handle*/
	if (simxGetCollisionHandle(clientID, full_name.data(), &p_Collision->collision_handle, simx_opmode_blocking) == simx_return_ok)
	{
		p_Collision->collision_name = full_name;
		Collision_list.push_back(p_Collision);
	}
	else
	{
		/*Failed to get the collision handle */
		delete p_Collision;
		p_Collision = NULL;
	}

	return p_Collision;
}

/**
* @brief  Functions to delete objects.
* @param  pObject: pointer of the object that will be deleted
* @return 1:operated.
*        		    0:aborted.
* @author Mentos Seetoo
*/
bool CoppeliaSim_Client::Delete_Object(_simObjectHandle_Type *pObject)
{
	/*Temporarily not doing anything*/
	return false;
}

_clientStatus_Type CoppeliaSim_Client::Is_Connected()
{
	int32_t res = simxGetPingTime(clientID, &ping_time);
	if (ping_time >= 2000 || ping_time == 0)
		state = Connect_Lost;
	else
		state = Connected;
	return state;
}

bool CoppeliaSim_Client::ComWithServer()
{
	/* Communication is established */
	if (clientID != -1)
	{
		/*Synchronous mode is enabled*/
		if (synchronous_mode)
		{
			if (multi_client_mode)
			{
				static simxInt iteration_temp;
				if (simxGetIntegerSignal(clientID, iteration->siganl_name.data(), &iteration_temp, simx_opmode_oneshot) == simx_return_ok)
					iteration->data = (simxFloat)iteration_temp;
				else
					iteration->data = (simxFloat)-1;
				simxSynchronousTrigger(clientID);
				while (iteration->data == (simxFloat)iteration_temp)
					if (simxGetIntegerSignal(clientID, iteration->siganl_name.data(), &iteration_temp, simx_opmode_oneshot) != simx_return_ok)
						iteration_temp = -1;
			}
			else
			{
				simxSynchronousTrigger(clientID);
				simxGetPingTime(clientID, &ping_time);
			}
			simTick = simxGetLastCmdTime(clientID);
		}
		/*Iterate all Joints*/
		for (auto &Joint : Joint_list)
		{
			for (auto &OP_Obj : Joint->operation_list)
			{
				/*Read operation*/
				if (GetIO_Type(OP_Obj.operation) != CLIENT_WO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_POSITION:
						OP_Obj.read_return_code = simxGetJointPosition(clientID, Joint->obj_handle,
																	   &Joint->obj_Data.angle_f, simx_opmode_streaming);
						break;
					case SIM_FORCE:
						OP_Obj.read_return_code = simxGetJointForce(clientID, Joint->obj_handle,
																	&Joint->obj_Data.torque_f, simx_opmode_streaming);
						break;
					default:
						break;
					}
				}
				/*Write operation*/
				if (GetIO_Type(OP_Obj.operation) != CLIENT_RO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_POSITION:
						OP_Obj.write_return_code = simxSetJointTargetPosition(clientID, Joint->obj_handle,
																			  Joint->obj_Target.angle_f, simx_opmode_oneshot);
						break;
					case SIM_VELOCITY:
						OP_Obj.write_return_code = simxSetJointTargetVelocity(clientID, Joint->obj_handle,
																			  Joint->obj_Target.angVelocity_f, simx_opmode_oneshot);
						break;
					case SIM_FORCE:
						OP_Obj.write_return_code = simxSetJointForce(clientID, Joint->obj_handle,
																	 Joint->obj_Target.torque_f, simx_opmode_oneshot);
					default:
						break;
					}
				}

				if ((OP_Obj.read_return_code | OP_Obj.write_return_code & 0x02) == 1)
				{
					/*Do nothing*/
				}
			}
		}

		/*Iterate all other_objects*/
		for (auto &OtherObject : OtherObject_list)
		{
			for (auto &OP_Obj : OtherObject->operation_list)
			{
				/*Read operation*/
				if (GetIO_Type(OP_Obj.operation) != CLIENT_WO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_POSITION:
						/*Absolute position*/
						OP_Obj.read_return_code = simxGetObjectPosition(clientID, OtherObject->obj_handle,
																		-1, OtherObject->obj_Data.position_3f, simx_opmode_streaming);
						break;
					case SIM_VELOCITY:
						OP_Obj.read_return_code = simxGetObjectVelocity(clientID, OtherObject->obj_handle,
																		OtherObject->obj_Data.linVelocity_3f, OtherObject->obj_Data.angVelocity_3f,
																		simx_opmode_streaming);
						break;
					case SIM_QUATERNION:
						/*Absolute quaternion*/
						OP_Obj.read_return_code = simxGetObjectQuaternion(clientID, OtherObject->obj_handle,
																		  -1, OtherObject->obj_Data.quaternion_4f, simx_opmode_streaming);
						break;
					case SIM_ORIENTATION:
						OP_Obj.read_return_code = simxGetObjectOrientation(clientID, OtherObject->obj_handle,
																		   -1, OtherObject->obj_Data.orientation_3f, simx_opmode_streaming);
						break;
					default:
						break;
					}
				}
				/*Write operation*/
				if (GetIO_Type(OP_Obj.operation) != CLIENT_RO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_POSITION:
						/*Absolute position*/
						OP_Obj.write_return_code = simxSetObjectPosition(clientID, OtherObject->obj_handle,
																		 -1, OtherObject->obj_Target.position_3f, simx_opmode_oneshot);
						break;
					case SIM_QUATERNION:
						/*Absolute quaternion*/
						OP_Obj.write_return_code = simxSetObjectOrientation(clientID, OtherObject->obj_handle,
																			-1, OtherObject->obj_Target.quaternion_4f, simx_opmode_oneshot);
						break;
					case SIM_ORIENTATION:
						OP_Obj.write_return_code = simxSetObjectOrientation(clientID, OtherObject->obj_handle,
																			-1, OtherObject->obj_Target.orientation_3f, simx_opmode_oneshot);
						break;
					default:
						break;
					}
				}

				if ((OP_Obj.read_return_code | OP_Obj.write_return_code & 0x02) == 1)
				{
					/*Do nothing*/
				}
			}
		}

		/*Iterate all vision sensors*/
		simxUChar *rgb_buff = NULL;
		simxFloat *depth_buff = NULL;
		size_t buff_size = 0;
		simxInt resolution_buf[2] = {0};
		for (auto &VisionSensor : VisionSensor_list)
		{
			for (auto &OP_Obj : VisionSensor->operation_list)
			{
				/*Read operation*/
				if (GetIO_Type(OP_Obj.operation) == CLIENT_RO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_NORMAL_IMAGE:
						OP_Obj.read_return_code = simxGetVisionSensorImage(clientID, VisionSensor->obj_handle,
																		   resolution_buf, &rgb_buff, 0, simx_opmode_streaming);
						/*New resolution and replace it*/
						buff_size = resolution_buf[0] *resolution_buf[1] * 3;
						
						if (resolution_buf[0] != VisionSensor->resolutionRGB[0] || resolution_buf[1] != VisionSensor->resolutionRGB[1])
						{
							if(VisionSensor->RGB_buffer != NULL)
							{
								delete[] VisionSensor->RGB_buffer;
								VisionSensor->RGB_buffer = NULL;
							} 
							VisionSensor->resolutionRGB[0] = resolution_buf[0];
							VisionSensor->resolutionRGB[1] = resolution_buf[1];

							/* 
								When the client is connected, the resolution will be a wrong value
								for a while. If the value is 0, skip the operation for this time.
							*/
							if (buff_size == 0) break;

							VisionSensor->RGB_buffer = new simxUChar[buff_size];
						}							

						/*
							Copy to local module, it may waste sometime.
						*/
						if (rgb_buff != NULL && VisionSensor->RGB_buffer != NULL)
							memcpy(VisionSensor->RGB_buffer, rgb_buff, buff_size*sizeof(simxUChar));

						break;
					case SIM_DEPTH_IMAGE:
						OP_Obj.read_return_code = simxGetVisionSensorDepthBuffer(clientID, VisionSensor->obj_handle,
																				 resolution_buf, &depth_buff, simx_opmode_streaming);
						/*New resolution and replace it*/
						buff_size = resolution_buf[0] *resolution_buf[1];

						if (resolution_buf[0] != VisionSensor->resolutionDepth[0] || resolution_buf[1] != VisionSensor->resolutionDepth[1])
						{
							if(VisionSensor->depth_buffer != NULL)
							{
								delete[] VisionSensor->depth_buffer;
								VisionSensor->depth_buffer = NULL;
							} 
							VisionSensor->resolutionDepth[0] = resolution_buf[0];
							VisionSensor->resolutionDepth[1] = resolution_buf[1];

							/* 
								When the client is connected, the resolution will be a wrong value
								for a while. If the value is 0, skip the operation for this time.
							*/
							if (buff_size == 0) break;

							VisionSensor->depth_buffer = new simxFloat[buff_size];
						}

						/*
							Copy to local module, it may waste sometime.
						*/
						if (depth_buff != NULL && VisionSensor->depth_buffer != NULL)
							memcpy(VisionSensor->depth_buffer, depth_buff, buff_size*sizeof(simxFloat));

						break;
					default:
						break;
					}
				}
				if ((OP_Obj.read_return_code | OP_Obj.write_return_code & 0x02) == 1)
				{
					/*Do nothing*/
				}
			}
		}
		/*Iterate all force sensors*/
		for (auto &ForceSensor : ForceSensor_list)
		{
			for (auto &OP_Obj : ForceSensor->operation_list)
			{
				/*Read operation*/
				if (GetIO_Type(OP_Obj.operation) == CLIENT_RO)
				{
					switch (GetOP_Type(OP_Obj.operation))
					{
					case SIM_FORCE:
						OP_Obj.read_return_code = simxReadForceSensor(clientID, ForceSensor->obj_handle,
																	  NULL, ForceSensor->obj_Data.force_3f, ForceSensor->obj_Data.torque_3f,
																	  simx_opmode_streaming);
						break;
					default:
						break;
					}
				}

				if ((OP_Obj.read_return_code | OP_Obj.write_return_code & 0x02) == 1)
				{
					/*Do nothing*/
				}
			}
		}
		/*Iterate all signals*/
		for (auto &Signal : Signal_list)
		{
			for (auto &OP_Obj : Signal->operation_list)
			{
				/*Check the operation type*/
				if (GetOP_Type(OP_Obj.operation) == SIM_SIGNAL_OP)
				{
					/*Read operation*/
					if (GetIO_Type(OP_Obj.operation) != CLIENT_WO)
					{
						simxInt temp = 0;
						/*Check the data type*/
						switch (Signal->signal_type)
						{
						case SIM_FLOAT_SIGNAL:
							OP_Obj.read_return_code = simxGetFloatSignal(clientID, Signal->siganl_name.data(),
																		 &Signal->data, simx_opmode_streaming);
							break;
						case SIM_STRING_SIGNAL:
							OP_Obj.read_return_code = simxGetStringSignal(clientID, Signal->siganl_name.data(),
																		  &Signal->string_data, &Signal->string_len, simx_opmode_streaming);
							break;
						case SIM_INTEGER_SIGNAL:
							OP_Obj.read_return_code = simxGetIntegerSignal(clientID, Signal->siganl_name.data(),
																		   &temp, simx_opmode_streaming);
							Signal->data = (simxFloat)temp;
							break;
						default:
							break;
						}
					}
					/*Write operation*/
					if (GetIO_Type(OP_Obj.operation) != CLIENT_RO)
					{
						/*Check the data type*/
						switch (Signal->signal_type)
						{
						case SIM_FLOAT_SIGNAL:
							OP_Obj.write_return_code = simxSetFloatSignal(clientID, Signal->siganl_name.data(),
																		  Signal->target, simx_opmode_streaming);
							break;
						case SIM_STRING_SIGNAL:
							OP_Obj.write_return_code = simxSetStringSignal(clientID, Signal->siganl_name.data(),
																		   Signal->string_data, Signal->string_len, simx_opmode_streaming);
							break;
						case SIM_INTEGER_SIGNAL:
							OP_Obj.write_return_code = simxSetIntegerSignal(clientID, Signal->siganl_name.data(),
																			(simxInt)Signal->target, simx_opmode_streaming);
							break;
						default:
							break;
						}
					}
					if ((OP_Obj.read_return_code | OP_Obj.write_return_code & 0x02) == 1)
					{
						/*Do nothing*/
					}
				}
			}
		}
		/*Iterate all collisions*/
		for (auto Collision : Collision_list)
		{
			if ((simxReadCollision(clientID, Collision->collision_handle, &Collision->collision_state, simx_opmode_streaming) & 0x02) == 1)
			{
				/*Do nothing*/
			}
		}
		return true;
	}
	else
		return false;
}

/**
* @brief  Stop simulation and clean up.
* @param  None.
* @retval None.
*/
CoppeliaSim_Client::~CoppeliaSim_Client()
{
	/*Clean up memory*/
	for (auto &Joint : Joint_list)
		delete Joint;
	Joint_list.clear();
	std::vector<_simObjectHandle_Type *>().swap(Joint_list);

	for (auto &ForceSensor : ForceSensor_list)
		delete ForceSensor;
	ForceSensor_list.clear();
	std::vector<_simObjectHandle_Type *>().swap(ForceSensor_list);

	for (auto &OtherObject : OtherObject_list)
		delete OtherObject;
	OtherObject_list.clear();
	std::vector<_simObjectHandle_Type *>().swap(OtherObject_list);

	for (auto &VisionSensor : VisionSensor_list)
		delete VisionSensor;
	VisionSensor_list.clear();
	std::vector<_simObjectHandle_Type *>().swap(VisionSensor_list);

	for (auto &Signal : Signal_list)
		delete Signal;
	Signal_list.clear();
	std::vector<_simSignalHandle_Type *>().swap(Signal_list);

	for (auto Collision : Collision_list)
		delete Collision;
	Collision_list.clear();
	std::vector<_simCollisHandle_Type *>().swap(Collision_list);
	/* End simulation service */
	simxStopSimulation(clientID, simx_opmode_oneshot);
	simxFinish(clientID);
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/