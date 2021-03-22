/* Includes ------------------------------------------------------------------*/
#include "coppeliaSim.h"
#include "sys_log.h"

/* Usr defines ---------------------------------------------------------------*/
using namespace std; 

_simObjectHandle_Type* Joint[7];
_simObjectHandle_Type* Tip[1];

/* Founctions ----------------------------------------------------------------*/

/**
* @brief This is the main function for user.
*/
void Usr_Main()
{
    //这里是主循环，可以运行我们的各部分算法
}

/**
* @brief User can config simulation client in this function.
* @note  It will be called before entering the main loop.   
*/
void Usr_ConfigSimulation()
{
    //这个函数里面配置要读写的对象
    Joint[0] = CoppeliaSim->Add_Object("IRB4600_auxJoint", JOINT, { SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[1] = CoppeliaSim->Add_Object("IRB4600_joint1", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[2] = CoppeliaSim->Add_Object("IRB4600_joint2", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[3] = CoppeliaSim->Add_Object("IRB4600_joint3", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[4] = CoppeliaSim->Add_Object("IRB4600_joint4", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[5] = CoppeliaSim->Add_Object("IRB4600_joint5", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[6] = CoppeliaSim->Add_Object("IRB4600_joint6", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
}

/**
* @brief These two function will be called for each loop.
*        User can set their message to send or read from sim enviroment.
*/
void Usr_SendToSimulation()
{
    for(int i = 0; i < 7; i++)
    {
        //在这里可以设置Joint的命令
    }
}

void Usr_ReadFromSimulation()
{
    for(int i = 0; i < 7; i++)
    {
        //在这里面循环从Joint中读取关节反馈
    }
}

/**
* @brief It's NOT recommended that user modefies this function.
*        Plz programm the functions with the prefix "Usr_". 
*/
int main(int argc, char *argv[])
{
    CoppeliaSim_Client *hClient = &CoppeliaSim_Client::getInstance();
    /*
        System Logger tool init.
    */
    std::cout << "[System Logger] Configuring... \n";
    SysLog->getMilliTick_regist(getSimTime);
    std::cout << "[System Logger] Logger is ready ! \n";

    /*
        Simulation connection init.
    */
    std::cout << "[CoppeliaSim Client] Connecting to server.. \n";
    while (!hClient->Start("127.0.0.1", 5000, 5, true)){};
    std::cout << "[CoppeliaSim Client] Successfully connected to server, configuring...\n";
    Usr_ConfigSimulation();
    std::cout << "[CoppeliaSim Client] Configure done, simulation is ready ! \n";

    while (1)
    {
        if (hClient->Is_Connected())
        {
            hClient->ComWithServer();
        }
        Usr_ReadFromSimulation();
        Usr_Main();
        Usr_SendToSimulation();
    };
}


/************************* END-OF-FILE SCUT-ROBOTLAB **************************/