/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include "coppeliaSim.h"
#include "sys_log.h"
#include "core/BezierCurve.h"
#include "core/Timer.h"
/* Usr defines ---------------------------------------------------------------*/
using namespace std;
enum Pose_t {
    x, y, z, alpha, beta, gamma
};

_simSignalHandle_Type *Tip[6];
_simObjectHandle_Type *Joint[6];

BezierCurve<float, 3, 2> straight_line;
Timer timer;
bool is_running = false;
float start_time = 0;
float total_time = 0;
/* Founctions ----------------------------------------------------------------*/

/**
* @brief This is the main function for user.
*/
void Usr_Main()
{
    //这里是主循环，可以运行我们的各部分算法
    if(is_running == true)
    {
        float now_time = (float)timer.getMs() - start_time;
        if (now_time >= total_time)
            is_running = false;

        float target_pt[3] = {};
        straight_line.getCurvePoint(now_time, target_pt);
        cout << "Target(x,y,z):" << target_pt[0] << ", " << target_pt[1] << ", " << target_pt[2] << endl;
    }
    else{
        //Set points
        float current_pt[3] = {Tip[x]->data, Tip[y]->data, Tip[z]->data};
        float next_pt[3];
        float **ctrl_pt = new float *[3];
        ctrl_pt[0] = current_pt;
        ctrl_pt[1] = next_pt;
        //Set time
        cout << "Current point:(" << current_pt[0] << ", " << current_pt[1] << ", " << current_pt[2] << ")" << endl;
        cout << "Next point(x, y, z): " << endl;
        cin >> next_pt[0] >> next_pt[1] >> next_pt[2] >> total_time ;

        straight_line.SetParam(ctrl_pt, total_time);

        start_time = timer.getMs();
        is_running = true;
    }
}

/**
* @brief User can config simulation client in this function.
* @note  It will be called before entering the main loop.   
*/
void Usr_ConfigSimulation()
{
    //添加关节对象，每个关节可以读写位置和速度，不用单独控制每个关节可以注释下面这段
    // Joint[0] = CoppeliaSim->Add_Object("IRB4600_joint1", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    // Joint[1] = CoppeliaSim->Add_Object("IRB4600_joint2", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    // Joint[2] = CoppeliaSim->Add_Object("IRB4600_joint3", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    // Joint[3] = CoppeliaSim->Add_Object("IRB4600_joint4", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    // Joint[4] = CoppeliaSim->Add_Object("IRB4600_joint5", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    // Joint[5] = CoppeliaSim->Add_Object("IRB4600_joint6", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});

    //读写执行末端相对于器件坐标系的位姿
    Tip[Pose_t::x] = CoppeliaSim->Add_Object("target_x", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    Tip[Pose_t::y] = CoppeliaSim->Add_Object("target_y", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    Tip[Pose_t::z] = CoppeliaSim->Add_Object("target_z", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    Tip[Pose_t::alpha] = CoppeliaSim->Add_Object("target_alpha", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    Tip[Pose_t::beta] = CoppeliaSim->Add_Object("target_beta", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    Tip[Pose_t::gamma] = CoppeliaSim->Add_Object("target_gamma", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
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
    std::cout << "[System Logger] Logger is ready ! \n";

    /*
        Simulation connection init.
    */
    std::cout << "[CoppeliaSim Client] Connecting to server.. \n";
    while (!hClient->Start("127.0.0.1", 5000, 5, false)){};
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