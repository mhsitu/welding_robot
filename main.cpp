/* Includes ------------------------------------------------------------------*/
#include "matplotlibcpp.h"
#include <iostream>
#include "coppeliaSim.h"
#include "sys_log.h"
#include "core/BsplineBasic.h"
#include "core/BezierCurve.h"
#include "core/Timer.h"
#include "core/ACSRank_3D.hpp"
#include "core/read_STL.hpp"
#include "core/ACS_GTSP.hpp"

/* Usr defines ---------------------------------------------------------------*/
using namespace std;
namespace plt = matplotlibcpp;
enum Pose_t
{
    x,
    y,
    z,
    alpha,
    beta,
    gamma
};

_simObjectHandle_Type *Tip_target;
_simObjectHandle_Type *Tip_op;
_simObjectHandle_Type *Joint[6];
_simObjectHandle_Type *platform[2];

/*Test*/
BezierCurve<float, 3> straight_line(2);
BezierCurve<float, 2> platform_angle(2);
Timer timer;
int demo_type;
bool is_running = false;
float start_time = 0;
float total_time = 0;
float current_pt[6];
float target_pt[6];

/* Founctions ----------------------------------------------------------------*/
void manual_input()
{

    if (is_running == true)
    {
        // exit: current time > move time ?
        float now_time = (float)timer.getMs() - start_time;
        if (now_time >= total_time)
            is_running = false;

        if (demo_type == 1)
        {
            float res[3] = {};
            straight_line.getCurvePoint(now_time, res);
            target_pt[0] = res[0];
            target_pt[1] = res[1];
            target_pt[2] = res[2];
            cout << "Target(x,y,z):" << target_pt[0] << ", " << target_pt[1] << ", " << target_pt[2] << endl;
        }
        else if (demo_type == 2)
        {
            float res[2];
            platform_angle.getCurvePoint(now_time, res);
            platform[0]->obj_Target.angle_f = res[0];
            platform[1]->obj_Target.angle_f = res[1];
            cout << "Target(pitch, yaw):" << res[0] << ", " << res[1] << endl;
        }
        else
        {
        }
    }
    else
    {
        //Select type
        cout << "Please choose control type: 1) Manipulator 2) Platform -- ";
        cin >> demo_type;
        if (demo_type == 1)
        {
            //Set terminal points
            float start_pt[3] = {current_pt[0], current_pt[1], current_pt[2]};
            float next_pt[3];
            float **ctrl_pt = new float *[3];
            ctrl_pt[0] = start_pt;
            ctrl_pt[1] = next_pt;
            cout << "Current point:(" << current_pt[0] << ", " << current_pt[1] << ", " << current_pt[2] << ")" << endl;
            cout << "Next point(x, y, z) and Time(t): ";
            cin >> next_pt[0] >> next_pt[1] >> next_pt[2] >> total_time;

            straight_line.SetParam(ctrl_pt, total_time);
            //Set time
            start_time = timer.getMs();
            is_running = true;
        }
        else if (demo_type == 2)
        {
            //Set terminal points
            float start_pt[2] = {platform[0]->obj_Data.angle_f, platform[1]->obj_Data.angle_f};
            float next_pt[2];
            float **ctrl_pt = new float *[2];
            ctrl_pt[0] = start_pt;
            ctrl_pt[1] = next_pt;
            cout << "Current point:(" << platform[0]->obj_Data.angle_f << ", " << platform[1]->obj_Data.angle_f << ")" << endl;
            cout << "Target angle(pitch, yaw) and Time(t): ";
            cin >> next_pt[0] >> next_pt[1] >> total_time;

            platform_angle.SetParam(ctrl_pt, total_time);
            //Set time
            start_time = timer.getMs();
            is_running = true;
        }
        else
        {
            cout << "Unidentified type, please select again." << endl;
        }
    }
}
/**
* @brief This is the main function for user.
*/
void Usr_Main()
{
    //这里是主循环，可以运行我们的各部分算法
    manual_input();
}

/**
* @brief User can config simulation client in this function.
* @note  It will be called before entering the main loop.   
*/
void Usr_ConfigSimulation()
{
    //添加关节对象，每个关节可以读写位置和速度，不用单独控制每个关节可以注释下面这段
    Joint[0] = CoppeliaSim->Add_Object("IRB4600_joint1", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[1] = CoppeliaSim->Add_Object("IRB4600_joint2", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[2] = CoppeliaSim->Add_Object("IRB4600_joint3", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[3] = CoppeliaSim->Add_Object("IRB4600_joint4", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[4] = CoppeliaSim->Add_Object("IRB4600_joint5", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
    Joint[5] = CoppeliaSim->Add_Object("IRB4600_joint6", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});

    //读写执行末端相对于器件坐标系的位姿
    Tip_target = CoppeliaSim->Add_Object("IRB4600_IkTarget", OTHER_OBJECT, {SIM_POSITION | CLIENT_WO, SIM_ORIENTATION | CLIENT_WO});
    Tip_op = CoppeliaSim->Add_Object("IRB4600_IkTip", OTHER_OBJECT, {SIM_POSITION | CLIENT_RO, SIM_ORIENTATION | CLIENT_RO});
    platform[0] = CoppeliaSim->Add_Object("platform_yaw", JOINT, {SIM_POSITION | CLIENT_RW});
    platform[1] = CoppeliaSim->Add_Object("platform_pitch", JOINT, {SIM_POSITION | CLIENT_RW});

    /*Init value*/
    target_pt[x] = -0.2;
    target_pt[y] = 0;
    target_pt[z] = 1.72;
    target_pt[alpha] = 0;
    target_pt[beta] = M_PI_2;
    target_pt[gamma] = -M_PI_2;

    Tip_target->obj_Target.position_3f[0] = target_pt[x] + 1.7;
    Tip_target->obj_Target.position_3f[1] = target_pt[y] + 0;
    Tip_target->obj_Target.position_3f[2] = target_pt[z] + 0;
    Tip_target->obj_Target.orientation_3f[0] = target_pt[alpha];
    Tip_target->obj_Target.orientation_3f[1] = target_pt[beta];
    Tip_target->obj_Target.orientation_3f[2] = target_pt[gamma];
}

/**
* @brief These two function will be called for each loop.
*        User can set their message to send or read from sim enviroment.
*/
void Usr_SendToSimulation()
{
    //这里可以设置关节指令
    Tip_target->obj_Target.position_3f[0] = target_pt[x] + 1.7;
    Tip_target->obj_Target.position_3f[1] = target_pt[y] + 0;
    Tip_target->obj_Target.position_3f[2] = target_pt[z] + 0;
    Tip_target->obj_Target.orientation_3f[0] = target_pt[alpha];
    Tip_target->obj_Target.orientation_3f[1] = target_pt[beta];
    Tip_target->obj_Target.orientation_3f[2] = target_pt[gamma];
}

void Usr_ReadFromSimulation()
{
    //这里可以读取反馈
    current_pt[x] = Tip_op->obj_Data.position_3f[0] - 1.7;
    current_pt[y] = Tip_op->obj_Data.position_3f[1] - 0;
    current_pt[z] = Tip_op->obj_Data.position_3f[2] - 0;
    current_pt[alpha] = Tip_op->obj_Data.orientation_3f[0];
    current_pt[beta] = Tip_op->obj_Data.orientation_3f[1];
    current_pt[gamma] = Tip_op->obj_Data.orientation_3f[2];
}

inline void show_vector_improve()
{
    std::map<std::string, std::string> keywords;

    std::vector<float> x1, y1, z1;
    srand(time(0));
    float r = 2;
    float beta = 0.8;
    x1.push_back(0);
    y1.push_back(0);
    z1.push_back(0);
    x1.push_back(5);
    y1.push_back(5);
    z1.push_back(5);
    x1.push_back(-2);
    y1.push_back(-2);
    z1.push_back(-2);
    for(float i = 0; i < 5;)
    {
        x1.push_back(i);
        y1.push_back(i);
        z1.push_back(i);
        i += 0.2;
    }
    // keywords.insert(std::pair<std::string, std::string>("c", "red"));
    // keywords.insert(std::pair<std::string, std::string>("linewidth", "2"));
    // plt::plot3(x1, y1, z1, keywords,1);

    keywords.clear();
    keywords.insert(std::pair<std::string, std::string>("c", "red"));
    keywords.insert(std::pair<std::string, std::string>("marker", "^"));
    plt::scatter(x1, y1, z1, 1,keywords,1);

    x1.clear();
    y1.clear();
    z1.clear();
    Point3<float> vector_a = Point3<float>(5,5,5);
    for (int i(0); i < 1000; i++)
    {
        float phi = -M_PI + 2*M_PI*(float)(rand()) / (float)RAND_MAX;
        float theta = -M_PI + 2*M_PI*(float)(rand()) / (float)RAND_MAX;
        float x = r * sin(theta) * cos(phi);
        float y = r * sin(theta) * sin(phi);
        float z = r * cos(theta);
        Point3<float> vector_b(x,y,z);
        float cos_garmma = power(Point3<float>::dot(vector_b, vector_a) / (vector_a.norm() * vector_b.norm()),3);
        x = beta*(1 + cos_garmma) * r * sin(theta) * cos(phi);
        y = beta*(1 + cos_garmma) * r * sin(theta) * sin(phi);
        z = beta*(1 + cos_garmma) * r * cos(theta);
        x1.push_back(x);
        y1.push_back(y);
        z1.push_back(z);
    }

    keywords.clear();
    keywords.insert(std::pair<std::string, std::string>("c", "blue"));
    keywords.insert(std::pair<std::string, std::string>("marker", "o"));
    plt::scatter(x1, y1, z1, 1, keywords, 1);
}

/**
* @brief It's NOT recommended that user modefies this function.
*        Plz programm the functions with the prefix "Usr_". 
*/
int main(int argc, char *argv[])
{
    STLReader model;
    ACS_Rank SearchPath;
    ACS_GTSP GlobalRoute;
    /*
        读取工件模型
    */
    model.readFile("./files/test.stl");
    const std::vector<Triangles<float>> meshes = model.TriangleList();
    
    /*
        搜索路径
    */
    SearchPath.creatGridMap(meshes, 0.05, 6,"./files/test_grid_map.in");
    //SearchPath.readGridMap("./files/test_grid_map.in");
    // SearchPath.plot_grid_map(1);
    // SearchPath.show_plot();
    SearchPath.searchBestPathOfPoints(10, "./files/weld_points.in", "./files/graph.in");

    GlobalRoute.readFromGraphFile("./files/graph.in");
    GlobalRoute.computeSolution();
    GlobalRoute.read_segment(SearchPath.best_matrix, 1);

    /*
        结果可视化
    */
    GlobalRoute.plot_route_path(1);
    SearchPath.plot_route_point(1);
    //SearchPath.plot_grid_map(1);
    SearchPath.show_plot();

    /*
        曲线平滑
    */
    const int pt_num = GlobalRoute.g_path_x.size() - 2;
    BezierCurve<float, 3> smooth_curve(pt_num);
    float **ctrl_points = new float *[pt_num];
    for (int i = 0; i < pt_num; ++i)
    {
        ctrl_points[i] = new float[3];
        ctrl_points[i][0] = GlobalRoute.g_path_x[i];
        ctrl_points[i][1] = GlobalRoute.g_path_y[i];
        ctrl_points[i][2] = GlobalRoute.g_path_z[i];
    }
    smooth_curve.SetParam(ctrl_points, 5);
    
    clock_t time = clock();

    float res[3];
    while (clock() - time < 5000)
    {
        plt::clf();
        smooth_curve.getCurvePoint(clock - time, );
        plt::pause(10);
    }

    exit(0);

    /*
        System Logger tool init.
    */
    std::cout << "[System Logger] Configuring... \n";
    std::cout << "[System Logger] Logger is ready ! \n";

    /*
        Simulation connection init.
    */
    CoppeliaSim_Client *hClient = &CoppeliaSim_Client::getInstance();
    std::cout << "[CoppeliaSim Client] Connecting to server.. \n";
    while (!hClient->Start("127.0.0.1", 5000, 5, false))
    {
    };
    std::cout << "[CoppeliaSim Client] Successfully connected to server, configuring...\n";
    Usr_ConfigSimulation();
    std::cout << "[CoppeliaSim Client] Configure done, simulation is ready ! \n";

    while (1)
    {
        // Abandon top 5 data
        static int init_num = 5;
        if (hClient->Is_Connected())
        {
            hClient->ComWithServer();
        }
        if (init_num > 0)
            init_num--;
        else
        {
            Usr_ReadFromSimulation();
            Usr_Main();
            Usr_SendToSimulation();
        }
    };
}

/************************* END-OF-FILE SCUT-ROBOTLAB **************************/