/* Includes ------------------------------------------------------------------*/
#include "matplotlibcpp.h"
#include <iostream>
#include "CoppeliaSim.h"
#include "sys_log.h"
#include "core/BSplineBasic.h"
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
    _gamma
    
};

_simObjectHandle_Type *Tip_target;
_simObjectHandle_Type *Tip_op;
_simObjectHandle_Type *Joint[6];
_simObjectHandle_Type *platform[2];
_simSignalHandle_Type *weld_cmd;
/*Test*/
STLReader model;
ACS_Rank SearchPath;
ACS_GTSP GlobalRoute;
BezierCurve<float, 3> straight_line(2);
BezierCurve<float, 2> platform_angle(2);
BS_Basic<float, 3, 0, 0, 0> *smooth_curve1;
Timer timer;
int demo_type;
bool is_running = false;
float start_time = 0;
float total_time = 0;
float current_pt[6];
float target_pt[6];
std::vector<float> smooth_x, smooth_y, smooth_z;
/* Founctions ----------------------------------------------------------------*/
// float[6], float[3]
bool go_next_point(float *next, float *res)
{
    static float last[6] = {0};
    bool state = false;
    for (int i(0); i < 6; i++)
    {
        state |= (next[i] != last[i]) ? 1 : 0;
    }

    if(state){ 
        float start_pt[3] = {current_pt[0], current_pt[1], current_pt[2]};
        float next_pt[3] = {next[0], next[1], next[2]};
        float **ctrl_pt = new float *[3];
        ctrl_pt[0] = start_pt;
        ctrl_pt[1] = next_pt;
        straight_line.SetParam(ctrl_pt, total_time);
        start_time = timer.getMs();
        for (int i(0); i < 6; i++)
        {
            last[i] = next[i];
        }
    }

    float now_time = (float)timer.getMs() - start_time;
    if (now_time >= total_time)
        return false;
    else
    {
        straight_line.getCurvePoint(now_time, res);
        printf("This point: %.3f, %.3f, %.3f \n", res[0], res[1], res[2]);
        return true;
    }
}

void manual_input()
{

    if (is_running == true)
    {
        if (demo_type == 1)
        {
            // exit: current time > move time ?
            float now_time = (float)timer.getMs() - start_time;
            if (now_time >= total_time)
            is_running = false;

            float res[3] = {};
            straight_line.getCurvePoint(now_time, res);
            target_pt[0] = res[0];
            target_pt[1] = res[1];
            target_pt[2] = res[2];
            cout << "Target(x,y,z):" << target_pt[0] << ", " << target_pt[1] << ", " << target_pt[2] << endl;
        }
        else if (demo_type == 2)
        {
            // exit: current time > move time ?
            float now_time = (float)timer.getMs() - start_time;
            if (now_time >= total_time)
            is_running = false;

            float res[2];
            platform_angle.getCurvePoint(now_time, res);
            platform[0]->obj_Target.angle_f = res[0];
            platform[1]->obj_Target.angle_f = res[1];
            cout << "Target(pitch, yaw):" << res[0] << ", " << res[1] << endl;
        }
        else
        {
            static int i = 0;
            if(i < smooth_x.size())
            {
                static clock_t lastTime = clock();
                if (clock() - lastTime >= 10)
                {
                    lastTime = clock();
                    if(smooth_x[i] - target_pt[0] < 0.3 && smooth_y[i] - target_pt[1] < 0.3
                        && smooth_z[i] - target_pt[2] < 0.3)
                    target_pt[0] = smooth_x[i];
                    target_pt[1] = smooth_y[i];
                    target_pt[2] = smooth_z[i];
                    cout << "Target(x,y,z):" << target_pt[0] << ", " << target_pt[1] << ", " << target_pt[2] << endl;
                    i++;
                }
            }
            else
            {
                is_running = false;
            }
            //     // 论文和答辩中简单的演示,简单的状态机
            //     static int stage = 0;
            //     switch(stage)
            //     {
            //         case 0:
            //         {
            //             //到第一个点
            //             total_time = 3000;
            //             float next[3] = {SearchPath.route_points[0].x, SearchPath.route_points[0].y, SearchPath.route_points[0].z};
            //             if(go_next_point(next,res))
            //             {
            //                 // target_pt[0] = res[0];
            //                 // target_pt[1] = res[1];
            //                 // target_pt[2] = res[2];
            //             }
            //             else
            //             {
            //                 start_time = timer.getMs();
            //                 stage = 1;
            //             }
            //         }
            //         break;
            //         case 1:
            //         {
            //             //开始焊接，到第二个点
            //             weld_cmd->target = 1;
            //             float next[3] = {SearchPath.route_points[1].x, SearchPath.route_points[1].y, SearchPath.route_points[1].z};
            //             if(go_next_point(next,res))
            //             {
            //                 // target_pt[0] = res[0];
            //                 // target_pt[1] = res[1];
            //                 // target_pt[2] = res[2];
            //             }
            //             else{
            //                 const std::vector<ACS_Node<float> *> *path = SearchPath.best_matrix[1][2].getPath();
            //                 int pt_num = (*path).size();
            //                 float start_pt[3] = {SearchPath.route_points[1].x, SearchPath.route_points[1].y, SearchPath.route_points[1].z};
            //                 float end_pt[3] = {SearchPath.route_points[2].x, SearchPath.route_points[2].y, SearchPath.route_points[2].z};
            //                 float **ctrl_pt = new float *[pt_num];
            //                 for (int i = 0; i < pt_num; ++i)
            //                 {
            //                     ctrl_pt[i] = new float[3];
            //                     ctrl_pt[i][0] = (*path)[i]->pt.x;
            //                     ctrl_pt[i][1] = (*path)[i]->pt.y;
            //                     ctrl_pt[i][2] = (*path)[i]->pt.z;
            //                 }
            //                 smooth_curve1 = new BS_Basic<float, 3, 0, 0, 0>(pt_num);
            //                 smooth_curve1->SetParam(start_pt, end_pt, ctrl_pt, total_time);
            //                 start_time = timer.getMs();
            //                 weld_cmd->target = 0;
            //                 stage = 2;
            //             }
            //         }
            //         break;
            //         case 2:
            //         {
            //             //停止焊接，到下面焊路
            //             float now_time = (float)timer.getMs() - start_time;
            //             if(now_time < total_time + 500)
            //             {
            //                 smooth_curve1->getCurvePoint(now_time, res);
            //             }
            //             else
            //             {
            //                 weld_cmd->target = 1;
            //                 stage = 3;
            //             }
            //         }
            //         break;
            //         case 3:
            //         {
            //             // 第二段焊路
            //             float next[3] = {SearchPath.route_points[3].x, SearchPath.route_points[3].y, SearchPath.route_points[3].z};
            //             if(go_next_point(next,res))
            //             {

            //             }
            //             else
            //             {
            //                 while(1){}
            //             }
            //         }
            //         break;
            //         default:
            //             break;
            //     }
            //     target_pt[0] = res[0];
            //     target_pt[1] = res[1];
            //     target_pt[2] = res[2];
            // }
        }
    }
    else
    {
        //Select type
        cout << "Please choose control type: 1) Manipulator 2) Platform 3) Demo : ";
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
        else if(demo_type == 3)
        {
            /*
                读取工件模型
            */
            model.readFile("./files/cubic.stl");
            const std::vector<Triangles<float>> meshes = model.TriangleList();
            
            /*
                搜索路径
            */
            SearchPath.creatGridMap(meshes, 0.005, 10,"./files/cubic_grid_map.in");
            SearchPath.searchBestPathOfPoints(0.5, "./files/cubic_weld_points.in", "./files/graph.in");
            GlobalRoute.readFromGraphFile("./files/graph.in");
            GlobalRoute.computeSolution();
            GlobalRoute.read_all_segments(SearchPath.best_matrix);
            /*
                曲线平滑
            */
            int pt_num = GlobalRoute.g_path_x.size();
            float start_pt[3] = {GlobalRoute.g_path_x[0], GlobalRoute.g_path_y[0], GlobalRoute.g_path_z[0]};
            float end_pt[3] = {GlobalRoute.g_path_x[pt_num - 1], GlobalRoute.g_path_y[pt_num - 1], GlobalRoute.g_path_z[pt_num - 1]};
            float **ctrl_pt = new float *[pt_num];
            for (int i = 0; i < pt_num; ++i)
            {
                ctrl_pt[i] = new float[3];
                ctrl_pt[i][0] = GlobalRoute.g_path_x[i];
                ctrl_pt[i][1] = GlobalRoute.g_path_y[i];
                ctrl_pt[i][2] = GlobalRoute.g_path_z[i];
            }

            BS_Basic<float, 3, 0, 0, 0> smooth_curve(pt_num);
            smooth_curve.SetParam(start_pt, end_pt, ctrl_pt, 150);

            clock_t base_t = clock();
            clock_t now_t = clock()-base_t;
            float res[3];

            do
            {
                if(clock() - base_t - now_t >= 10)
                {
                    now_t = clock() - base_t;
                    smooth_curve.getCurvePoint(now_t, res);
                    smooth_x.push_back(res[0]);
                    smooth_y.push_back(res[1]);
                    smooth_z.push_back(res[2]);
                    //printf("Curve point: %f, %f, %f, time:%d \n", res[0], res[1], res[2], now_t);
                }
            } while (now_t <= 150);

            //二次平滑
            const float constrain = 0.05;
            pt_num = smooth_y.size();
            float second_start_pt[9] = {GlobalRoute.g_path_x[0], GlobalRoute.g_path_y[0], GlobalRoute.g_path_z[0],0,0,0,0,0,0}; 
            float second_end_pt[9] = {GlobalRoute.g_path_x[pt_num - 1], GlobalRoute.g_path_y[pt_num - 1], GlobalRoute.g_path_z[pt_num - 1],0,0,0,0,0,0};
            float **second_pt = new float*[pt_num];
            for (int i = 0; i < pt_num; ++i)
            {
                second_pt[i] = new float[9];
                second_pt[i][0] = smooth_x[i];
                second_pt[i][1] = smooth_y[i];
                second_pt[i][2] = smooth_z[i];
                for (int j(3); j < 9; j++)
                    second_pt[i][j] = constrain;
            }
            smooth_x.clear();
            smooth_y.clear();
            smooth_z.clear();
            BS_Basic<float, 3, 2, 2, 2> second_curve(pt_num);
            second_curve.SetParam(second_start_pt,second_end_pt,second_pt, 6000);
            base_t = clock();
            now_t = clock()-base_t;
            do
            {
                if(clock() - base_t - now_t >= 50)
                {
                    now_t = clock() - base_t;
                    second_curve.getCurvePoint(now_t, res);
                    smooth_x.push_back(res[0]);
                    smooth_y.push_back(res[1]);
                    smooth_z.push_back(res[2]);
                    //printf("Second point: %f, %f, %f, time:%d \n", res[0], res[1], res[2], now_t);
                }
            } while (now_t <= 6000);
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
    weld_cmd = CoppeliaSim->Add_Object("weld_cmd", SIM_INTEGER_SIGNAL, {SIM_SIGNAL_OP | CLIENT_WO});

    /*Init value*/
    target_pt[x] = 1.76; //-0.2;
    target_pt[y] = 0.09;
    target_pt[z] = 1.42;
    target_pt[alpha] = 0;
    target_pt[beta] = M_PI_2 + M_PI_2/2;
    target_pt[_gamma] = -M_PI_2;

    Tip_target->obj_Target.position_3f[0] = target_pt[x] + 0;//1.7;
    Tip_target->obj_Target.position_3f[1] = target_pt[y] + 0;
    Tip_target->obj_Target.position_3f[2] = target_pt[z] + 0;
    Tip_target->obj_Target.orientation_3f[0] = target_pt[alpha];
    Tip_target->obj_Target.orientation_3f[1] = target_pt[beta];
    Tip_target->obj_Target.orientation_3f[2] = target_pt[_gamma];
}

/**
* @brief These two function will be called for each loop.
*        User can set their message to send or read from sim enviroment.
*/
void Usr_SendToSimulation()
{
    //这里可以设置关节指令
    Tip_target->obj_Target.position_3f[0] = target_pt[x] + 0; //1.7;
    Tip_target->obj_Target.position_3f[1] = target_pt[y] + 0;
    Tip_target->obj_Target.position_3f[2] = target_pt[z] + 0;
    Tip_target->obj_Target.orientation_3f[0] = target_pt[alpha];
    Tip_target->obj_Target.orientation_3f[1] = target_pt[beta];
    Tip_target->obj_Target.orientation_3f[2] = target_pt[_gamma];
}

void Usr_ReadFromSimulation()
{
    //这里可以读取反馈
    current_pt[x] = Tip_op->obj_Data.position_3f[0] - 0; //1.7;
    current_pt[y] = Tip_op->obj_Data.position_3f[1] - 0;
    current_pt[z] = Tip_op->obj_Data.position_3f[2] - 0;
    current_pt[alpha] = Tip_op->obj_Data.orientation_3f[0];
    current_pt[beta] = Tip_op->obj_Data.orientation_3f[1];
    current_pt[_gamma] = Tip_op->obj_Data.orientation_3f[2];
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
    // STLReader model;
    // ACS_Rank SearchPath;
    // ACS_GTSP GlobalRoute;
    /*
        读取工件模型
    */
    // model.readFile("./files/cubic.stl");
    // const std::vector<Triangles<float>> meshes = model.TriangleList();
    
    // /*
    //     搜索路径
    // */
    // SearchPath.creatGridMap(meshes, 0.005, 10,"./files/cubic_grid_map.in");
    // SearchPath.searchBestPathOfPoints(0.5, "./files/cubic_weld_points.in", "./files/graph.in");

    // GlobalRoute.readFromGraphFile("./files/graph.in");
    // GlobalRoute.computeSolution();
    // GlobalRoute.read_all_segments(SearchPath.best_matrix);

    // /*
    //     结果可视化
    // */
    // GlobalRoute.plot_route_path(1);
    // SearchPath.plot_route_point(1);
    // SearchPath.plot_grid_map(1);
    // SearchPath.show_plot();

    // /*
    //     曲线平滑
    // */
    // int pt_num = GlobalRoute.g_path_x.size();
    // float start_pt[3] = {SearchPath.route_points[0].x, SearchPath.route_points[0].y, SearchPath.route_points[0].z};
    // float end_pt[3] = {SearchPath.route_points[1].x, SearchPath.route_points[1].y, SearchPath.route_points[1].z};
    // //{GlobalRoute.g_path_x[pt_num - 1], GlobalRoute.g_path_y[pt_num - 1], GlobalRoute.g_path_z[pt_num - 1]};
    // float **ctrl_pt = new float *[pt_num];
    // for (int i = 0; i < pt_num; ++i)
    // {
    //     ctrl_pt[i] = new float[3];
    //     ctrl_pt[i][0] = GlobalRoute.g_path_x[i];
    //     ctrl_pt[i][1] = GlobalRoute.g_path_y[i];
    //     ctrl_pt[i][2] = GlobalRoute.g_path_z[i];
    // }

    // BS_Basic<float, 3, 0, 0, 0> smooth_curve(pt_num);
    // smooth_curve.SetParam(start_pt, end_pt, ctrl_pt, 500);

    // clock_t base_t = clock();
    // clock_t now_t = clock()-base_t;
    // float res[3];
    // std::vector<float> smooth_x, smooth_y, smooth_z;
    // do
    // {
    //     if(clock() - base_t - now_t >= 25)
    //     {
    //         now_t = clock() - base_t;
    //         smooth_curve.getCurvePoint(now_t, res);
    //         smooth_x.push_back(res[0]);
    //         smooth_y.push_back(res[1]);
    //         smooth_z.push_back(res[2]);
    //         printf("Curve point: %f, %f, %f, time:%d \n", res[0], res[1], res[2], now_t);
    //     }
    // } while (now_t <= 500);

    // //二次平滑
    // const float constrain = 0.2;
    // pt_num = smooth_y.size();
    // float second_start_pt[9] = {SearchPath.route_points[0].x, SearchPath.route_points[0].y, SearchPath.route_points[0].z,0,0,0,0,0,0}; 
    // float second_end_pt[9] = {SearchPath.route_points[1].x, SearchPath.route_points[1].y, SearchPath.route_points[1].z,0,0,0,0,0,0};
    // float **second_pt = new float*[pt_num];
    // for (int i = 0; i < pt_num; ++i)
    // {
    //     second_pt[i] = new float[9];
    //     second_pt[i][0] = smooth_x[i];
    //     second_pt[i][1] = smooth_y[i];
    //     second_pt[i][2] = smooth_z[i];
    //     for (int j(3); j < 9; j++)
    //         second_pt[i][j] = constrain;
    // }
    // smooth_x.clear();
    // smooth_y.clear();
    // smooth_z.clear();
    // BS_Basic<float, 3, 2, 2, 2> second_curve(pt_num);
    // second_curve.SetParam(second_start_pt,second_end_pt,second_pt, 500);
    // base_t = clock();
    // now_t = clock()-base_t;
    // do
    // {
    //     if(clock() - base_t - now_t >= 2)
    //     {
    //         now_t = clock() - base_t;
    //         second_curve.getCurvePoint(now_t, res);
    //         smooth_x.push_back(res[0]);
    //         smooth_y.push_back(res[1]);
    //         smooth_z.push_back(res[2]);
    //         printf("Second point: %f, %f, %f, time:%d \n", res[0], res[1], res[2], now_t);
    //     }
    // } while (now_t <= 500);

    // std::map<std::string, std::string> keywords;
    // keywords.insert(std::pair<std::string, std::string>("c", "red"));
    // plt::plot3(smooth_x, smooth_y, smooth_z,keywords,1);
    // plt::show();
    // exit(0);

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