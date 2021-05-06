/**
  ******************************************************************************
  * @file    ACS_3D.hpp
  * @brief   Using Ant Colony System to search a path in 3D grid-based map.
  * @author  Stuart(South China University of Technology)
  *          1356046979@qq.com
  * @date    18/04/2021
  * @version 0.1
  ******************************************************************************
*/
#ifndef _ACS_3D_HPP
#define _ACS_3D_HPP
#include <assert.h>
#include <bits/stdc++.h>
#include <map>
#include <functional>
#include "model_grid_map.hpp"

#define INF 0x3f3f3f3f
#define eps 1e-8

typedef Point3<int> Point3i;

//任意两点间的信息
template <class T>
struct _Inf_of_Points_t
{
    T distance;  //距离
    T pheromone; //信息素
    //T herustic;  //启发值
    T info;      //信息值
};

template <class T>
class ACS_Node : public Vertex3<T>
{
    std::vector<ACS_Node<T> *> adjacency_nodes;    // 邻接结点
    std::vector<_Inf_of_Points_t> adjacency_infos; // 邻接结点的信息
};

//快速幂，计算x ^ y，时间复杂度O(logn)
template <class T>
T power(T x, int y)
{
    T ans = 1;
    while (y)
    {
        if (y & 1)
            ans *= x;
        x *= x;
        y >>= 1;
    }
    return ans;
}

template <class T>
class Agent
{
private:
    std::vector<Node<T> *> path; // 路径点

public:
    std::set<int> tabu_list; // 禁止列表（已走过结点）
    T L;                     //路径长度

    void addNextNode(Vertex3<T> *pt, T _dis)
    {
        tabu_list.insert(pt->id);
        path.push_back(pt);
        L += _dis;
    }

    void addStartNode(Vertex3<T> *_start)
    {
        tabu_list.insert(_start->id);
        path.push_back(_start);
    }

    void setDeadEnd()
    {
        L = INF;
    }

    const std::vector<Vertex3<T> *> *getPath() const
    {
        return &path;
    }
};

class ACS_Base : public GridMap<float>
{
private:
    int colony_num;      // 蚁群数量
    int index_iteration; // 迭代索引
    int max_iteration;   // 最大迭代次数
    float pheromone_0, Q;
    _Inf_of_Points_t<float> **info_matrix; // 邻接信息矩阵
    int alpha, beta;                       // pheromone, heuristic information的权重
    float rho;                             // pheromone的挥发系数
    std::vector<Agent<float>> agents;      // 蚁群中每只蚂蚁
    Agent<float> *best;                    // 当前最优蚂蚁

    /**
    * @brief next nodes of ant K.  
    * @param agentK ant K
    * @param cur pointer of current node
    * @param next pointer of next node
    * @retval true update next point
    * @retval false dead end/bad route or reach end point
    */
    bool selectNext(Agent<float> &agentK, Vertex3<float> *cur, Vertex3<float> *next)
    {
        std::vector<Vertex3<float> *> J;

        // 当前点周围被障碍包围
        if (cur->adjacency_nodes.empty())
        {
            agentK.setDeadEnd();
            return false;
        }

        float prob_sum = 0, total = 0;
        // 查找所有可达点和信息素总和
        for_each(cur->adjacency_nodes.begin(), cur->adjacency_nodes.end(), [&](auto _it) {
            // 检查邻近结点是否被这只蚂蚁走过，防止闭环
            if (agentK.tabu_list.find(cur->id) != agentK.tabu_list.end())
            {
                J.push_back(_it);
                info_matrix[cur->id][_it->id].info = power(info_matrix[cur->id][_it->id].pheromone, alpha);
                total += info_matrix[cur->id][_it->id].info;
            }
        });

        // 没有可达点，本路径到尽头
        if (J.empty())
        {
            agentK.setDeadEnd();
            return false;
        }

        // 计算每个点的概率,轮盘赌（roulette wheel selection）选择下一个点
        float rnd = (float)(rand()) / (float)RAND_MAX;
        rnd *= total;

        for (int i = 0, s = J.size(); i < s; i++)
        {
            prob_sum += info_matrix[cur->id][J[i]->id].info;
            if (prob_sum >= rnd)
            {
                next = J[i];
                agentK.addNextNode(J[i], info_matrix[cur->id][J[i]->id].distance);
                // 到终点
                if (next == end_node)
                    return false;
                else
                    return true;
            }
        }

        agentK.setDeadEnd();
        return false;
    }

    void update_pheromone(Agent<float> &agentK)
    {
        const std::vector<Vertex3<float> *> *path = agentK.getPath();

        int _size = path->size();

        for (int i(0); i < _size - 1; i++)
        {
            // 对称矩阵
            info_matrix[(*path)[i]->id][(*path)[i + 1]->id].info += Q / agentK.L;
            info_matrix[(*path)[i + 1]->id][(*path)[i]->id].info = info_matrix[(*path)[i]->id][(*path)[i + 1]->id].info;
        }
    }

public:
    int node_num;                          // 点数量
    Vertex3<float> ***nodes;               // 所有结点的体阵cuboid
    Vertex3<float> *start_node, *end_node; // 起始点和终点的结点
    //Read 3D nodes from files
    // bool initParamFromFile(std::string file_name)
    // { //输入
    //     printf("Read file: %s and process data type %i \n", file_name.c_str());
    //     FILE *fp = fopen(file_name.c_str(), "r");

    //     if (fp == NULL)
    //     {
    //         std::cout << "Failed to read file, reject to init." << std::endl;
    //         return false;
    //     }
    //     //总结点数量,每个维度的范围，参数初始值
    //     fscanf(fp, "%d %d %d %d", &node_num, &rangeX, &rangeY, &rangeZ);

    //     assert(rangeX & rangeY & rangeZ);

    //     // 创建结点体阵
    //     creat_all_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
    //         nodes[z][y][x].input(fp, z, y, x);
    //     });

    //     // 初始化所有参数
    //     initParam();

    //     fclose(fp);
    //     return true;
    // }

    void initParam()
    {
        alpha = 1;
        beta = 11;
        rho = 0.9;
        max_iteration = 50;
        colony_num = 50;
        pheromone_0 = 0.5;
        Q = 5;
        index_iteration = 0;
        // 根据几何属性,按顺序推入邻接结点，超出边界的结点位置推入自己
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            /* 
                1. 一个正方体结点六个面,以x,y,z下标正向为自身坐标的参考正向
                2. 沿x,y,z正向方向邻接结点下标分别为1,2,3
                3. 对应负向结点下标为4,5,6, 那么i和i+3就是一对相对的面
            */
            if(x + 1 > rangeX)//1
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x+1]);

            if(y + 1 > rangeY)//2
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y+1][x]);

            if (z + 1 > rangeZ) //3
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z+1][y][x]);

            if (x - 1 < 0) //4
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x-1]);

            if (y - 1 < 0) //5
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y-1][x]);

            if (z - 1 < 0) //6
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
            else
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z-1][y][x]);
                
            // if (x + 1 < rangeX ? nodes[z][y][x + 1].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x + 1]);

            // if (y + 1 < rangeY ? nodes[z][y + 1][x].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y + 1][x]);

            // if (z + 1 < rangeZ ? nodes[z + 1][y][x].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z + 1][y][x]);

            // if (x - 1 >= 0 ? nodes[z][y][x - 1].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x - 1]);

            // if (y - 1 >= 0 ? nodes[z][y - 1][x].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y - 1][x]);

            // if (z - 1 >= 0 ? nodes[z - 1][y][x].isFree : false)
            //     nodes[z][y][x].adjacency_nodes.push_back(&nodes[z - 1][y][x]);
        });

        // 创建信息矩阵并初始化
        info_matrix = new _Inf_of_Points_t<float> *[node_num];
        int i = 0, j = 0;
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            info_matrix[i] = new _Inf_of_Points_t<float>[node_num];
            for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z1, int y1, int x1) {
                info_matrix[i][j].distance = Vertex3<float>::calMD_3D(
                    nodes[z][y][x], nodes[z1][y1][x1]);
                info_matrix[i][j].pheromone = pheromone_0;
                // 这里由于所有可达点都是邻接点，两点间距离是一样的，启发值也一样了
                // 后面的概率公式可以约去这个常数
                //info_matrix[i][j].herustic = 1 / (info_matrix[i][j].distance + eps);
                j++;
            });
            j = 0;
            i++;
            printf("Round: %d \r", i);
        });
    }

    /*
        Use Ant Cycle modle
    */
    void computeSolution()
    {
        Vertex3<float> *cur, *next;

        // 产出蚂蚁，初始化最优蚂蚁
        agents.resize(colony_num);
        best = &agents[0];

        for (int i(0); i < max_iteration; i++)
        {
            //信息素挥发
            for (int i = 0; i < node_num; i++)
                for (int j = 0; j < node_num; j++)
                    info_matrix[i][j].pheromone *= rho;

            for_each(agents.begin(), agents.end(), [&](auto agentK) {
                cur = start_node;
                // 搜索路径
                while (selectNext(agentK, cur, next))
                {
                    cur = next;
                }
                // 更新最优解
                if (agentK.L < best->L)
                    best = &agentK;
                // 更新留下的信息素
                update_pheromone(agentK);
            });

            //重置蚂蚁
            agents.clear();
            agents.resize(colony_num);
        }
    }

    void setPoints(Point3<float> start, Point3<float> end)
    {
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            float t = 2 * precision;
            if (my_abs(start.x - nodes[z][y][x].pt.x) < t &&
                my_abs(start.y - nodes[z][y][x].pt.y) < t &&
                my_abs(start.z - nodes[z][y][x].pt.z) < t)
            {
                start_node = &nodes[z][y][x];
            }

            if (my_abs(end.x - nodes[z][y][x].pt.x) < t &&
                my_abs(end.y - nodes[z][y][x].pt.y) < t &&
                my_abs(end.z - nodes[z][y][x].pt.z) < t)
            {
                end_node = &nodes[z][y][x];
            }
        });
    }
};

#endif