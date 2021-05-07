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
#include <time.h>
#include <bits/stdc++.h>
#include <map>
#include <functional>
#include "model_grid_map.hpp"

#define INF_FLOAT (1.0/0.0)
#define INF_INT 0x3f3f3f3f
#define eps 1e-8

typedef Point3<int> Point3i;

//任意两点间的信息
template <class T>
struct _Inf_of_Points_t
{
    _Inf_of_Points_t(T a, T b, T c):distance(a), pheromone(b), info(c){}
    T distance;   //距离
    T pheromone;  //信息素
    //T herustic; //启发值
    T info;       //info = pheromone^alpha
};

template <class T>
class ACS_Node : public Vertex3<T>
{
public:
    std::vector<ACS_Node<T> *> adjacency_nodes;    // 邻接结点
    std::vector<_Inf_of_Points_t<T>> adjacency_infos; // 邻接结点的信息
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
    std::vector<ACS_Node<T> *> path; // 路径点
    std::vector<int> node_index;     // 每个路径点的选择 

public:
    std::set<unsigned long int> tabu_list; // 禁止列表（已走过结点）
    T L;                     // 路径长度

    void addNextNode(ACS_Node<T> *node, int index, T _dis)
    {
        tabu_list.insert(node->id);
        path.push_back(node);
        node_index.push_back(index);
        L += _dis;
    }

    void addStartNode(ACS_Node<T> *_start)
    {
        tabu_list.insert(_start->id);
        path.push_back(_start);
        L = 0;
    }

    void setDeadEnd()
    {
        L = INF_FLOAT;
    }

    const std::vector<ACS_Node<T> *> *getPath() const
    {
        return &path;
    }
    const std::vector<int> *nodeIndex()const
    {
        return &node_index;
    }
};

class ACS_Base : public GridMap<float>
{
private:
    int colony_num;      // 蚁群数量
    int max_iteration;   // 最大迭代次数
    float pheromone_0, Q;
    int alpha, beta;                        // pheromone, heuristic information的权重
    float rho;                              // pheromone的挥发系数
    int node_num;                           // 点数量
    ACS_Node<float> ***nodes;               // 所有结点的体阵cuboid
    ACS_Node<float> *start_node, *end_node; // 起始点和终点的结点
    std::vector<Agent<float>> agents;       // 蚁群中每只蚂蚁
    Agent<float> best;                     // 当前最优蚂蚁
    std::vector<float> path_x, path_y, path_z;
    /**
    * @brief next nodes of ant K.  
    * @param agentK ant K
    * @param cur pointer of current node
    * @param next pointer of next node
    * @retval true update next point
    * @retval false dead end/bad route or reach end point
    */
    bool selectNext(Agent<float> &agentK, ACS_Node<float>* &cur, ACS_Node<float>* &next)
    {
        std::vector<int> J;

        float prob_sum = 0, total = 0;
        // 查找所有可达点和信息素总和
        for (int i(0); i < 6; i++)
        {   // 检查邻近结点是否被这只蚂蚁走过，防止闭环
            int _id = cur->adjacency_nodes[i]->id;
            auto iter = agentK.tabu_list.find(_id);
            if (iter == agentK.tabu_list.end())
            {
                if (cur->adjacency_nodes[i] != cur && cur->adjacency_nodes[i]->isFree)
                {
                    J.push_back(i);
                    cur->adjacency_infos[i].info = power(cur->adjacency_infos[i].pheromone, alpha);
                    total += cur->adjacency_infos[i].info;
                }
            }
        }

        // 没有可达点，本路径到尽头
        if (J.empty())
        {
            agentK.setDeadEnd();
            return false;
        }

        // 计算每个点的概率,轮盘赌（roulette wheel selection）选择下一个点
        float rnd = (float)(rand()) / (float)RAND_MAX;
        rnd *= total;

        for (int i(5); i >= 0; i--)
        {
            if(i == J.back())
            {
                J.pop_back();
                prob_sum += cur->adjacency_infos[i].info;
                if (prob_sum >= rnd)
                {
                    next = cur->adjacency_nodes[i];
                    agentK.addNextNode(next, i, cur->adjacency_infos[i].distance);
                    if (next != end_node)
                        return true;
                    // 到终点
                    else
                        return false;
                }
            }
        }

        agentK.setDeadEnd();
        return false;
    }

    /*
        更新第K只蚂蚁在路径上留下的信息素
    */
    void update_pheromone(Agent<float> &agentK)
    {
        const std::vector<ACS_Node<float> *> *path = agentK.getPath();
        const std::vector<int> *next_select = agentK.nodeIndex();

        int _size = path->size();

        for (int i(0); i < _size - 1; i++)
        {
            (*path)[i]->adjacency_infos[(*next_select)[i]].pheromone += Q / agentK.L;
            // 无向图
            // if( next_select[i] < 3 )
            //     (*path)[i + 1]->adjacency_infos[next_select[i] + 3].pheromone += Q / agentK.L;
            // else
            //     (*path)[i + 1]->adjacency_infos[next_select[i] - 3].pheromone += Q / agentK.L;
        }
    }

public:
    ACS_Base(){
        nodes = NULL;
    } 

    void initFromGridMap()
    {
        alpha = 1;
        beta = 11;
        rho = 0.9;
        pheromone_0 = 0.5;
        Q = 5;
        max_iteration = 130;
        colony_num = 160;
        node_num = size_of_map();
        srand(time(0));

        Vertex3<float> ***map = ptr_grid_map();

        if(nodes != NULL)
            delete_all_nodes(nodes, rangeX, rangeY, rangeZ);

        creat_all_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            nodes[z][y][x].pt = map[z][y][x].pt;
            nodes[z][y][x].id = map[z][y][x].id;
            nodes[z][y][x].isFree = map[z][y][x].isFree;
        });

        // 根据几何属性,按顺序推入邻接结点，超出边界的结点位置推入自己
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            /* 
                1. 一个正方体结点六个面,以x,y,z下标正向为自身坐标的参考正向
                2. 沿x,y,z正向方向邻接结点下标分别为1,2,3
                3. 对应负向结点下标为4,5,6, 那么i和i+3就是一对相对的面
                4. 初始化信息素, 由于是相邻点，点间距离都设为1, herustic = 1 / (info_matrix[i][j].distance + eps);
            */
            if(x + 1 >= rangeX)//1
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0,0,0));
            }
            else
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x + 1]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }

            if (y + 1 >= rangeY) //2
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
            }
            else
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y+1][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }

            if (z + 1 >= rangeZ) //3
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
            }
            else
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z+1][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }

            if (x - 1 < 0) //4
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
            }
            else
            {    
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x-1]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }

            if (y - 1 < 0) //5
            {    
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
            }
            else
            {    
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y-1][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }

            if (z - 1 < 0) //6
            {    
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
            }
            else
            {
                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z - 1][y][x]);
                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(1, pheromone_0, 0));
            }
        });
        printf("[ACS 3D] Created %d nodes, node cubiod [x: %d, y: %d, z: %d]\r\n", size_of_map(),rangeX, rangeY, rangeZ);
    }

    /*
        Use Ant Cycle modle
    */
    void computeSolution()
    {
        ACS_Node<float> *cur, *next;

        // 初始化蚂蚁和最短距离
        agents.resize(colony_num);
        best = agents[0];
        best.L = INF_FLOAT;

        for (int i(0); i < max_iteration; i++)
        {
            printf("[ACS 3D] Computing iteration: %d | Total Progress: %d %% \r", i+1,
                     (int)((float)(i+1) / (float)max_iteration * 100));

            //信息素挥发
            for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
                for (int i(0); i < 6; i++)
                    nodes[z][y][x].adjacency_infos[i].pheromone *= rho;
            });

            for_each(agents.begin(), agents.end(), [&](auto agentK) {
                agentK.addStartNode(start_node);
                cur = start_node;
                // 搜索路径
                while (selectNext(agentK, cur, next))
                {
                    cur = next;
                }
                // 更新最优解
                if (agentK.L < best.L)
                    best = agentK;

                // 更新留下的信息素
                update_pheromone(agentK);
            });

            //重置蚂蚁
            agents.clear();
            agents.resize(colony_num);
        }

        const std::vector<ACS_Node<float> *> *path = best.getPath();
        printf("Best Path: ");
        for_each(path->begin(), path->end(), [&](auto _it) {
            path_x.push_back(_it->pt.x);
            path_y.push_back(_it->pt.y);
            path_z.push_back(_it->pt.z);
            printf("%d -> ", _it->id);
        });
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

    void plot_path()
    {
        std::map<std::string, std::string> keywords;
        keywords.insert(std::pair<std::string, std::string>("c", "grey"));
        keywords.insert(std::pair<std::string, std::string>("linewidth", "2"));
        plt::plot3(path_x, path_y, path_z, keywords);
    }
};

#endif