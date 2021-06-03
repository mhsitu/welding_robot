/**
  ******************************************************************************
  * @file    ACSRank_3D.hpp
  * @brief   Using Rank Based Ant Colony System to search a path in 3D 
  *          grid-based map, the ACSRnk_3D is optimized to automatically choose
  *          searching parameters.
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
    _Inf_of_Points_t(){}
    _Inf_of_Points_t(T a, T b, T c) : distance(a), pheromone(b), info(c) {}
    T distance;  //距离
    T pheromone; //信息素
    T info;      //info = pheromone^alpha
};

template <class T>
class ACS_Node : public Vertex3<T>
{
public:
    std::vector<ACS_Node<T> *> adjacency_nodes;       // 邻接结点
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
    T L;                                   // 路径长度

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
    const std::vector<int> *nodeIndex() const
    {
        return &node_index;
    }
    bool findPathNode(ACS_Node<T> *target)
    {
        for(auto node:path)
        {
            if(target == node) return true;
        }
        return false;
    }
};

class ACS_Rank : public GridMap<float>
{
private:
    int colony_num;    // 蚁群数量
    int max_iteration; // 最大迭代次数
    float pheromone_0, Q;
    int alpha;                              // pheromone的权重
    float beta,rho, lambda;                 // 先验知识权重，pheromone的挥发系数,蚁群中精英蚂蚁的比例
    int node_num;                           // 点数量
    ACS_Node<float> ***nodes;               // 所有结点的体阵cuboid
    ACS_Node<float> *start_node, *end_node; // 起始点和终点的结点
    std::vector<Agent<float>> agents;       // 蚁群中每只蚂蚁
    Agent<float> best;                      // 当前最优蚂蚁
    std::vector<float> path_x, path_y, path_z;
  
    /**
    * @brief next nodes of ant K.  
    * @param agentK ant K
    * @param cur pointer of current node
    * @param next pointer of next node
    * @retval true update next point
    * @retval false dead end/bad route or reach end point
    */
    bool selectNext(Agent<float> &agentK, ACS_Node<float> *&cur, ACS_Node<float> *&next)
    {
        std::vector<int> J;
        Point3<float> vector_a = end_node->pt - cur->pt;
        float prob_sum = 0, total = 0;

        // 查找所有可达点和信息素总和
        int _adaj_size = cur->adjacency_nodes.size();
        for (int i(0); i < _adaj_size; i++)
        { // 检查邻近结点是否被这只蚂蚁走过，防止闭环
            int _id = cur->adjacency_nodes[i]->id;
            auto iter = agentK.tabu_list.find(_id);
            if (iter == agentK.tabu_list.end())
            {
                if (cur->adjacency_nodes[i] != cur && cur->adjacency_nodes[i]->isFree)
                {
                    // 加入先验信息：该点与当前点向量在目标点方向上的余弦值
                    Point3<float> vector_b = cur->adjacency_nodes[i]->pt - cur->pt;
                    float cos = Point3<float>::dot(vector_a, vector_b) / (vector_a.norm() * vector_b.norm());
                    //cur->adjacency_infos[i].info = power(cur->adjacency_infos[i].pheromone, alpha);
                    cur->adjacency_infos[i].info = power(cur->adjacency_infos[i].pheromone, alpha) * (1 + beta * cos);
                    total += cur->adjacency_infos[i].info;
                    J.push_back(i);
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

        for (int i(_adaj_size-1); i >= 0; i--)
        {
            if (i == J.back())
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
    void update_pheromone(Agent<float> &agentK, int order)
    {
        if(agentK.L == INF_FLOAT || order > lambda - 1)
            return;
        const std::vector<ACS_Node<float> *> *path = agentK.getPath();
        const std::vector<int> *next_select = agentK.nodeIndex();

        int _size = path->size();
        bool isOnBestPath;
        for (int i(0); i < _size - 1; i++)
        {
            isOnBestPath = best.findPathNode((*path)[i]) && best.findPathNode((*path)[i]->adjacency_nodes[(*next_select)[i]]);
            (*path)[i]->adjacency_infos[(*next_select)[i]].pheromone +=
                (lambda - order) * Q / agentK.L + static_cast<float>(isOnBestPath) * lambda * Q / best.L;

            //(*path)[i]->adjacency_infos[(*next_select)[i]].pheromone += Q / agentK.L;
        }
    }

    /*
        使用 Ant Cycle model
    */
    void computeSolution(float predict_path_len)
    {
        ACS_Node<float> *cur, *next;
        std::vector<float> _path_index;
        std::vector<float> _path_lens;
        float last_path;
        int local_min = 0;

        // 初始化蚂蚁和最短距离
        path_x.clear();
        path_y.clear();
        path_z.clear();
        best.L = INF_FLOAT;
        last_path = INF_FLOAT;

        clock_t time = clock();

        for (int i(0); i < max_iteration; i++)
        {
            printf("[ACS 3D] Computing iteration: %d | Total Progress: %d%% | Search time: %.1f \r", i + 1,
                   (int)((float)(i + 1) / (float)max_iteration * 100), (float)(clock() - time)/CLOCKS_PER_SEC);
            /*
                迭代参数优化
            */
            // 每次迭代都更改Q为当前最佳路径的x倍，x才是真正意义上的Q，但此时Q与路径长度无关
            // x / L为每次蚂蚁走过后期望增加的信息素总值，这个值应与信息素初值接近
            // 蚂蚁数量约为搜索总步数的1/2~1/3
            colony_num = 0.35 * (best.L < predict_path_len ?  best.L : predict_path_len) / precision;
            lambda = 0.2 * colony_num;
            Q = pheromone_0 / lambda * (best.L == INF_FLOAT ? predict_path_len : best.L);
            agents.resize(colony_num);

            for(auto& agentK:agents)
            {
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
            }

            // 信息素挥发-更新留下的信息素(更新部分)-重置蚂蚁
            for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x)
            {
                for (int k(0); k < 6; k++)
                    nodes[z][y][x].adjacency_infos[k].pheromone *= rho;
            });
            std::sort(agents.begin(), agents.end(), [](Agent<float> &a, Agent<float> &b)->bool
                      { return a.L < b.L; });
            int agent_order = 1;
            for(auto &agentK: agents)
            {
                update_pheromone(agentK, agent_order);
                agent_order++;
            }

            // 检测到搜索停滞，局部最优后清空信息素
            if(best.L == last_path)
                local_min++;
            else
                local_min = 0;
            if(local_min >= 50)
            {
                //reset();
                local_min = 0;
            }
            last_path = best.L;
            agents.clear();

            _path_index.push_back(i);
            _path_lens.push_back(best.L);

            //printf("Path length: %.4f, colony num %d \n", _path_lens[i], colony_num);
        }
        printf("\n");
        //plt::plot(_path_index, _path_lens, "b-");
        //plot_grid_map(3);
        //plot_path(best,2);
        //show_plot();
    }

    void reset()
    {
        //信息素初始化
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x)
        {
            for (int i(0); i < 6; i++)
                nodes[z][y][x].adjacency_infos[i].pheromone = pheromone_0;
        });
    }

    void initFromGridMap()
    {
        alpha = 1;
        beta = 0.6;
        rho = 0.8;
        max_iteration = 150;// 自动收敛
        colony_num = 80;    // 根据路径长度自适应
        pheromone_0 = 1;
        Q = 50;
        node_num = size_of_map();
        srand(time(0));

        Vertex3<float> ***map = ptr_grid_map();
        assert(map != NULL);

        if (nodes != NULL)
            delete_all_nodes(nodes, rangeX, rangeY, rangeZ);

        creat_all_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x)
                        {
                            nodes[z][y][x].pt = map[z][y][x].pt;
                            nodes[z][y][x].id = map[z][y][x].id;
                            nodes[z][y][x].isFree = map[z][y][x].isFree;
                        });

        // 根据几何属性,按顺序推入邻接结点，超出边界的结点位置推入自己
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x)
        {
            /* 
                1. 一个正方体结点六个面,以x,y,z下标正向为自身坐标的参考正向
                2. 沿x,y,z正向方向邻接结点下标分别为1,2,3
                3. 对应负向结点下标为4,5,6, 那么i和i+3就是一对相对的面
                4. 初始化信息素, 由于是相邻点，点间距离都设为1, herustic = 1 / (info_matrix[i][j].distance + eps);
            */

            int _pt_type;
            float distance;
            //遍历以本结点为中心的边长为3的空间立方体，Z
            for (int i(-1); i <= 1; i++)
            {   // Y
                for (int j(-1); j <= 1;j++)
                {   // X
                    for (int k(-1); k <= 1; k++)
                    {   // 顶角邻格,三个数全不为0
                        _pt_type = i * j * k != 0 ? 3 :
                        // 邻面边邻格，有且只有一个为0
                        (i == 0 && j * k != 0) ||(j == 0 && i * k != 0) || (k == 0 && i * j != 0) ? 2 :
                        // 自己 / 邻面中间点
                        (i == 0 && j == 0 && k == 0) ? 0 : 1;

                        /*
                            下面是为完整立方体写的，第2，3种情况不为0的话就是完整的26个立方体结点；
                            考虑到这样路径会更加连续，但是收敛时间会翻几倍，还没找到很好的处理方法；
                            暂时只按6个面来搜索
                        */
                        switch(_pt_type)
                        {
                            case 0:
                                distance = 0;
                                break;
                            case 1:
                                distance = precision;
                                break;
                            case 2:
                                distance = 0; //precision * 1.414f;
                                break;
                            case 3:
                                distance = 0; //precision * 1.732f;
                                break;
                            default:
                                printf("Error point type: %d, %d, %d and index: %d, %d, %d", z, y, x, i, j, k);
                        }
                        if(distance != 0){
                            // 任意一维超出边界该点都为无效点
                            if(x + k >= rangeX || x + k < 0 
                                || y + j >= rangeY || y + j < 0
                                || z + i >= rangeZ || z + i < 0)
                            {
                                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z][y][x]);
                                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(0, 0, 0));
                            }
                            else
                            {
                                nodes[z][y][x].adjacency_nodes.push_back(&nodes[z+i][y+j][x+k]);
                                nodes[z][y][x].adjacency_infos.push_back(_Inf_of_Points_t<float>(distance, pheromone_0, 0));
                            }
                        }

                    }
                }
            }
        });
        printf("[ACS 3D] Created %d nodes, node cubiod [x: %d, y: %d, z: %d]\r\n", size_of_map(), rangeX, rangeY, rangeZ);
    }

public:
    Agent<float> **best_matrix;  
    std::vector<Point3<float>> route_points;
    ACS_Rank()
    {
        nodes = NULL;
    }

    /**
    * @brief Search best path between given points  
    * @param predict_path_len single path length
    * @param read_file file name to read search points
    * @param output_file graph file to record
    * @retval void
    */
    void searchBestPathOfPoints(float predict_path_len = 10, std::string read_file = "", std::string output_file = "")
    {
        int point_num = 0;

        if(read_file == "")
        {
            std::cout << "[ACS 3D] Please enter passing point number: ";
            std::cin >> point_num;
            route_points.resize(point_num);
            std::cout << "[ACS 3D] Please enter passing point in order: " << std::endl;
            for (int i = 0; i < point_num; i++)
            {
                std::cin >> route_points[i].x >> route_points[i].y >> route_points[i].z;
            }
        }
        else
        {
            FILE *fp = fopen(read_file.c_str(), "r");  
            if (fp == NULL)
            {
                std::cout << "[ACS 3D] Failed to read file, reject to init." << std::endl;
                return;
            }
            fscanf(fp, "%d", &point_num);
            route_points.resize(point_num);
            for(int i = 0; i < point_num; i++)
            {
                fscanf(fp, "%f %f %f", &route_points[i].x, &route_points[i].y, &route_points[i].z);
            }
            best_matrix = new Agent<float>*[point_num];
            for(int i = 0; i < point_num; i++)
            {
                best_matrix[i] = new Agent<float>[point_num];
            }
            fclose(fp);
        }

        // 初始化参数和信息链表
        initFromGridMap();
        checkRoutePoints();

        // 蚁群搜索无碰撞路径，两两之间的距离都算一次
        FILE *fp = fopen(output_file.c_str(), "w");
        fprintf(fp, "%d %d\n",0,0);
        int dis_num = 0;
        for(int i=0; i<point_num;i++)
        {
            for(int j = i+1; j < point_num;j++)
            {
                if (setPoints(route_points[i], route_points[j]))
                {
                    //SearchRoute.setPoints(Point3f(-1.2, -1.2, 0), Point3f(1.2, 1.2, 2.2));
                    //SearchRoute.setPoints(Point3f(2.36, 0, 0.9), Point3f(3.2, -0.2, 0.9));
                    computeSolution(predict_path_len);
                    reset();
                    best_matrix[i][j] = best;
                    best_matrix[j][i] = best;
                    printf("[ACS 3D] <Point (%.3f, %.3f, %.3f) : Point (%.3f, %.3f, %.3f)> Path length: %.3f\r\n", 
                            route_points[i].x, route_points[i].y, route_points[i].z,
                            route_points[j].x, route_points[j].y, route_points[j].z,
                            best.L);
                    fprintf(fp, "%.3f\n", best.L);
                    dis_num++;
                }
                else
                {
                    printf("[ACS 3D] Wrong point : (%.3f, %.3f, %.3f) or (%.3f, %.3f, %.3f), program will exit immediately \r\n",
                            route_points[i].x, route_points[i].y, route_points[i].z,
                            route_points[j].x, route_points[j].y, route_points[j].z);
                    return;
                }
            }
        }
        rewind(fp);
        fprintf(fp, "%d %d\r", point_num,dis_num);
        fclose(fp);
        printf("[ACS 3D] %d Result has been written to \"%s\" \r\n", dis_num, output_file.c_str());
    }

    const Agent<float> *getSolution() const
    {
        return &best;
    }

    void checkRoutePoints()
    {
        float t =  1.2*precision;
        bool isFind;
        for(int i = 0; i < route_points.size(); i++)
        {
            isFind = false;
            for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
                if (my_abs(route_points[i].x - nodes[z][y][x].pt.x) < t &&
                    my_abs(route_points[i].y - nodes[z][y][x].pt.y) < t &&
                    my_abs(route_points[i].z - nodes[z][y][x].pt.z) < t &&
                    nodes[z][y][x].isFree)
                {
                    start_node = &nodes[z][y][x];
                    isFind = true;
                    return;
                }
            });

            if(!isFind)
                printf("[ACS 3D] Invalid route point, please reset point(%.3f, %.3f, %.3f) \n",
                         route_points[i].x, route_points[i].y, route_points[i].z);
        }
        printf("[ACS 3D] %d route points have been checked. \n", route_points.size());
    }

    bool setPoints(Point3<float> &start, Point3<float> &end)
    {
        int findx = 0;
        start_node = NULL;
        end_node = NULL;
        for_each_nodes(nodes, rangeX, rangeY, rangeZ, [&](int z, int y, int x)
        {
            float t =  1.2*precision;
            if (my_abs(start.x - nodes[z][y][x].pt.x) < t &&
                my_abs(start.y - nodes[z][y][x].pt.y) < t &&
                my_abs(start.z - nodes[z][y][x].pt.z) < t &&
                nodes[z][y][x].isFree)
            {
                start_node = &nodes[z][y][x];
                findx++;
            }

            if (my_abs(end.x - nodes[z][y][x].pt.x) < t &&
                my_abs(end.y - nodes[z][y][x].pt.y) < t &&
                my_abs(end.z - nodes[z][y][x].pt.z) < t &&
                nodes[z][y][x].isFree)
            {
                end_node = &nodes[z][y][x];
                findx++;
            }
        });

        return (findx >= 2 && start_node != NULL && end_node != NULL) ? true : false;
    }

    void plot_path(Agent<float> &agentK, int figureNumber)
    {
        const std::vector<ACS_Node<float> *> *path = agentK.getPath();
        for_each(path->begin(), path->end(), [&](auto _it)
        {
            path_x.push_back(_it->pt.x);
            path_y.push_back(_it->pt.y);
            path_z.push_back(_it->pt.z);
        });

        std::map<std::string, std::string> keywords;
        keywords.insert(std::pair<std::string, std::string>("c", "red"));
        //keywords.insert(std::pair<std::string, std::string>("marker", "o"));
        keywords.insert(std::pair<std::string, std::string>("linewidth", "2"));
        //plt::scatter(path_x, path_y, path_z, 1, keywords,2);
        plt::plot3(path_x, path_y, path_z, keywords,figureNumber);
    }

    void plot_route_point(int figureNumber)
    {
        std::map<std::string, std::string> keywords;
        keywords.insert(std::pair<std::string, std::string>("c", "red"));
        keywords.insert(std::pair<std::string, std::string>("marker", "o"));
        
        for_each(route_points.begin(), route_points.end(), [&](auto _it)
        {
            path_x.push_back(_it.x);
            path_y.push_back(_it.y);
            path_z.push_back(_it.z);
        });
        plt::scatter(path_x, path_y, path_z, 3, keywords,figureNumber);
    }
};

#endif