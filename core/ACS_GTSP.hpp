/**
  ******************************************************************************
  * @file    ACS.hpp
  * @brief   Using Ant Colony System to solve generalized traveling salesman 
  *          problem(GTSP).
  * @author  Xing He(Huazhong University of Science and Technology)
  *          Modefied by Stuart(South China University of Technology)
  * @date    18/04/2021
  * @version 0.1
  * @note    Original version please visit: https://paste.ubuntu.com/25353205/
  *          to see more.
  ******************************************************************************
*/
#ifndef _ACS_HPP
#define _ACS_HPP
#include "model_grid_map.hpp"
#include <bits/stdc++.h>

#define INF 0x3f3f3f3f
#define eps 1e-8

typedef std::pair<int, int> pair_int;

typedef struct ACS_Tour
{                               //路径
    std::vector<pair_int> path; //path[i]，存储一条边(r,s)
    double L;

    void clean()
    {
        L = INF;
        path.clear();
        path.shrink_to_fit();
    } //清空

    void calc(double **_dis)
    {
        L = 0;
        int sz = path.size();
        for (int i = 0; i < sz-1; i++)
        {
            L += _dis[path[i].first][path[i].second];
        }
    } //计算长度

    void push_back(int x, int y)
    {
        path.push_back(std::make_pair(x, y));
    }

    int size()
    {
        return (int)path.size();
    }

    int r(int i)
    {
        return path[i].first;
    }

    int s(int i)
    {
        return path[i].second;
    }

    void print()
    {
        int sz = path.size();
        for (int i = 0; i < sz; i++)
        {
            printf("%d->", path[i].first + 1);
        }
        printf("%d\n", path[sz - 1].second + 1);
    }

    bool operator<(const ACS_Tour &a) const
    {
        return L < a.L;
    } //重载
} ACS_Tour;

class ACS_GTSP
{
private:
    int type;                   // type == 1 全矩阵, type == 2 二维欧拉距离
    int city_num;               // 城市数量
    double **dis;               // 城市间距离
    double **pheromone;         // 信息素
    double **herustic;          // 启发式值
    double **info;              // info = pheromone ^ delta * herustic ^ beta
    double pheromone_0;         // pheromone初始值，这里是1 / (avg * N)其中avg为
                                // 图网中所有边边权的平均数。
    int colony_num;             // 种群数量
    int delta, beta;            // delta 和 beta分别表示pheromones 和 herustic的比重
    double alpha;               // evaporation parameter，挥发参数，每次信息素要挥发的量
    int *r1, *s, *r;            // agent k的出发城市，下一个点，当前点。
    int MAX_itera, index_itera; //最大迭代次数，迭代计数变量
    std::set<int> empty, *J;
    ACS_Tour *tour, best;
    Vertex3<float> *node;
    bool init_flag;

    void reset()
    {
        tour = new ACS_Tour[colony_num + 5];
        for (int i = 0; i < city_num; i++)
        {
            tour[i].clean();
            r1[i] = i; //初始化出发城市
            J[i] = empty;
            J[i].erase(r1[i]); //初始化agent i需要访问的城市
            r[i] = r1[i];      //当前在出发点
        }
        for (int i = 0; i < city_num; i++)
            for (int j = 0; j < city_num; j++)
            { //选择公式
                info[i][j] = power(pheromone[i][j], delta) *
                             power(herustic[i][j], beta);
            }
    }

    int select_next(int k)
    {
        if (J[k].empty())
            return r1[k];                                 //如果J是空的，那么返回出发点城市
        double rnd = (double)(rand()) / (double)RAND_MAX; //产生0..1的随机数
        std::set<int>::iterator it = J[k].begin();
        double sum_prob = 0, sum = 0;
        for (; it != J[k].end(); it++)
        {
            sum += info[r[k]][*it];
        } //计算概率分布
        rnd *= sum;
        it = J[k].begin();
        for (; it != J[k].end(); it++)
        { //依照概率选取下一步城市
            sum_prob += info[r[k]][*it];
            if (sum_prob >= rnd)
            {
                return *it;
            }
        }
        return r1[k];
    }

    void construct_solution()
    {
        for (int i = 0; i < city_num; i++)
        {
            for (int k = 0; k < colony_num; k++)
            {
                int next = select_next(k); //选择下一步的最优决策
                J[k].erase(next);
                s[k] = next;
                tour[k].push_back(r[k], s[k]);
                r[k] = s[k];
            }
        }
    }

    void update_pheromone()
    {
        ACS_Tour now_best;
        now_best.clean(); //初始化
        for (int i = 0; i < colony_num; i++)
        {
            tour[i].calc(dis);
            if (tour[i] < now_best)
                now_best = tour[i]; //寻找当前迭代最优解
        }
        if (now_best < best)
        {
            best = now_best; //更新最优解
        }
        for (int i = 0; i < city_num; i++)
            for (int j = 0; j < city_num; j++)
                pheromone[i][j] *= (1 - alpha); //信息素挥发

        int sz = now_best.size();
        for (int i = 0; i < sz; i++)
        {
            pheromone[now_best.r(i)][now_best.s(i)] += 1. / (double)now_best.L;
            pheromone[now_best.s(i)][now_best.r(i)] = pheromone[now_best.r(i)][now_best.s(i)]; //对称
        }                                                                                      //更新信息素含量
    }

    void init_param()
    {
        alpha = 0.1;
        delta = 1;
        beta = 6;
        colony_num = city_num;
        pheromone = new double *[city_num + 5];
        herustic = new double *[city_num + 5];
        info = new double *[city_num + 5];
        r1 = new int[city_num + 5];
        r = new int[city_num + 5];
        s = new int[city_num + 5];
        J = new std::set<int>[city_num + 5];
        empty.clear();
        for (int i = 0; i < city_num; i++)
        {
            pheromone[i] = new double[city_num + 5];
            herustic[i] = new double[city_num + 5];
            info[i] = new double[city_num + 5];
            empty.insert(i);
            for (int j = 0; j < city_num; j++)
            {
                pheromone[i][j] = pheromone_0;
                //加一个小数eps，防止被零除
                herustic[i][j] = 1 / (dis[i][j] + eps);
            }
        }
        best.clean();
        index_itera = 0;
        MAX_itera = city_num * city_num;
        init_flag = true;
    }

public:
    /*构造函数初始化参数*/
    std::vector<float> g_path_x, g_path_y, g_path_z;
    // 无向图读取
    bool readFromGraphFile(std::string filename)
    {
        FILE *fp = fopen(filename.c_str(), "r");
        double tmp = 0;
        int cnt = 0;
        fscanf(fp, "%d %d", &city_num, &cnt);

        dis = new double *[city_num];
        for(int i = 0; i < city_num; i++)
        {
            dis[i] = new double[city_num];
            for (int j = i + 1; j < city_num; j++)
                dis[i][j] = 0;
        }

        for (int i = 0; i < city_num; i++)
        {
            for (int j = i + 1; j < city_num; j++)
            {
                fscanf(fp, "%lf", &dis[i][j]);
                dis[j][i] = dis[i][j];  //无向图为对称矩阵
                printf("distance: %lf \r\n", dis[i][j]);
                tmp += dis[i][j];       // i == j的时候 dis不存在，所以不考虑。
            }
        }
        pheromone_0 = (double)cnt / (tmp * city_num);
        fclose(fp);
        init_param();
        return true;
    }

    bool computeSolution()
    {
        if (init_flag)
        {
            double last = INF;
            int bad_times = 0;
            for (; index_itera < MAX_itera; index_itera++)
            {
                if (bad_times > city_num)
                    break;            //进入局部最优
                reset();              //初始化agent信息
                construct_solution(); //对于所有的agent构造一个完整的tour
                update_pheromone();   //更新信息素
                printf("iteration %d:Best so far = %.2lf\n", index_itera, best.L);
                if (last > best.L)
                {
                    last = best.L;
                    bad_times = 0;
                }
                else
                    bad_times++; //记录当前未更新代数，若迭代多次未更新，认为进入局部最优
            }
            printf("Best in all = %.2lf\n", best.L); //输出目标值
            best.print();                            //输出路径

            return true;
        }
        else
            return false;
    }

    void read_all_segments(Agent<float>** &best_matrix)
    {
        for(int i = 0; i < best.path.size()-1; i++)
        {
            const std::vector<ACS_Node<float> *> *segment = best_matrix[best.path[i].first][best.path[i].second].getPath();
            for(int j = 0; j < segment->size(); j++)
            {
                g_path_x.push_back((*segment)[j]->pt.x);
                g_path_y.push_back((*segment)[j]->pt.y);
                g_path_z.push_back((*segment)[j]->pt.z);
            }
        }
    }

    /*
        i starts from 1.
    */
    void read_segment(Agent<float>** best_matrix, int i)
    {
        const std::vector<ACS_Node<float> *> *segment = best_matrix[best.path[i-1].first][best.path[i-1].second].getPath();
        for(int j = 0; j < segment->size(); j++)
        {
            g_path_x.push_back((*segment)[j]->pt.x);
            g_path_y.push_back((*segment)[j]->pt.y);
            g_path_z.push_back((*segment)[j]->pt.z);
        }
    }

    int path_segment_nums()
    {
        return best.path.size() - 1;
    }

    void plot_route_path(int figureNumber)
    {
        std::map<std::string, std::string> keywords;
        keywords.insert(std::pair<std::string, std::string>("c", "gray"));
        //keywords.insert(std::pair<std::string, std::string>("linewidth", "4"));
        keywords.insert(std::pair<std::string, std::string>("marker", "o"));
        plt::scatter(g_path_x, g_path_y, g_path_z, 1,keywords,figureNumber);
    }
    ~ACS_GTSP(){};
};

#endif
