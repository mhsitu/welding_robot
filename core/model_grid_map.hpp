/**
  ******************************************************************************
  * @file    model_grid_map.hpp
  * @brief   3D enviroment modeling with grid method.
  * @author  Stuart(South China University of Technology)
  *          1356046979@qq.com
  * @date    26/04/2021
  * @version 0.1
  ******************************************************************************
*/
#ifndef _MODEL_GRID_MAP_HPP
#define _MODEL_GRID_MAP_HPP
#include <assert.h>
#include <vector>
#include <functional>

#define sqr(x) ((x) * (x))
#define my_abs(x) ((x) > 0 ? (x) : ((x) <= 0 ? -(x) : (x)))

template <class T>
class Point3
{
public:
    Point3() : x(0), y(0), z(0) {}
    Point3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    T x;
    T y;
    T z;
};

template <class T>
struct Triangles
{
public:
    Point3<T> nor_vec;   // 三角平面的法向量
    Point3<T> vertex[3]; // 三角形的三个顶点
    int trait;
};

template <class T>
struct Vertex3
{
    std::vector<Vertex3<T> *> adjacency_nodes;
    // 邻接的结点
    Point3<T> pt;  // 本节点的坐标
    bool isFree; // 自由点标志
    int id;      // 结点编号

    int input(FILE *fp, T _z, T _y, T _x)
    {
        //暂时令坐标等于下标
        pt.x = _x;
        pt.y = _y;
        pt.z = _z;
        return fscanf(fp, "%d %d", &id, &isFree);
    }

    // 计算两顶点间三维曼哈顿距离（Manhattan Distance）
    static T calMD_3D(const Vertex3<T> &a, const Vertex3<T> &b)
    {
        return (a.pt.x - b.pt.x) + (a.pt.y - b.pt.y) + (a.pt.z - b.pt.z);
    }
    // 计算两顶点间欧拉距离
    static T calEUC_3D(const Vertex3<T> &a, const Vertex3<T> &b)
    {
        return sqrt(sqr(a.pt.x - b.pt.x) + sqr(a.pt.y - b.pt.y) + sqr(a.pt.z - b.pt.z));
    }
};

template <class T>
void creat_all_nodes(T ***matrix, int rx, int ry, int rz, std::function<void(int, int, int)> f)
{
    assert(matrix != NULL);
    // 创建结点体阵和初始化
    matrix = new T **[rz];
    for (int i(0); i < rz; i++)
    {
        matrix[i] = new T *[ry];
        for (int j(0); j < ry; j++)
        {
            matrix[i][i] = new T[rx];
            for (int k(0); k < rz; k++)
                //初始化操作
                f(i, j, k);
        }
    }
}

template <class T>
void for_each_nodes(T ***matrix, int rx, int ry, int rz, std::function<void(int, int, int)> f)
{
    assert(matrix != NULL);
    for (int i(0); i < rz; i++)
    {
        for (int j(0); j < ry; j++)
        {
            for (int k(0); k < rx; k++)
                //对每个结点的操作
                f(i, j, k);
        }
    }
}

template <class T>
void delete_all_nodes(T ***matrix, int rx, int ry, int rz)
{
    assert(matrix != NULL);

    for (int i(0); i < rz; i++)
    {
        for (int j(0); j < ry; j++)
        {
            delete matrix[i][j];
        }
        delete matrix[i];
    }
    delete matrix;
}

template <class T>
class GridMap
{
public:
    /**
     * @brief create grid map from meshes
     * 
     * @param mesh 
     * @param percision 
     * @param wall 
     */
    Vertex3<float> ***creatGridMap(const std::vector<Triangles<T>> &mesh, T _precision, int _wall)
    {
        Vertex3<float> ***grid_map;
        T min_x, min_y, min_z;
        T max_x, max_y, max_z;

        precision = _precision;
        wall = _wall;

        // 遍历所有三角形，找到三轴坐标范围
        min_x = mesh[0].vertex[0].x;
        min_y = mesh[0].vertex[0].y;
        min_z = mesh[0].vertex[0].z;
        max_x = min_x;
        max_y = min_y;
        max_z = min_z;

        for_each(mesh.begin(), mesh.end(), [&](auto _it) {
            for (int i(0); i < 3; i++)
            {
                max_x = _it.vertex[i].x > max_x ? _it.vertex[i].x : max_x;
                max_y = _it.vertex[i].y > max_y ? _it.vertex[i].y : max_y;
                max_z = _it.vertex[i].z > max_z ? _it.vertex[i].z : max_z;
                min_x = _it.vertex[i].x < min_x ? _it.vertex[i].x : min_x;
                min_y = _it.vertex[i].y < min_y ? _it.vertex[i].y : min_y;
                min_z = _it.vertex[i].z < min_z ? _it.vertex[i].z : min_z;
            }
        });

        // 创建[0,range]的结点体阵，单位为1，其中range由栅格精度percision决定
        rangeX = (max_x - min_x) / precision + 1 + 2 * wall;
        rangeY = (max_y - min_y) / precision + 1 + 2 * wall;
        rangeZ = (max_z - min_z) / precision + 1 + 2 * wall;

        int index = 0;
        creat_all_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            grid_map[z][y][x].isFree = true;
            grid_map[z][y][x].id = index;
            index++;
        });

        for (int i(wall); i < rangeZ - wall; i ++)
        {
            for (int j(wall); i < rangeY - wall; j++)
            {
                for (int k(wall); k < rangeX - wall; k++)
                {
                    grid_map[i][j][k].pt.x = min_x + k * precision;
                    grid_map[i][j][k].pt.y = min_y + j * precision;
                    grid_map[i][j][k].pt.z = min_z + i * precision;
                }
            }
        }
        for_each_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            grid_map[z][y][x].pt.x = x < wall ? min_x - precision : 
            (x > wall ? max_x + precision : grid_map[z][y][x].pt.x = min_x + x * precision);

            grid_map[z][y][x].pt.y = y < wall ? min_y - precision :
             (y > wall ? max_y + precision : grid_map[z][y][x].pt.y = min_y + y * precision);

            grid_map[z][y][x].pt.z = z < wall ? min_z - precision :
             (z > wall ? max_z + precision : grid_map[z][y][x].pt.z = min_z + z * precision);
        });

        // TODO use mesh loop instead.
        // calculate distance from points to triangle plane
        for_each(mesh.begin(), mesh.end(), [&](auto _it) {
            T D = -(_it.vertex[0].x * _it.nor_vec.x +
                    _it.vertex[0].y * _it.nor_vec.y +
                    _it.vertex[0].z * _it.nor_vec.z);
            T min_dis = precision * sqrt(sqr(_it.nor_vec.x) + sqr(_it.nor_vec.y) + sqr(_it.nor_vec.z));

            for_each_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
                if (my_abs(grid_map[z][y][x].pt.x * _it.nor_vec.x +
                           grid_map[z][y][x].pt.y * _it.nor_vec.y +
                           grid_map[z][y][x].pt.z * _it.nor_vec.z + D) < min_dis)
                {
                    grid_map[z][y][x].isFree = false;
                }
            });
        });

        grid_map_list.push_back(grid_map);

        return grid_map;
    }

    T precision;                    // 每个栅格代表的实际大小
    T wall;                         // 物体最大轮廓外留出空白圈的厚度
    int rangeX, rangeY, rangeZ;     // 每个维度的大小

    std::vector<Vertex3<float> ***> grid_map_list;
};

#endif