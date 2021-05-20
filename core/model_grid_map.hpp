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
#include "matplotlibcpp.h"
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <functional>
namespace plt = matplotlibcpp;

#define sqr(x) ((x) * (x))
#define my_abs(x) ((x) > 0 ? (x) : -(x))

enum class GRID_PROGRESS
{
    READ_MESHES,
    READ_GRID_MAP,
    CREATED_NODES,
    OPERATING_MESHES,
    WRITING_FILE
};

template <class T>
class Point3
{
public:
    Point3() : x(0), y(0), z(0) {}
    Point3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    T x;
    T y;
    T z;
    Point3<T> operator+(const Point3<T> &b)
    {
        return Point3<T>(this->x + b.x, this->y + b.y, this->z + b.z);
    }
    Point3<T> operator-(const Point3<T> &b)
    {
        return Point3<T>(this->x - b.x, this->y - b.y, this->z - b.z);
    }
    T norm()
    {
        return sqrt(sqr(x) + sqr(y) + sqr(z));
    }
    // 计算两顶点间三维曼哈顿距离（Manhattan Distance）
    static T manhattan_distance(const Point3<T> &a, const Point3<T> &b)
    {
        return (a.x - b.x) + (a.y - b.y) + (a.z - b.z);
    }
    // 计算两顶点间欧拉距离，向量模长
    static T euler_distance(const Point3<T> &a, const Point3<T> &b)
    {
        return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z));
    }
    // 计算A,B向量的点积
    static T dot(const Point3<T> &a, const Point3<T> &b)
    {
        return (a.x * b.x + a.y * b.y + a.z * b.z);
    }
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
class Vertex3
{
public:
    Point3<T> pt;  // 本顶点的坐标
    bool isFree; // 自由点标志
    unsigned long int id;      // 结点编号
};

template <class T>
void creat_all_nodes(T*** &matrix, int rx, int ry, int rz, std::function<void(int, int, int)> f)
{
    assert(matrix == NULL);
    // 创建结点体阵和初始化
    matrix = new T **[rz];
    for (int i(0); i < rz; i++)
    {
        matrix[i] = new T *[ry];
        for (int j(0); j < ry; j++)
        {
            matrix[i][j] = new T[rx];
            for (int k(0); k < rx; k++)
                //初始化操作
                f(i, j, k);
        }
    }
}

template <class T>
void for_each_nodes(T*** &matrix, int rx, int ry, int rz, std::function<void(int, int, int)> f)
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
void delete_all_nodes(T*** &matrix, int rx, int ry, int rz)
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
    Vertex3<T> ***creatGridMap(const std::vector<Triangles<T>> &mesh, T _precision, int _wall, std::string file_name="")
    {
        if(grid_map != NULL)
        {
            x_list.clear();
            y_list.clear();
            z_list.clear();
            delete_all_nodes(grid_map, rangeX, rangeY, rangeZ);
        }

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
/* 这段为了让显示更直观*/
        // T max_len = max_x - min_x;
        // max_len = max_y - min_y > max_len ? max_y - min_y:max_len;
        // max_len = max_z - min_z > max_len ? max_z - min_z:max_len;

        // x_list.push_back(0.5 * (max_x - min_x) + max_len);
        // y_list.push_back(0.5 * (max_y - min_y) + max_len);
        // z_list.push_back(0.5 * (max_z - min_z) + max_len);
        // x_list.push_back(0.5 * (max_x - min_x) - max_len);
        // y_list.push_back(0.5 * (max_y - min_y) - max_len);
        // z_list.push_back(0.5 * (max_z - min_z) - max_len);
/**/
        display_progress(GRID_PROGRESS::READ_MESHES, mesh.size());
        // 创建[0,range]的结点体阵，单位为1，其中range由栅格精度percision决定
        // 1是补偿浮点转int的截断精度, wall为边沿留出的自由空间
        rangeX = (int)((max_x - min_x) / precision) + 1 + 2 * wall;
        rangeY = (int)((max_y - min_y) / precision) + 1 + 2 * wall;
        rangeZ = (int)((max_z - min_z) / precision) + 1 + 2 * wall;

        map_size = 0;
        creat_all_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            grid_map[z][y][x].pt.x = x < wall ? min_x - (wall - x) * precision  : 
                (x >= (rangeX - wall) ? max_x + (x - rangeX + wall) * precision : min_x + (x - wall) * precision);

            grid_map[z][y][x].pt.y = y < wall ? min_y - (wall - y) * precision : 
                (y >= (rangeY - wall) ? max_y + (y - rangeY + wall) * precision : min_y + (y - wall) * precision);

            grid_map[z][y][x].pt.z = z < wall ? min_z - (wall - z) * precision :
                 (z >= (rangeZ - wall) ? max_z + (z - rangeZ + wall) * precision : min_z + (z - wall) * precision);

            grid_map[z][y][x].isFree = true;
            grid_map[z][y][x].id = map_size;
            map_size++;
        });

        display_progress(GRID_PROGRESS::CREATED_NODES, map_size);

        // TODO use mesh loop instead.
        // calculate distance from points to triangle plane
        int index = 0;
        for_each(mesh.begin(), mesh.end(), [&](auto _it) {
            T D = -(_it.vertex[0].x * _it.nor_vec.x +
                    _it.vertex[0].y * _it.nor_vec.y +
                    _it.vertex[0].z * _it.nor_vec.z);
            // 找到该三角形的三轴坐标范围和对应的下标
            min_x = _it.vertex[0].x;
            min_y = _it.vertex[0].y;
            min_z = _it.vertex[0].z;
            max_x = min_x;
            max_y = min_y;
            max_z = min_z;
            for (int i(0); i < 3; i++)
            {
                max_x = _it.vertex[i].x > max_x ? _it.vertex[i].x : max_x;
                max_y = _it.vertex[i].y > max_y ? _it.vertex[i].y : max_y;
                max_z = _it.vertex[i].z > max_z ? _it.vertex[i].z : max_z;
                min_x = _it.vertex[i].x < min_x ? _it.vertex[i].x : min_x;
                min_y = _it.vertex[i].y < min_y ? _it.vertex[i].y : min_y;
                min_z = _it.vertex[i].z < min_z ? _it.vertex[i].z : min_z;
            }
            min_x -= precision;
            min_y -= precision;
            min_z -= precision;
            max_x += precision;
            max_y += precision;
            max_z += precision;

            // 计算每个结点到该三角形面的距离,可用局部点计算代替全部点计算
            for_each_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
                T distance = grid_map[z][y][x].pt.x * _it.nor_vec.x +
                             grid_map[z][y][x].pt.y * _it.nor_vec.y +
                             grid_map[z][y][x].pt.z * _it.nor_vec.z + D;

                if (my_abs(distance) < 1.2*precision)
                {   //判断该点是否在三个顶点范围内
                    if (min_x <= grid_map[z][y][x].pt.x && grid_map[z][y][x].pt.x <= max_x &&
                        min_y <= grid_map[z][y][x].pt.y && grid_map[z][y][x].pt.y <= max_y &&
                        min_z <= grid_map[z][y][x].pt.z && grid_map[z][y][x].pt.z <= max_z)
                    {
                        grid_map[z][y][x].isFree = false;
                        x_list.push_back(grid_map[z][y][x].pt.x);
                        y_list.push_back(grid_map[z][y][x].pt.y);
                        z_list.push_back(grid_map[z][y][x].pt.z);
                    }
                }
            });

            index++;
            int progress = (float)index / (float)mesh.size() * 100;
            display_progress(GRID_PROGRESS::OPERATING_MESHES, progress, index);
        });
        
        if(file_name!="")
        {
            FILE *fp = fopen(file_name.c_str(), "w");
            fprintf(fp, "%d %d %d %d %f %d\n", map_size, rangeX, rangeY, rangeZ, precision, wall);
            fprintf(fp, "%f %f %f %f %f %f\n", min_x, min_y, min_z, max_x, max_y, max_z);
            for(int i=0; i< rangeZ; i++)
            {
                for(int j = 0; j < rangeY; j++)
                {
                    for(int k = 0; k < rangeX; k++)
                    {
                        fprintf(fp, "%d ", (int)grid_map[i][j][k].isFree);
                    }
                    fprintf(fp, "\n");
                }
            }
            fclose(fp);

            display_progress(GRID_PROGRESS::WRITING_FILE,file_name.c_str());
        }

        printf("[Grid Map] Done! \r\n");
        return grid_map;
    }

    void readGridMap(std::string file_name)
    { 
        FILE *fp = fopen(file_name.c_str(), "r");
        if (fp == NULL)
        {
            std::cout << "[Grid Map] Failed to read file, skipping..." << std::endl;
            return;
        }
        fscanf(fp, "%d %d %d %d %f %d", &map_size, &rangeX, &rangeY, &rangeZ, &precision, &wall);
        fscanf(fp, "%f %f %f %f %f %f", &min_x, &min_y, &min_z, &max_x, &max_y, &max_z);

        if(grid_map != NULL)
        {
            x_list.clear();
            y_list.clear();
            z_list.clear();
            delete_all_nodes(grid_map, rangeX, rangeY, rangeZ);
        }

        int index = 0;
        creat_all_nodes(grid_map, rangeX, rangeY, rangeZ, [&](int z, int y, int x) {
            grid_map[z][y][x].pt.x = x < wall ? min_x - (wall - x) * precision  : 
                (x >= (rangeX - wall) ? max_x + (x - rangeX + wall) * precision : min_x + (x - wall) * precision);

            grid_map[z][y][x].pt.y = y < wall ? min_y - (wall - y) * precision : 
                (y >= (rangeY - wall) ? max_y + (y - rangeY + wall) * precision : min_y + (y - wall) * precision);

            grid_map[z][y][x].pt.z = z < wall ? min_z - (wall - z) * precision :
                 (z >= (rangeZ - wall) ? max_z + (z - rangeZ + wall) * precision : min_z + (z - wall) * precision);

            grid_map[z][y][x].isFree = true;
            grid_map[z][y][x].id = index;
            index++;
        });

        for(int i=0; i< rangeZ; i++)
        {
            for(int j = 0; j < rangeY; j++)
            {
                for(int k = 0; k < rangeX; k++)
                {
                    fscanf(fp, "%d", &grid_map[i][j][k].isFree);
                    if(!grid_map[i][j][k].isFree)
                    {
                        x_list.push_back(grid_map[i][j][k].pt.x);
                        y_list.push_back(grid_map[i][j][k].pt.y);
                        z_list.push_back(grid_map[i][j][k].pt.z);
                    }
                }
                //fprintf(fp, "\n");
            }
        }

        fclose(fp);

        display_progress(GRID_PROGRESS::READ_GRID_MAP,file_name.c_str());
    }

    Vertex3<T> ***ptr_grid_map() const
    {
        return grid_map;
    }

    int size_of_map() const
    {
        return map_size;
    }

    void plot_grid_map(int figureNumber)
    {
        std::map<std::string, std::string> keywords;
        keywords.insert(std::pair<std::string, std::string>("marker", "o"));
        plt::scatter(x_list, y_list, z_list, 1, keywords,figureNumber);
    }

    void show_plot()
    {
        plt::show();
        plt::cla();
    }

    T precision;                    // 每个栅格代表的实际大小
    int wall;                       // 物体最大轮廓外留出空白圈的厚度
    int rangeX, rangeY, rangeZ;     // 每个维度的大小

private:
    Vertex3<float> ***grid_map = NULL;
    std::vector<T> x_list, y_list, z_list;
    // 区域的最值
    T min_x, min_y, min_z;
    T max_x, max_y, max_z;
    int map_size;

    void display_progress(GRID_PROGRESS prg, ...)
    {
        va_list args;

        /* args point to the first variable parameter */
        va_start(args, prg);

        switch(prg)
        {
            case GRID_PROGRESS::READ_MESHES:
                printf("[Grid Map] %d triangles is scanned... \n", (int)va_arg(args, int));
                break;
            case GRID_PROGRESS::READ_GRID_MAP:
                printf("\n[Grid Map] Successfully read grid map from %s \r\n", (char*)va_arg(args, char*));
                break;
            case GRID_PROGRESS::CREATED_NODES:
                printf("[Grid Map] %d nodes is created... \n", (int)va_arg(args, int));
                break;
            case GRID_PROGRESS::OPERATING_MESHES:
                printf("[Grid Map] Processing triangle: %d , Total Progress: %d %% \r", (int)va_arg(args, int), (int) va_arg(args, int));
                break;
            case GRID_PROGRESS::WRITING_FILE:
                printf("\n[Grid Map] Successfully write to %s \r\n", (char*)va_arg(args, char*));
                break;
        }

        va_end(args);
    }
};

#endif