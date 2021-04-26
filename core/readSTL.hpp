/**
  ******************************************************************************
  * @file    readSTL.hpp
  * @brief   Class to read meshes from STL file.
  * @author  Stuart(South China University of Technology)
  *          1356046979@qq.com
  * @date    23/04/2021
  * @version 0.1
  ******************************************************************************
*/
#ifndef _READ_STL_HPP
#define _READ_STL_HPP
#include <windows.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>
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

typedef Point3<float> Point3f;

class STLReader
{
public:
    bool ReadFile(std::string file_name)
    {
        char *buffer;
        long lSize;
        size_t result;

        /* 若要一个byte不漏地读入整个文件，只能采用二进制方式打开 */
        FILE *pFile = fopen(file_name.c_str(), "rb");
        if (pFile == NULL)
        {
            fputs("File error", stderr);
            exit(1);
        }

        /* 获取文件大小 */
        fseek(pFile, 0, SEEK_END);
        lSize = ftell(pFile);
        rewind(pFile);

        /* 分配内存存储整个文件 */
        buffer = (char *)malloc(sizeof(char) * lSize);
        if (buffer == NULL)
        {
            fputs("Memory error", stderr);
            exit(2);
        }

        /* 将文件拷贝到buffer中 */
        result = fread(buffer, 1, lSize, pFile);
        if (result != lSize)
        {
            fputs("Reading error", stderr);
            exit(3);
        }

        /* 结束演示，关闭文件并释放内存 */
        fclose(pFile);

        std::ios::sync_with_stdio(false);
        if (buffer[79] != '\0') //判断格式
        {
            ReadASCII(buffer);
        }
        else
        {
            ReadBinary(buffer);
        }
        std::ios::sync_with_stdio(true);

        free(buffer);
        return true;
    }
    int NumTri()
    {
        return unTriangles;
    }

    std::vector<Point3f> &PointList()
    {
        return pointList;
    }

    const std::vector<Triangles<float>> &TriangleList()
    {
        return triangleList;
    }

private:
    std::vector<Point3f> pointList;
    std::vector<Triangles<float>> triangleList;
    unsigned int unTriangles;
    char *memwriter;

    bool ReadASCII(const char *buffer)
    {
        unTriangles = 0;
        float x, y, z;
        int i;
        std::string name, useless;
        std::stringstream ss(buffer);
        ss >> name >> name;
        ss.get();
        Triangles<float> tri;
        do
        {
            ss >> useless;
            if (useless != "facet")
                break;
            getline(ss, useless);
            getline(ss, useless);
            for (i = 0; i < 3; i++)
            {
                ss >> useless >> tri.vertex[i].x >> tri.vertex[i].y >> tri.vertex[i].z;
                // ss >> useless >> x >> y >> z;
                // pointList.push_back(Point3f(x, y, z));
            }
            triangleList.push_back(tri);
            unTriangles++;
            getline(ss, useless);
            getline(ss, useless);
            getline(ss, useless);
        } while (1);
        return true;
    }

    bool ReadBinary(const char *buffer)
    {
        const char *p = buffer;
        char name[80];
        int i, j;
        memcpy(name, p, 80);
        p += 80;
        unTriangles = cpyint(p);
        Triangles<float> tri;
        for (i = 0; i < unTriangles; i++)
        {
            //p += 12;                //跳过头部法向量
            tri.nor_vec.x = cpyfloat(p);
            tri.nor_vec.y = cpyfloat(p);
            tri.nor_vec.z = cpyfloat(p);
            for (j = 0; j < 3; j++) //读取三顶点
            {
                tri.vertex[j].x = cpyfloat(p);
                tri.vertex[j].y = cpyfloat(p);
                tri.vertex[j].z = cpyfloat(p);
                //pointList.push_back(Point3f(cpyfloat(p), cpyfloat(p), cpyfloat(p)));
            }
            //p += 2; //跳过尾部标志
            tri.trait = cpyint(p);
            triangleList.push_back(tri);
        }
        return true;
    }

    int cpyint(const char *&p)
    {
        int cpy;
        memwriter = (char *)&cpy;
        memcpy(memwriter, p, 4);
        p += 4;
        return cpy;
    }

    float cpyfloat(const char *&p)
    {
        float cpy;
        memwriter = (char *)&cpy;
        memcpy(memwriter, p, 4);
        p += 4;
        return cpy;
    }
};

#endif