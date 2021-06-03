/**
  ******************************************************************************
  * @file    read_STL.hpp
  * @brief   Class to read meshes from STL file.
  * @author  Stuart(South China University of Technology)
  *          1356046979@qq.com
  * @date    23/04/2021
  * @version 1.0
  ******************************************************************************
*/
#ifndef _READ_STL_HPP
#define _READ_STL_HPP
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "model_grid_map.hpp"

typedef Point3<float> Point3f;

class STLReader
{
public:
    bool readFile(std::string file_name)
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
        return triangleMesh;
    }

private:
    std::vector<Point3f> pointList;
    std::vector<Triangles<float>> triangleMesh;
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
            triangleMesh.push_back(tri);
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
            tri.nor_vec.x = cpyfloat(p);
            tri.nor_vec.y = cpyfloat(p);
            tri.nor_vec.z = cpyfloat(p);
            for (j = 0; j < 3; j++) //读取三顶点
            {
                tri.vertex[j].x = cpyfloat(p);
                tri.vertex[j].y = cpyfloat(p);
                tri.vertex[j].z = cpyfloat(p);
            }
            p += 2; //跳过尾部标志

            triangleMesh.push_back(tri);
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