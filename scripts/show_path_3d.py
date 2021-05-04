#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from stl import mesh

# 使用figure对象
fig = plt.figure()
# 创建3D轴对象
axes = mplot3d.Axes3D(fig)

# 读取stl文件
m1 = mesh.Mesh.from_file(os.path.join(os.getcwd(),'test.stl'))

axes.plot_surface(m1.x, m1.y, m1.z)
axes.plot_wireframe(m1.x, m1.y, m1.z, linewidth=0.25, color='black')

scale = m1.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

plt.show()

