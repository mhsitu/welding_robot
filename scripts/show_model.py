#!/usr/bin/env python
# -*- coding: utf-8 -*-

import plotly.graph_objects as go
import numpy as np
from stl import mesh
import os

# 读取stl文件
m1 = mesh.Mesh.from_file(os.path.join(os.getcwd(), 'test.stl'))

fig = go.Figure(
    data=[go.Mesh3d(x=x, y=y, z=z, color='lightpink', opacity=0.50)])
fig.show()
