# **智能焊接机器人**
## **一、简介**
- 本项目为华南理工大学2017级自动化毕业设计项目。
- 包含基于Coppeliasim搭建的仿真系统，仿真接口目前仅支持C++。
- 包含在实物系统上运行的代码。
- 本项目所有功能已进行模块化，需要使用部分功能可快速移植调用
## **二、如何运行这个程序？**

### **1. 在仿真环境中运行**

- **编译运行：**
  1. 检查文件夹路径中是否为全英文，路径包含中文会报错
  2. 打开vscode， 编译并运行`./build/bin/welding_robot.exe`

- **开始仿真：**
  1. 打开目录下的`/sim/`文件夹，运行`welding_sim.ttt`

### **2. 在实物系统上运行** 


## **三、环境依赖**

### **运行环境 :**

> 1. 极力推荐使用GCC编译器。如果你使用的是windows平台，请安装MinGW编译器。
> 2. 具体软件的安装请自行百度。

**Ubuntu 18.04 LTS / Windows 10.0+**

- CMake (version 3.12.0 +) 

- Python(version 3.0 +)

  用于数据可视化。Python的版本的位数取决于取决于编译器的编译类型。然而在Windows10平台上，推荐安装的MinGW只能编译32位程序，因此Windows10上请安装Python-32bit。

### **仿真环境 :**

- Coppeliasim 4.1.1 (推荐版本)

- Qt ([version 5.12.0 +](http://download.qt.io/archive/qt/5.12/5.12.9/), Windows上不需要安装)


## **四、CoppeliaSim Client**
> coppeliasim-client是基于官方提供的remoteAPI进行再封装的C++接口
### **使用简介**

1. API的使用

- 使用模块主要涉及三个函数：

```c++
/*
  函数作用：
  开始与模拟器的通信

  参数说明：
  - p_IP: 仿真软件的地址，通常为"127.0.0.1"
  - p_connection_port: 使用的端口号，需要和仿真主脚本启动时候填写的一致，通常可以给5000
  - commThreadCycleInMs: 每次和模拟器通信的间隔，通常设置5ms
  - sysnchronousMode: true为开启同步仿真模式
  - multiClientMode: 多客户端模式，通常为flase

  返回值：连接成功返回true，否则为false

  例：
  Start("127.0.0.1", 5000, 5, true);
*/
bool Start(const char* p_IP, int32_t p_conection_port, int commThreadCycleInMs, bool synchronousMode = false, bool multiClientMode = false);

/*
  函数作用：
  添加一个模型中存在的对象以及要对这个对象进行的操作，如读取关节速度、角度，读取传感器数据，摄像头图片，或者设置关节转动角度，速度等

  参数说明：
  - full_name: 对象在仿真软件中的名字，一定要完全匹配否则找不到对象
  - type：对象的类型，详细类型参考下面对数据结构的解释
  - operation_ls: 要进行的操作列表，可操作的数据参考下面对数据结构的解释

  返回值：如果添加成功，返回操作对象的地址，可以直接从这个地址读取反馈的数值；添加失败时返回NULL。

  例（添加名为“Joint1”的关节对象，读取当前速度和写入目标速度，读取当前角度和写入目标角度）：
  CoppeliaSim->Add_Object("Joint1", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW});
*/
_simObjectHandle_Type *Add_Object(std::string full_name, _simObj_Type type, std::initializer_list<simxInt> operation_ls);

/*
  函数作用：
  每次调用这个函数，就会自动执行使用Add_Object()函数添加的一系列读写操作，一般会循环不停地调用这个函数。

  例：
  while(1)
  {
    ComWithServer();
  }
*/
bool ComWithSevrer()
```

2. 核心的数据结构（具体查看头文件的枚举类型）

- `_simOP_Type`: 操作类型（Operation Type），可对某个对象进行不同类型的操作，包括修改或读取位置、姿态、四元数、速度、力等。
- `_simIO_Type`: 输入输出类型（Input/Output Type），有只读（Read Only），只写（Write Only）和读写（Read and Write）三种。在Coppeliasim中不支持的操作将被跳过，比如不能写入传感器的值而只能读取它。

- `_simObj_Type`:对象类型（Object Type），模块里面暂时只提供了四种对象类型，分别是关节（Joint）、视觉传感器（Vision sensor）、力传感器（Force sensor）和其他对象（Other Object），对应Coppeliasim里面的不同对象类型。

- `_simOPData_Struct`:操作数据结构体，保存有不同类型的数据，在配置好对象要进行的操作之后，就会自动从服务器端读入数据到对应的变量中或自动写入到服务器端。注意设置或读取的时候要区分“obj_Data”和“obj_Target”

### **项目演示**
演示代码位于main.cpp，下面对直线焊接进行演示：
- 直线焊接

  ![](img/straight_line.gif)

  ![](img/straight_line2.gif)

## **参考文献**
  1. X. Wang, L. Xue, Y. Yan, and X. Gu, “Welding robot collision-free path optimization,” Appl. Sci., vol. 7, no. 2, 2017, doi: 10.3390/app7020089.