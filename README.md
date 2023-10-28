# ZJU-I_manipulator
robot 1 practice

先从github上下载本分支

UBuntu系统编译：
open the command line, type in as follows
```
cd ZJU-I_manipulator
cmake -B build
cmake --build build
cd .. && pip install ./ZJU-I_manipulator
```

Windows系统里建议使用VS进行编译（目前已测试成功）
先把所有.h文件和.cpp文件加入当前项目（记得把下图项目名字命名成handsome）
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/b42e0ac0-c2a5-4742-868d-2e9db421cc6b)
然后设置项目属性（记得改成Release模式配置）

1.设置配置类型为动态库(.dll)
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/c33416d5-03ec-468d-a21e-9d5da8c7c9a5)
2.更改高级中的目标文件扩展名
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/ec2290ce-4040-4fa1-8019-03d542b727fe)
3.在包含目录中加入include和pybind11的include
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/20afb629-3f5c-4009-9654-d8a0a0bfd954)
找到本地python文件夹，一般会在C盘自己用户文件夹下AppData/Local/Programs/Python文件夹下，如果找不到可以去看自己PATH设置
包含其中的include文件夹，比如我的就像下面这样
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/155b2b87-cd55-470e-ad09-643df533017f)
4.将本地python加入库目录
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/c5839890-8873-46de-951e-f2aa2692e38f)
5.设置链接器附加依赖项
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/2836f55b-8487-4e42-a3d9-28a265cbe70d)
这里面的dll取决于刚刚库目录里的，比如我的是这样
![image](https://github.com/mostmosthandsome/ZJU-I_manipulator/assets/121703002/7deb9cee-ce9b-4d33-9137-329780e70202)

确认，然后生成即可
在Release文件夹下运行python， 
输入import handsome没有错误即可
