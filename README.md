# 用于关节电机的功率控制
## 项目结构
Cmake Driver均为cubmx自动生成，Core中的User里面的can驱动为核心文件
## 文件应用
项目采用Cmake.txt进行编译，使用请根据版本进行CmakePresets.json的配置，然后根据Cmake-tools进行生成，
本项目采用中断裸机框架，采用低性能单核芯片，具有实用性
## 项目复刻
文件根目录下有三张照片，表明了原理图和驱动原理，默认状态仅仅只有PWM驱动，需要开启请去can_drv.c取消中断注释
