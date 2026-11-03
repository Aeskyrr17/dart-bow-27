# 飞镖应用

## 迁移进度:

26.11.3
- 完成并接入了最小手动控制的上弦,舵机,两侧步进电机的张力手动调整.
- trigger扳机舵机和zdt_stepper做了单独封装，暂时没有和其他电机统一接口
- 配置集中在app/dart/config.hpp中
- 目前对dm4310p的电机多圈处理直接放在了motor.cpp中

- 线程初始化及线程入口: app/dart/app.cpp