# RM26_H7
PnX26赛季电控H7开发板通用仓库，只需要做小幅度修改，即可改为F4仓库，根据需要会再开一个F4专用仓库。
## ToolChain
使用的工具链为CubeMX+CLion+Ozone，可以实现全平台开发、调试，在合适的系统支持下可以实现快速编译。
### CubeMX
用于配置各个外设，生成初始化代码，勾选`STM32CubeIDE`选项，生成的代码可以直接导入CLion。
### CLion
用于代码编写，代码管理，代码编译，只需配置`arm-none-gebi`编译。
### Ozone
用于代码调试，烧录。
## Usage
### ThreadX
### Onemessage
我们使用青岛大学开源的Onemessage进行各个模块与线程间的通信。简单使用方法如下：
创建与发布
```c++
om_topic_t *your_topic = om_config_topic(nullptr, "ca", "name", sizeof(msg_struct_t));
msg_struct_t msg;
...
for(;;){
    om_publish(your_topic, &msg, sizeof(msg), true, false);
}
```
订阅
```c++
om_suber_t *your_suber = om_subscribe(om_find_topic("name", UINT32_MAX));
msg_struct_t msg;
...
for(;;){
    om_suber_export(your_suber, &msg, false);
}
...
```
> 注意：如果订阅不存在话题，可能导致线程卡死。
### Service
### Task
## Workflow
### Doxygen
### Cherrypick
