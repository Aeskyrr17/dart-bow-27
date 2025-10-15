//
// Created by ASUS on 2025/10/15.
//

#ifndef TASKCHASSIS_HPP
#define TASKCHASSIS_HPP

#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "slope.hpp"
#include "filter.hpp"
#include "bsp_can.hpp"
#include "magicmsgs.hpp"
#include "kalman_filter.h"

/*
 *@brief: 匀加速模型卡尔曼滤波器
*/

#define WHEEL_RADIUS 0.077f
#define MWHEEL 1.136f //两个轮子
#define GACCEL 9.78f

#define M_BODY_HALF 6.25f

#endif //TASKCHASSIS_HPP