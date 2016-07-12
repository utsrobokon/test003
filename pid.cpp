/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pid.cpp
 * Author: 広明
 * 
 * Created on 2016/07/09, 16:08
 */

#include "pid.h"


pid::pid() {
}

pid::pid(const pid& orig) {
}

pid::~pid() {
}

float pid::getTurnVal(long sensor_val, long target_val)
{
    float p, i, d;
    
    diff[0] = diff[1];
    diff[1] = sensor_val - target_val;  //偏差を取得
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1] - diff[0]) / DELTA_T;
    
    return math_limit(p + i + d, LEFT_LIMIT, RIGHT_LIMIT);
}

float pid::math_limit(float pid, float left_limit, float right_limit)
{
    if(pid < left_limit)
    {
        return left_limit;
    }
    else if(pid > right_limit)
    {
        return right_limit;
    }
    
    return right_limit;
}
