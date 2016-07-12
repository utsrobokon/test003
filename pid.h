/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pid.h
 * Author: 広明
 *
 * Created on 2016/07/09, 16:08
 */

#ifndef PID_H
#define PID_H

#include "ev3api.h"

#define DELTA_T 0.004
#define KP 0.83
#define KI 0.0
#define KD 0.0
#define LEFT_LIMIT -100.0
#define RIGHT_LIMIT 100.0


class pid {
public:
    pid();
    pid(const pid& orig);
    virtual ~pid();
    float getTurnVal(long sensor_val, long target_val);
    
    
private:
    float diff[2];
    float integral;
    float math_limit(float pid, float left_limit, float right_limit);
    
};

#endif /* PID_H */

