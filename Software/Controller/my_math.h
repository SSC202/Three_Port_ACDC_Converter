/**
 * @file    数学计算和通用计算相关
 * @note    本文件包含数学常量定义,通用计算的相关定义和基础数学运算的相关定义。
 *          1. 通用计算相关定义
 *              接口函数:    get_max()       三值取最大值函数
 *                          get_min()       三值取最小值函数
 *                          get_middle()    三值取中间值函数
 *                          normalize()     机械角度转换为电角度
 *                          fast_sin()      快速正弦函数
 *                          fast_cos()      快速余弦函数
 *                          fast_sqrt()     快速开方函数
 *          2. PID 计算(增量式PID)
 *              接口类型: PID_t  PID结构体句柄,使用时在主函数内定义全局结构体
 *              接口函数:   PID_init()      PID结构体初始化
 *                          PID_Calc()     单次PID计算
 *          3. IIR 滤波器
 *              接口类型: LPF_t  低通滤波器结构体句柄,使用时在主函数内定义全局结构体
 *              接口函数:   LPF_Init()      低通滤波器结构体初始化
 *                          LPF_Calc()     单次低通滤波器计算
 *          4. QPR 控制器
 *              接口类型: QPR_t  QPR结构体句柄，使用时在主函数内定义全局结构体
 *              接口函数    QPR_Init()      QPR控制器初始化
 *                          QPR_Calc()      单次QPR计算
 *
 */
#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "math.h"
#include "stm32h7xx.h"

#define M_PI         3.141592653589793f // PI
#define M_TABLE_SIZE 1024

/**
 * PID 相关定义
 */
typedef struct {
    float KP;        // PID参数P
    float KI;        // PID参数I
    float KD;        // PID参数D
    float fdb;       // PID反馈值
    float ref;       // PID目标值
    float cur_error; // 当前误差
    float error[2];  // 前两次误差
    float output;    // 输出值
    float outputMax; // 最大输出值的绝对值
    float outputMin; // 最小输出值的绝对值用于防抖
} PID_t;

/**
 * IIR 滤波器相关定义
 */
typedef struct
{
    float tsample;     // 采样时间
    float wc;          // 截止频率
    float alpha;       // 滤波器系数
    float input;       // 当前输入值
    float output_last; // 前次输出值
    float output;      // 当前输出值
} LPF_t;

/**
 * SOGI 相关定义
 */
typedef struct {
    // 初始化参数
    float tsample; // 采样时间
    float w0;      // 中心频率

    // 传递函数系数
    float d0;
    float d1;
    float d2;
    float a1;
    float a2;
    float b0_alpha;
    float b0_beta;
    float b1_beta;

    // 临时变量
    float u[3];
    float y_alpha[3];
    float y_beta[3];

    // 输入输出端口
    float input;        // 输入值
    float output_alpha; // alpha 输出值
    float output_beta;  // beta 输出值
} SOGI_t;

/**
 * QPR 控制器相关定义
 */
typedef struct {
    // 初始化参数
    float wc;      // 谐振峰带宽
    float wr;      // 谐振频率
    float tsample; // 采样时间
    float kp;      // 直流增益
    float kr;      // 谐振增益

    // 传递函数系数
    float d0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    // 临时变量
    float u[3];
    float y[3];

    // 输入输出端口
    float ref;    // 指令值
    float fdb;    // 反馈值
    float output; // 输出值

} QPR_t;

float normalize(int pole_pairs, float mechine_angle, float offset);
float get_middle(float a, float b, float c);
float get_max(float a, float b, float c);
float get_min(float a, float b, float c);

void PID_init(PID_t *pid, float kp, float ki, float kd, float outputMax, float outputMin);
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample);

void LPF_Init(LPF_t *lpf, float f_c, float t_sample);
void LPF_Calc(LPF_t *lpf);

void SOGI_Init(SOGI_t *sogi, float w0, float t_sample);
void SOGI_Calc(SOGI_t *sogi);

void QPR_Init(QPR_t *qpr, float wc, float wr, float t_sample, float kp, float kr);
void QPR_Calc(QPR_t *qpr, uint8_t enable);

float fast_sin(float x);
float fast_cos(float x);
float fast_sqrt(float x);

#endif