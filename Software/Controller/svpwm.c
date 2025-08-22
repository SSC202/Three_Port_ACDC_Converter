#include "svpwm.h"

/**
 * @brief       SVPWM 占空比生成函数
 * @attention   使用三次谐波注入法
 * @param       u_ref      参考电压
 * @param       u_dc       母线电压
 * @param       duty_abc   输出占空比
 */
void e_svpwm(abc_t *u_ref, float u_dc, duty_abc_t *duty_abc)
{
    // float u_max = get_max(u_ref->a, u_ref->b, u_ref->c);
    // float u_min = get_min(u_ref->a, u_ref->b, u_ref->c);

    // float u_offset = -0.5f * (u_max + u_min);
    float u_offset = 0;

    float ua_ref_eq = u_ref->a + u_offset;
    duty_abc->dutya = 0.5f + ua_ref_eq / u_dc;
    float ub_ref_eq = u_ref->b + u_offset;
    duty_abc->dutyb = 0.5f + ub_ref_eq / u_dc;
    float uc_ref_eq = u_ref->c + u_offset;
    duty_abc->dutyc = 0.5f + uc_ref_eq / u_dc;
}