/********************************************************************
 * @brief   2025年电赛 三相逆变器+三相整流器 能量回馈装置
 * @brief
 */
#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**********************************************
 * @brief   用户自定义变量
 */
uint8_t system_sample_flag = 0;                                                // 采样标志
float system_sample_time   = 0.00005f;                                         // 采样时间
uint8_t system_print       = 0;                                                // 用户打印标志
uint8_t system_flag        = 0;                                                // 系统状态机标志
uint8_t system_run_flag    = 0;                                                // 系统标志运行标志:0:无运行;1:逆变器运行;2:整流器运行
uint16_t system_adc_value[6] __attribute__((section(".ARM.__at_0x24000000"))); // ADC 采样值

/************************************* 按键标志 ************************************* */
uint8_t system_key1_flag;  // SWITCH_1 按键标志
uint8_t system_key2_flag;  // SWITCH_2 按键标志
uint8_t system_key5_flag;  // SWITCH_5 按键标志
uint8_t system_key6_flag;  // SWITCH_6 按键标志
uint8_t system_key7_flag;  // SWITCH_7 按键标志
uint8_t system_key8_flag;  // SWITCH_8 按键标志
uint8_t system_key16_flag; // SWITCH_16 按键标志

static void switch_clear(void)
{
    system_key1_flag  = 0;
    system_key2_flag  = 0;
    system_key5_flag  = 0;
    system_key6_flag  = 0;
    system_key7_flag  = 0;
    system_key8_flag  = 0;
    system_key16_flag = 0;
}

/************************************ 三相逆变器 ************************************ */
float u_dc = 56.0f; // 母线电压

float freq = 50.0f; // 输出频率(Hz)

float ud_ref = 21.9f; // 输出电压(可调)

float uo_theta; // 输出电压电角度

float uo_ab;  // 输出 AB 线电压
float uo_bc;  // 输出 BC 线电压
float uo_ca;  // 输出 CA 线电压
abc_t uo_abc; // 输出 ABC 相电压
dq_t uo_dq;   // 输出 dq 轴电压

PID_t uod_controller; // 输出 d 轴 PI 控制器
PID_t uoq_controller; // 输出 q 轴 PI 控制器

dq_t udq_inv;            // 逆变器 dq 指令输出电压
abc_t uabc_inv;          // 逆变器 ABC 指令输出电压
duty_abc_t duty_abc_inv; // 逆变器输出占空比

/************************************ 三相整流器 ************************************ */
float us_theta; // 整流器输入 A 相相位

SOGI_t us_sogi; // 整流器输入 A 相 SOGI
PID_t us_pll;   // 整流器输入锁相环

abc_t is_abc;     // 输入 ABC 相电流
dq_t is_dq;       // 输入 dq 轴电流
LPF_t isd_filter; // 输入 d 轴电流滤波器
LPF_t isq_filter; // 输入 q 轴电流滤波器

float is_0; // 输入零序电流

PID_t isd_controller; // 输入 d 轴 PI 控制器
PID_t isq_controller; // 输入 q 轴 PI 控制器

dq_t udq_rec;            // 整流器 dq 指令输出电压
abc_t uabc_rec;          // 整流器 ABC 指令输出电压
duty_abc_t duty_abc_rec; // 整流器输出占空比

float id_ref = 2.39f; // 输入电流(可调)

/**********************************************
 * @brief   用户自定义临时变量
 */
float duty;
/**
 * @brief System init
 */
static void init()
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    // TIM 开启
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    TIM8->CCR1  = 0;
    TIM8->CCR2  = 0;
    TIM8->CCR3  = 0;
    TIM1->CCR1  = 0;
    TIM1->CCR2  = 0;
    TIM1->CCR3  = 0;
    system_flag = 0;
    switch_clear();
    // 逆变器相关参数初始化
    PID_init(&uod_controller, 0.01, 5, 0, 28, -28);
    PID_init(&uoq_controller, 0.01, 5, 0, 28, -28);
    // 整流器相关参数初始化
    // PID_init(&us_pll, 5000, 20000, 0, INFINITY, -INFINITY);
    // SOGI_Init(&us_sogi, 2 * M_PI * 50.0f, system_sample_time);
    LPF_Init(&isd_filter, 80, system_sample_time);
    LPF_Init(&isq_filter, 80, system_sample_time);
    PID_init(&isd_controller, 15, 1500, 0, 8, -8);
    PID_init(&isq_controller, 15, 1500, 0, 8, -8);
    // 等待选题
    while (1) {
        if (system_key1_flag == 1) {
            system_flag = 1;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
            switch_clear();
            break;
        } else if (system_key2_flag == 1) {
            system_flag = 2;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
            switch_clear();
            break;
        }
    }
    HAL_Delay(8000);
    // ADC init
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)system_adc_value, 6);
    while (system_sample_flag == 0) {
        ;
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    system_run_flag = 0;
}

void usermain()
{
    init();
    while (1) {
        // // 调试打印输出(注意:正式使用时请注释)
        // if (system_print == 0) {
        //     printf("u:%f,%f,%f\r\n", is_dq.d, is_dq.q, is_0);
        // } else if (system_print == 1) {
        //     printf("u:%f,%f,%f\r\n", is_abc.a, is_abc.b, is_abc.c);
        // } else if (system_print == 2) {
        //     printf("u:%f,%f,%f\r\n", is_abc.a, is_abc.b, is_abc.c);
        // } else if (system_print == 3) {
        //     printf("u:%f,%f\r\n", uo_ab, uo_bc);
        // } else if (system_print == 4) {
        //     printf("u:%f,%f\r\n", uo_dq.d, uo_dq.q);
        // }
        // printf("u:%f,%f\r\n", uo_ab, uo_bc);
        // printf("u:%f,%f,%f\r\n", duty_abc_inv.dutya, duty_abc_inv.dutyb, duty_abc_inv.dutyc);
        // printf("u:%f,%f,%f\r\n", uo_abc.a, uo_abc.b, uo_abc.c);
        // printf("u:%f,%f\r\n", uo_dq.d, uo_dq.q);
        // printf("u:%f,%f\r\n", uo_abc.a, 4 * cosf(uo_theta));
        // printf("u:%f,%f,%f\r\n", is_abc.a, is_abc.b, is_abc.c);
        // printf("u:%f,%f\r\n", is_dq.d, is_dq.q);
        // printf("u:%f,%f,%f\r\n", duty_abc_rec.dutya, duty_abc_rec.dutyb, duty_abc_rec.dutyc);
        // OLED 显示程序
    }
}

/*************************************** 中断主程序 ******************************************* */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    static uint32_t adc_cnt = 0;

    static uint32_t adc_uab_offset_sum = 0;
    static uint32_t adc_uab            = 0;
    static float adc_uab_offset        = 0;

    static uint32_t adc_ubc_offset_sum = 0;
    static uint32_t adc_ubc            = 0;
    static float adc_ubc_offset        = 0;

    static uint32_t adc_isa_offset_sum = 0;
    static uint32_t adc_isa            = 0;
    static float adc_isa_offset        = 0;

    static uint32_t adc_isb_offset_sum = 0;
    static uint32_t adc_isb            = 0;
    static float adc_isb_offset        = 0;

    static uint32_t adc_isc_offset_sum = 0;
    static uint32_t adc_isc            = 0;
    static float adc_isc_offset        = 0;

    /**********************************
     * @brief   电压/电流采样
     */
    if (hadc == &hadc1) {

        if (system_sample_flag == 0) {
            adc_cnt++;
            if (adc_cnt >= 20001) {
                adc_uab_offset_sum += system_adc_value[0];
                adc_ubc_offset_sum += system_adc_value[1];
                adc_isa_offset_sum += system_adc_value[2];
                adc_isb_offset_sum += system_adc_value[3];
                adc_isc_offset_sum += system_adc_value[4];
            }
            if (adc_cnt == 21000) {
                adc_uab_offset     = adc_uab_offset_sum / 1000.0f;
                adc_ubc_offset     = adc_ubc_offset_sum / 1000.0f;
                adc_isa_offset     = adc_isa_offset_sum / 1000.0f;
                adc_isb_offset     = adc_isb_offset_sum / 1000.0f;
                adc_isc_offset     = adc_isc_offset_sum / 1000.0f;
                system_sample_flag = 1;
                adc_uab_offset_sum = 0;
                adc_ubc_offset_sum = 0;
                adc_isa_offset_sum = 0;
                adc_isb_offset_sum = 0;
                adc_isc_offset_sum = 0;
                adc_cnt            = 0;
            }
        } else {
            adc_uab = system_adc_value[0];
            uo_ab   = ((((float)adc_uab - adc_uab_offset) / 65535.f) * 3.3f) * 66.0999f;

            adc_ubc = system_adc_value[1];
            uo_bc   = ((((float)adc_ubc - adc_ubc_offset) / 65535.f) * 3.3f) * 66.0999f;

            adc_isa  = system_adc_value[2];
            is_abc.a = ((((float)adc_isa - adc_isa_offset) / 65535.f) * 3.3f) * 5.0f;

            adc_isb  = system_adc_value[3];
            is_abc.b = ((((float)adc_isb - adc_isb_offset) / 65535.f) * 3.3f) * 5.0f;

            adc_isc  = system_adc_value[4];
            is_abc.c = ((((float)adc_isc - adc_isc_offset) / 65535.f) * 3.3f) * 5.0f;
            // 过流保护
            if ((is_abc.a * is_abc.a > 25) || (is_abc.b * is_abc.b > 25) || (is_abc.c * is_abc.c > 25)) {
                system_run_flag = 0;
            }
        }
    }
    // ADC 校准完毕后，可运行采样程序，控制程序受状态机控制
    if (system_sample_flag == 1) {
        /*********************************
         * @brief   逆变器控制
         */

        if (system_flag == 1) {
            if (system_key7_flag == 1) {
                freq = freq + 1;
                switch_clear();
            } else if (system_key8_flag == 1) {
                freq = freq - 1;
                switch_clear();
            }
            if (freq > 100.f) {
                freq = 100.f;
            } else if (freq < 20.f) {
                freq = 20.f;
            }
        } else if (system_flag == 2) {
            freq = 50.f;
        }
        if (system_flag == 1) {
            if (system_key1_flag == 1) {
                ud_ref = ud_ref + 0.01f;
                switch_clear();
            } else if (system_key2_flag == 1) {
                ud_ref = ud_ref - 0.01f;
                switch_clear();
            }
            if (ud_ref < 4.f) {
                ud_ref = 4.f;
            } else if (ud_ref > 28.f) {
                ud_ref = 28.f;
            }
        } else if (system_flag == 2) {
            if (system_key1_flag == 1) {
                ud_ref = ud_ref + 0.01f;
                switch_clear();
            } else if (system_key2_flag == 1) {
                ud_ref = ud_ref - 0.01f;
                switch_clear();
            }
            if (ud_ref < 15.f) {
                ud_ref = 15.f;
            } else if (ud_ref > 25.f) {
                ud_ref = 28.f;
            }
        }

        // 相位生成
        uo_theta = uo_theta + (2 * M_PI * freq) * system_sample_time;
        uo_theta = normalize(1, uo_theta, 0);

        // 逆变器电压采样/电压环控制
        // 注意：采集AB相/BC相电压
        uo_abc.a = (2 * uo_ab + uo_bc) / 3.0f;
        uo_abc.b = (uo_bc - uo_ab) / 3.0f;
        uo_abc.c = (-2 * uo_bc - uo_ab) / 3.0f;

        // Clark-Park
        abc_2_dq(&uo_abc, &uo_dq, uo_theta);

        // PID 计算
        uod_controller.ref = ud_ref;
        uod_controller.fdb = uo_dq.d;
        if (system_run_flag == 1) {
            PID_Calc(&uod_controller, 1, system_sample_time);
        } else {
            PID_Calc(&uod_controller, 0, system_sample_time);
        }
        udq_inv.d = uod_controller.output;

        uoq_controller.ref = 0;
        uoq_controller.fdb = uo_dq.q;
        if (system_run_flag == 1) {
            PID_Calc(&uoq_controller, 1, system_sample_time);
        } else {
            PID_Calc(&uoq_controller, 0, system_sample_time);
        }
        udq_inv.q = uoq_controller.output;

        // 输出
        dq_2_abc(&udq_inv, &uabc_inv, uo_theta);
        e_svpwm(&uabc_inv, u_dc, &duty_abc_inv);

        /********************************
         * @brief   整流器控制
         */
        if (system_flag == 2) {
            if (system_key7_flag == 1) {
                id_ref = id_ref + 0.01f;
                switch_clear();
            } else if (system_key8_flag == 1) {
                id_ref = id_ref - 0.01f;
                switch_clear();
            }
            if (id_ref < 1.5f) {
                id_ref = 1.5f;
            } else if (id_ref > 2.6f) {
                id_ref = 2.6f;
            }
        }

        // 电流采样/电流环控制
        // Clark-Park
        // dq_t is_dq_ori;
        abc_2_dq(&is_abc, &is_dq, uo_theta);

        // isd_filter.input = is_dq_ori.d;
        // LPF_Calc(&isd_filter);
        // is_dq.d = isd_filter.output;

        // isq_filter.input = is_dq_ori.q;
        // LPF_Calc(&isq_filter);
        // is_dq.q = isq_filter.output;

        // PID
        isd_controller.ref = is_dq.d;
        isd_controller.fdb = id_ref;
        if (system_run_flag == 1) {
            PID_Calc(&isd_controller, 1, system_sample_time);
        } else {
            PID_Calc(&isd_controller, 0, system_sample_time);
        }
        udq_rec.d = udq_inv.d + isd_controller.output;

        isq_controller.ref = is_dq.q;
        isq_controller.fdb = 0;
        if (system_run_flag == 1) {
            PID_Calc(&isq_controller, 1, system_sample_time);
        } else {
            PID_Calc(&isq_controller, 0, system_sample_time);
        }
        udq_rec.q = udq_inv.q + isq_controller.output;

        // 零序抑制器
        is_0 = (is_abc.a + is_abc.b + is_abc.c) / 3;
        float u_offset;
        u_offset = is_0 * 10;

        // 输出
        dq_2_abc(&udq_rec, &uabc_rec, uo_theta);
        uabc_rec.a = uabc_rec.a + u_offset;
        uabc_rec.b = uabc_rec.b + u_offset;
        uabc_rec.c = uabc_rec.c + u_offset;
        e_svpwm(&uabc_rec, u_dc, &duty_abc_rec);
    }
    /**********************************
     * @brief   状态机转换
     */
    switch (system_run_flag) {
        case 0:
            if (system_key5_flag == 1) {
                system_run_flag = 1;
                switch_clear();
            }
            break;
        case 1:
            if (system_key6_flag == 1) {
                system_run_flag = 0;
                switch_clear();
            }
            break;
        default:
            break;
    }

    /**********************************
     * @brief   输出控制
     */
    if (system_flag == 1) {
        if (system_run_flag == 1) {
            TIM8->CCR1 = TIM8->ARR * duty_abc_inv.dutya;
            TIM8->CCR2 = TIM8->ARR * duty_abc_inv.dutyb;
            TIM8->CCR3 = TIM8->ARR * duty_abc_inv.dutyc;
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
        } else {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
        }
    } else if (system_flag == 2) {
        if (system_run_flag == 0) {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
        } else if (system_run_flag == 1) {
            TIM1->CCR1 = TIM1->ARR * duty_abc_rec.dutya;
            TIM1->CCR2 = TIM1->ARR * duty_abc_rec.dutyb;
            TIM1->CCR3 = TIM1->ARR * duty_abc_rec.dutyc;
            TIM8->CCR1 = TIM8->ARR * duty_abc_inv.dutya;
            TIM8->CCR2 = TIM8->ARR * duty_abc_inv.dutyb;
            TIM8->CCR3 = TIM8->ARR * duty_abc_inv.dutyc;
        }
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2) {
        if (system_key1_flag == 0) {
            system_key1_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_13) {
        if (system_key2_flag == 0) {
            system_key2_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_7) {
        if (system_key5_flag == 0) {
            system_key5_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_8) {
        if (system_key6_flag == 0) {
            system_key6_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_9) {
        if (system_key7_flag == 0) {
            system_key7_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_10) {
        if (system_key8_flag == 0) {
            system_key8_flag = 1;
        }
    }
    if (GPIO_Pin == GPIO_PIN_4) {
        if (system_key16_flag == 0) {
            system_key16_flag = 1;
        }
    }
}