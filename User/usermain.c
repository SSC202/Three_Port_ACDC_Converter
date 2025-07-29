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
uint16_t system_adc_value[6] __attribute__((section(".ARM.__at_0x24000000"))); // ADC 采样值

/************************************* 按键标志 ************************************* */
uint8_t system_key1_flag; // SWITCH_1 按键标志
/************************************ 单相整流器 ************************************ */
float u_s;          // 电网电压
float us_theta = 0; // 电网电压相位

SOGI_t us_sogi; // 电网电压积分器
PID_t us_pll;   // 电网电压锁相环

float u_dc; // 母线电压

/************************************ 三相逆变器 ************************************ */
float uo_theta; // 输出电压电角度

abc_t uo_abc; // 逆变器输出电压 ABC 分量
abc_t io_abc; // 逆变器输出电流 ABC 分量

dq_t uo_dq; // 逆变器输出电压 dq 分量
dq_t io_dq; // 逆变器输出电流 dq 分量

dq_t u_dq;           // dq 电压指令值
abc_t u_abc;         // ABC 电压指令值
duty_abc_t duty_abc; // ABC 占空比指令值

PID_t uod_controller; // 逆变器输出 d 轴电压控制器
PID_t iod_controller; // 逆变器输出 d 轴电流控制器
PID_t uoq_controller; // 逆变器输出 q 轴电压控制器
PID_t ioq_controller; // 逆变器输出 q 轴电流控制器

/**********************************************
 * @brief   用户自定义临时变量
 */

/**
 * @brief System init
 */
static void init()
{
    system_flag      = 0;
    system_key1_flag = 0;
    // 整流器相关参数初始化
    PID_init(&us_pll, 5000, 20000, 0, INFINITY, -INFINITY);
    SOGI_Init(&us_sogi, 2 * M_PI * 50.0f, system_sample_time);
    // 逆变器相关参数初始化
    PID_init(&uod_controller, 0.1f, 0.01f, 0.001f, 5, -5);
    PID_init(&iod_controller, 0.1f, 0.01f, 0.001f, 27.71, -27.71);
    PID_init(&uoq_controller, 0.1f, 0.01f, 0.001f, 5, -5);
    PID_init(&ioq_controller, 0.1f, 0.01f, 0.001f, 27.71, -27.71);
    // ADC init
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)system_adc_value, 6);
    HAL_TIM_Base_Start(&htim1);
    while (system_sample_flag == 0) {
        ;
    }
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

void usermain()
{
    init();
    while (1) {
        // 状态机控制程序
        switch (system_flag) {
            case 0: // 0: 初始化状态，此时提示可以打开电网电闸
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
                // 跳转条件：打开电闸后按键
                if (system_key1_flag == 1) {
                    system_key1_flag = 0;
                    system_flag      = 1;
                }
                break;
            case 1: // 1: 软启动状态，此时提示软启动中，检测母线电压，如果母线电压达到48V以上可以开启整流器
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
                // 跳转条件：母线电压在48V以上且按键在此后按下
                if (u_dc > 48) {
                    if (system_key1_flag == 1) {
                        system_key1_flag = 0;
                        system_flag      = 2;
                    }
                }
                break;
            case 2: // 2: 整流器运行状态，此时显示母线电压，在按键按下后可打开逆变器
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
                if (system_key1_flag == 1) {
                    system_key1_flag = 0;
                    system_flag      = 3;
                }
                break;
            case 3: // 3: 逆变器运行状态，在停止按键按下后可关闭逆变器
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
                if (system_key1_flag == 1) {
                    system_key1_flag = 0;
                    system_flag      = 4;
                }
                break;
            default:
                break;
        }

        // 调试打印输出
        // printf("u:%f,%f,%f\r\n", duty_abc.dutya, duty_abc.dutyb, duty_abc.dutyc);

        // OLED 显示程序
    }
}

/*************************************** 中断主程序 ******************************************* */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    static uint32_t adc_cnt = 0;

    static uint32_t adc_us_offset_sum = 0;
    static uint32_t adc_us            = 0;
    static float adc_us_offset        = 0;

    static uint32_t adc_udc_offset_sum = 0;
    static uint32_t adc_udc            = 0;
    static float adc_udc_offset        = 0;

    static uint32_t adc_uoa_offset_sum = 0;
    static uint32_t adc_uoa            = 0;
    static float adc_uoa_offset        = 0;

    static uint32_t adc_uob_offset_sum = 0;
    static uint32_t adc_uob            = 0;
    static float adc_uob_offset        = 0;

    static uint32_t adc_ioa_offset_sum = 0;
    static uint32_t adc_ioa            = 0;
    static float adc_ioa_offset        = 0;

    static uint32_t adc_iob_offset_sum = 0;
    static uint32_t adc_iob            = 0;
    static float adc_iob_offset        = 0;

    /**********************************
     * @brief   电压/电流采样
     */
    if (hadc == &hadc1) {

        if (system_sample_flag == 0) {
            adc_cnt++;
            if (adc_cnt >= 1) {
                adc_us_offset_sum += system_adc_value[0];
                adc_udc_offset_sum += system_adc_value[1];
                adc_uoa_offset_sum += system_adc_value[2];
                adc_uob_offset_sum += system_adc_value[3];
                adc_ioa_offset_sum += system_adc_value[4];
                adc_iob_offset_sum += system_adc_value[5];
            }
            if (adc_cnt == 100) {
                adc_us_offset      = adc_us_offset_sum / 100.0f;
                adc_udc_offset     = adc_udc_offset_sum / 100.0f;
                adc_uoa_offset     = adc_uoa_offset_sum / 100.0f;
                adc_uob_offset     = adc_uob_offset_sum / 100.0f;
                adc_ioa_offset     = adc_ioa_offset_sum / 100.0f;
                adc_iob_offset     = adc_iob_offset_sum / 100.0f;
                system_sample_flag = 1;
                adc_us_offset_sum  = 0;
                adc_udc_offset_sum = 0;
                adc_uoa_offset_sum = 0;
                adc_uob_offset_sum = 0;
                adc_ioa_offset_sum = 0;
                adc_iob_offset_sum = 0;
                adc_cnt            = 0;
            }
        } else {
            adc_us   = system_adc_value[0];
            u_s      = ((((float)adc_us - adc_us_offset) / 65535.f) * 3.3f) * 67.7492f;
            adc_udc  = system_adc_value[1];
            u_dc     = ((((float)adc_udc - adc_udc_offset) / 65535.f) * 3.3f) * 67.7492f;
            adc_uoa  = system_adc_value[2];
            uo_abc.a = ((((float)adc_uoa - adc_uoa_offset) / 65535.f) * 3.3f) * 67.7492f;
            adc_uob  = system_adc_value[3];
            uo_abc.b = ((((float)adc_uob - adc_uob_offset) / 65535.f) * 3.3f) * 67.7492f;
            uo_abc.c = -uo_abc.a - uo_abc.b;
            adc_ioa  = system_adc_value[4];
            io_abc.a = ((((float)adc_ioa - adc_ioa_offset) / 65535.f) * 3.3f) * 5.0f;
            adc_iob  = system_adc_value[5];
            io_abc.b = ((((float)adc_iob - adc_iob_offset) / 65535.f) * 3.3f) * 5.0f;
            io_abc.c = -io_abc.a - io_abc.b;
        }
    }
    // ADC 校准完毕后，可运行采样程序，控制程序受状态机控制
    if (system_sample_flag == 1) {
        /**********************************
         * @brief   整流器控制
         */

        // SOGI 生成正交 alpha-beta 分量
        float us_alpha, us_beta;
        us_sogi.input = u_s;
        SOGI_Calc(&us_sogi);
        us_alpha = us_sogi.output_alpha;
        us_beta  = us_sogi.output_beta;

        // PLL 锁相环
        float us_peak;
        us_peak = sqrtf(us_alpha * us_alpha + us_beta * us_beta);
        float us_alpha_pu, us_beta_pu;
        us_alpha_pu = us_alpha / us_peak;
        us_beta_pu  = us_beta / us_peak;

        us_pll.ref = -us_alpha_pu * sinf(us_theta) + us_beta_pu * cosf(us_theta);
        us_pll.fdb = 0;
        if (system_flag == 1 || system_flag == 2 || system_flag == 3 || system_flag == 4) {
            PID_Calc(&us_pll, 1, system_sample_time);
        } else {
            PID_Calc(&us_pll, 0, system_sample_time);
        }
        us_theta = us_pll.output * system_sample_time + us_theta;
        us_theta = normalize(1, us_theta, 0);

        /*********************************
         * @brief   逆变器控制
         */

        // 相位生成
        uo_theta = uo_theta + (2 * M_PI * 50) * system_sample_time;
        uo_theta = normalize(1, uo_theta, 0);

        // // Clark + Park
        // abc_2_dq(&uo_abc, &uo_dq, uo_theta);
        // abc_2_dq(&io_abc, &io_dq, uo_theta);

        // // Voltage PI controller
        // uod_controller.ref = 12;
        // uod_controller.fdb = uo_dq.d;
        // if (system_flag == 3 || system_flag == 4) {
        //     PID_Calc(&uod_controller, 1, system_sample_time);
        // } else {
        //     PID_Calc(&uod_controller, 0, system_sample_time);
        // }

        // uoq_controller.ref = 0;
        // uoq_controller.fdb = uo_dq.q;
        // if (system_flag == 3 || system_flag == 4) {
        //     PID_Calc(&uoq_controller, 1, system_sample_time);
        // } else {
        //     PID_Calc(&uoq_controller, 0, system_sample_time);
        // }

        // // Current PI controller
        // iod_controller.ref = uod_controller.output;
        // iod_controller.fdb = io_dq.d;
        // if (system_flag == 3 || system_flag == 4) {
        //     PID_Calc(&iod_controller, 1, system_sample_time);
        // } else {
        //     PID_Calc(&iod_controller, 0, system_sample_time);
        // }
        // u_dq.d = iod_controller.output;

        // ioq_controller.ref = uoq_controller.output;
        // ioq_controller.fdb = io_dq.q;
        // if (system_flag == 3 || system_flag == 4) {
        //     PID_Calc(&ioq_controller, 1, system_sample_time);
        // } else {
        //     PID_Calc(&ioq_controller, 0, system_sample_time);
        // }
        // u_dq.q = ioq_controller.output;

        // AntiPark + AntiClark
        dq_2_abc(&u_dq, &u_abc, uo_theta);

        // SVPWM
        e_svpwm(&u_abc, u_dc, &duty_abc);

        /**********************************
         * @brief   输出控制
         */
        if (system_flag == 3 || system_flag == 4) {
            TIM8->CCR1 = (uint32_t)(duty_abc.dutya * (float)TIM8->ARR);
            TIM8->CCR2 = (uint32_t)(duty_abc.dutyb * (float)TIM8->ARR);
            TIM8->CCR3 = (uint32_t)(duty_abc.dutyc * (float)TIM8->ARR);
        } else {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
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
        if (system_key1_flag == 0)
            system_key1_flag = 1;
    }
}