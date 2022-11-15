// Hydraulic Control Board Rainbow
// distributed by Sungwoo Kim
//        2022/08/01
//revised by Buyoun Cho
//       2022/11/15 

#include "FastPWM.h"
#include "FlashWriter.h"
#include "INIT_HW.h"
#include "SPI_EEP_ENC.h"
#include "function_CAN.h"
#include "function_utilities.h"
#include "mbed.h"
#include "setting.h"
#include "stm32f4xx_flash.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

// DAC ///////////////////////////////////////////
AnalogOut dac_1(PA_4); // 0.0f ~ 1.0f
AnalogOut dac_2(PA_5); // 0.0f ~ 1.0f

// ADC ///////////////////////////////////////////
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

AnalogIn adc1(PC_4); // pressure_1
AnalogIn adc2(PC_5); // pressure_2
AnalogIn adc3(PC_1); // current
AnalogIn adc4(PB_1); // V_EXI
AnalogIn adc5(PA_1); // LVDT

// PWM ///////////////////////////////////////////
float PWM_duty = 0.0f;

// SPI ///////////////////////////////////////////
SPI eeprom(PB_15, PB_14, PB_13); // EEPROM //(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut eeprom_cs(PB_12);
SPI enc(PC_12, PC_11, PC_10);
DigitalOut enc_cs(PD_2);

// LED ///////////////////////////////////////////
DigitalOut LED(PA_15);

// LVDT ///////////////////////////////////////////
DigitalOut LVDT_H(PB_6);
DigitalOut LVDT_L(PB_7);

// MOTOR_ENA ///////////////////////////////////////////
DigitalOut M_ENABLE(PA_2);

// CAN ///////////////////////////////////////////
CAN can(PB_8, PB_9, 1000000);
CANMessage msg;
// void onMsgReceived()
//{
//     CAN_RX_HANDLER();
// }

// State Variables ///////////////////////////////////////////
State pos;
State vel;
State force;
State torq; // unit : N
State torq_dot;
State pres_A; // unit : bar
State pres_B;
State cur; // unit : mA
State valve_pos;
State valve_pos_raw;
State Vout;

// CAN ID  ///////////////////////////////////////////
extern int CID_RX_CMD;
extern int CID_RX_REF_POSITION;
extern int CID_RX_REF_VALVEPOS;
extern int CID_RX_REF_PWM;

extern int CID_TX_RSP;
extern int CID_TX_POS_VEL_TORQ;
extern int CID_TX_VALVE_POSITION;
extern int CID_TX_PRESSURE;
extern int CID_TX_PWMnCURRENT;
extern int CID_TX_SOMETHING;


/*******************************************************************************
 * ENUMARATED TYPES
 ******************************************************************************/

enum _REFERENCE_MODE {
    MODE_REF_NO_ACT = 0,
    MODE_REF_EXTERNAL,
    MODE_REF_UTILMODE
};

enum _CONTROL_MODE {
    //control mode
    MODE_NO_ACT = 0,             // 0
    MODE_VALVE_POSITION_CONTROL, // 1
    MODE_JOINT_CONTROL,          // 2
    MODE_VALVE_OPEN_LOOP,        // 3 

    //utility
    MODE_TORQUE_SENSOR_NULLING   = 20,                    
    MODE_FIND_HOME               = 22,                                     

    MODE_ID_VALVEPOS_VS_PWM      = 30,                      
    MODE_ID_DEADZONE_AND_CENTER  = 31,                 
    MODE_ID_FLOWRATE_VS_VALVEPOS = 32,                       

};

/*******************************************************************************
 Untility Mode Functions and Variables 
 ******************************************************************************/
float Mapping_ValvePos2PWM(float _REF_VALVE_POS);
float Mapping_FlowRate2ValvePos(float _REF_FLOWRATE);
float VALVE_POS_CONTROL(float REF_VALVE_POS);

int CNT4UtilityMode = 0;
int CNT4UtilityMode_Old = 0;
float UtilityMode_Pos = 0.0f;
float UtilityMode_PosOld = 0.0f;
float UtilityMode_PosRef = 0.0f;
float UtilityMode_PosRef_INIT = 0.0f;
float UtilityMode_Vel = 0.0f;
int UtilityMode_ID_index = 0;

enum _FINDHOME_STAGESET {
    FINDHOME_INIT = 0,
    FINDHOME_GOTOLIMIT,
    FINDHOME_ZEROPOSE
};
int FINDHOME_STAGE = FINDHOME_INIT;
bool Flag_FINDHOME_DONE = false;
int cnt_touch_end = 0;
bool Flag_PosMidFound = false;

enum _VALVEPOS_VS_PWM_ID_STAGESET {
    VALVEPOS_VS_PWM_ID_INIT = 0,
    VALVEPOS_VS_PWM_ID_MAIN,
    VALVEPOS_VS_PWM_ID_TERMINATE
};
int STAGE_VALVEPOS_VS_PWM_ID = VALVEPOS_VS_PWM_ID_INIT;

enum _VALVEDZ_ID_STAGESET {
    VALVEDZ_ID_INIT = 0,
    VALVEDZ_ID_FIND_POS_PLUS,
    VALVEDZ_ID_FIND_POS_MINUS,
    VALVEDZ_ID_MOVE2MID,
    VALVEDZ_ID_FIND_STARTPOINT,
    VALVEDZ_ID_FIND_DZBAND_LOWERBOUND,
    VALVEDZ_ID_FIND_DZBAND_UPPERBOUND,
    VALVEDZ_ID_TERMINATE
};
int STAGE_VALVEDZ_ID = VALVEDZ_ID_INIT;

enum _FLOWRATE_VS_VALVEPOS_ID_STAGESET {
    FLOWRATE_VS_VALVEPOS_ID_INIT = 0,
    FLOWRATE_VS_VALVEPOS_ID_FIND_POS_PLUS,
    FLOWRATE_VS_VALVEPOS_ID_FIND_POS_MINUS,
    FLOWRATE_VS_VALVEPOS_ID_MOVE2MID,
    FLOWRATE_VS_VALVEPOS_ID_MAIN,
    FLOWRATE_VS_VALVEPOS_ID_RETURN2MID,
    FLOWRATE_VS_VALVEPOS_ID_TERMINATE
};
int STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_INIT;

float PosCtrl4UtilFunc(float _PosRef);
void UtilFunc_TORQUE_SENSOR_NULLING(void);
void UtilFunc_FIND_HOME(void);
void UtilFunc_ID_VALVEPOS_VS_PWM(void);
void UtilFunc_ID_DEADZONE_AND_CENTER(void);
void UtilFunc_ID_FLOWRATE_VS_VALVEPOS(void);

//////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void) {
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

__HAL_RCC_PWR_CLK_ENABLE();
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 4;  // 4
RCC_OscInitStruct.PLL.PLLN = 96; // 96
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 2;
RCC_OscInitStruct.PLL.PLLR = 2;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    // Error_Handler();
}
if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    // Error_Handler();
}
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Start 

int main() {

    HAL_Init();
    SystemClock_Config();

    LED = 0;

    // SPI INIT
    eeprom_cs = 1;
    eeprom.format(8, 3);
    eeprom.frequency(5000000); // 5M
    eeprom_cs = 0;
    make_delay();

    enc_cs = 1;
    enc.format(8, 0);
    enc.frequency(5000000); // 10M
    enc_cs = 0;
    make_delay();

    // spi _ enc
    spi_enc_set_init();
    make_delay();

    //  bno rom
    spi_eeprom_write(RID_BNO, (int16_t)2); // RHP
    // spi_eeprom_write(RID_BNO, (int16_t)8); // LHP
    // spi_eeprom_write(RID_OPERATING_MODE, (int16_t)0); // MOOG TSV & Rotary Actuator
    // spi_eeprom_write(RID_OPERATING_MODE, (int16_t)1); // MOOG TSV & Linear Actuaotor
    // spi_eeprom_write(RID_OPERATING_MODE, (int16_t)4); // LIGHT's DDV & Rotary Actuator
    spi_eeprom_write(RID_OPERATING_MODE, (int16_t)5); // LIGHT's DDV & Linear Actuator
    make_delay();
    ////////

    // rom
    ROM_CALL_DATA();
    make_delay();

    // Parameter Setting according to the Valve Type
    if(Classify_ValveType()==VALVETYPE_LIGHTDDV) {
        VALVE_MAX_POS = spi_eeprom_read(RID_VALVE_MAX_POS);
        VALVE_MIN_POS = spi_eeprom_read(RID_VALVE_MIN_POS);
        VALVE_POS_NUM = spi_eeprom_read(RID_VALVE_POS_NUM);
        VALVE_ELECTRIC_CENTER = spi_eeprom_read(RID_VALVE_ELECTRIC_CENTER);
        DZ_OPENSIZE = 128.0f;
        INIT_DZ_OPENSIZE = 128.0f;
        for (int i = 0; i < NUM_VALVEPOS_VS_PWM_ID; i++) {
            if (i % 2 == 0) {
                PWMs_for_ValvePosID[i] = -((float)i * 0.5f * 0.500f);
                PWMs_for_ValvePosID_Sorted[(NUM_VALVEPOS_VS_PWM_ID-1-i)/2] = PWMs_for_ValvePosID[i];
            } else {
                PWMs_for_ValvePosID[i] = (float)(i+1) * 0.5f * 0.500f;
                PWMs_for_ValvePosID_Sorted[(NUM_VALVEPOS_VS_PWM_ID+i)/2] = PWMs_for_ValvePosID[i];
            }
        }
        for (int i = 0; i < NUM_FLOWRATE_VS_VALVEPOS_ID; i++) {
            if (i % 2 == 0) {
                ValvePosOff_for_FlowrateID[i] = -((float)i * 0.5f * 36.0f);
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID-1-i)/2] = ValvePosOff_for_FlowrateID[i];
            } else {
                ValvePosOff_for_FlowrateID[i] = (float)(i+1) * 0.5f * 36.0f;
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID+i)/2] = ValvePosOff_for_FlowrateID[i];
            }
        }
    } else if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) {
        VALVE_MAX_POS = 10000;
        VALVE_MIN_POS = -10000;
        DZ_OPENSIZE = 512.0f;
        INIT_DZ_OPENSIZE = 512.0f;
        for (int i = 0; i < NUM_FLOWRATE_VS_VALVEPOS_ID; i++) {
            if (i % 2 == 0) {
                ValvePosOff_for_FlowrateID[i] = -((float)i * 0.5f * 400.0f);
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID-1-i)/2] = ValvePosOff_for_FlowrateID[i];
            } else {
                ValvePosOff_for_FlowrateID[i] = (float)(i+1) * 0.5f * 400.0f;
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID+i)/2] = ValvePosOff_for_FlowrateID[i];
            }
        }
    }

    // ADC init
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // clock for ADC3
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // clock for ADC2
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // clock for ADC1

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock for GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock for GPIOB

    Init_ADC1();
    Init_ADC2();
    Init_ADC3();
    make_delay();

    // CAN
    can.attach(&CAN_RX_HANDLER);
    CAN_ID_INIT();
    make_delay();

    // can.reset();
    can.filter(msg.id, 0xFFFFF000, CANStandard);
    //    can.filter(0b100000000, 0b100000010, CANStandard);  //CAN ID 100~400번대
    //    통과하게

    // TMR1 init (PWM)
    Init_TMR1();
    TIM1->CR1 &= ~(TIM_CR1_UDIS);
    make_delay();

    // TMR2 init (Control)
    Init_TMR2();
    TIM2->CR1 &= ~(TIM_CR1_UDIS);
    make_delay();

    // TMR3 init (Sensors)
    Init_TMR3();
    TIM3->CR1 &= ~(TIM_CR1_UDIS);
    make_delay();

    // TIM4 init (LVDT)
    Init_TIM4();
    TIM4->CR1 &= ~(TIM_CR1_UDIS);
    make_delay();

    // Timer priority
    NVIC_SetPriority(TIM3_IRQn, 3);
    NVIC_SetPriority(TIM2_IRQn, 4);
    NVIC_SetPriority(TIM4_IRQn, 2);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 14, 0);

    // DAC init
    if (SENSING_MODE == 0) {
        dac_1 = FORCE_VREF / 3.3f;
        dac_2 = 0.0f;
    } else if (SENSING_MODE == 1) {
        dac_1 = PRES_A_VREF / 3.3f;
        dac_2 = PRES_B_VREF / 3.3f;
    }

    make_delay();


    // FET On/Off
    M_ENABLE = 1;

    /************************************
    ***     Program is operating!
    *************************************/
    while (1) {

        //        if (LED > 0) LED = 0;
        //        else LED = 1;
    }
}

// Main End
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// PWM Command Input vs. voltage output of L6205 in STM board with Moog TSV
#define LT_MAX_IDX 45
float LT_PWMCommand_Input_MoogTSV[LT_MAX_IDX] = {
    0.0f,     650.0f,   700.0f,   750.0f,   800.0f,   900.0f,   1000.0f,  1100.0f,  1200.0f,  1300.0f,
    1400.0f,  1500.0f,  1600.0f,  1700.0f,  1800.0f,  1900.0f,  2000.0f,  2100.0f,  2200.0f,  2300.0f,
    2400.0f,  2500.0f,  2700.0f,  2900.0f,  3100.0f,  3300.0f,  3500.0f,  3700.0f,  3900.0f,  4200.0f,
    4500.0f,  4800.0f,  5100.0f,  5400.0f,  5700.0f,  6000.0f,  6500.0f,  7000.0f,  7500.0f,  8000.0f,
    8500.0f,  9000.0f,  10000.0f, 11000.0f, 12000.0f}; // *100mV input
float LT_PWMVoltage_Output_MoogTSV[LT_MAX_IDX] = {
    0.0f,    70.0f,   430.0f,  600.0f,  680.0f,  775.0f,  880.0f,  975.0f,  1070.0f, 1160.0f,
    1260.0f, 1360.0f, 1460.0f, 1550.0f, 1640.0f, 1740.0f, 1840.0f, 1930.0f, 2025.0f, 2120.0f,
    2220.0f, 2310.0f, 2500.0f, 2690.0f, 2880.0f, 3070.0f, 3260.0f, 3450.0f, 3640.0f, 3920.0f,
    4210.0f, 4495.0f, 4770.0f, 5065.0f, 5350.0f, 5640.0f, 6110.0f, 6620.0f, 7110.0f, 7620.0f,
    8100.0f, 8610.0f, 9620.0f, 10600.0f, 11600.0f}; // *100mV input

float Mapping_OutputVoltage2PWMCommand_MoogTSV(float Ref_V) {
    float PWMCommand = 0.0f;
    bool sign_inversed= false;

    if(Ref_V<0.0f) {
        sign_inversed = true;
        Ref_V = -Ref_V;
    }

    if(Ref_V < LT_PWMVoltage_Output_MoogTSV[1]) { 
        PWMCommand = 0.0f;
    } else if (Ref_V > LT_PWMVoltage_Output_MoogTSV[LT_MAX_IDX-1]) {
        PWMCommand = LT_PWMCommand_Input_MoogTSV[LT_MAX_IDX-2] + (Ref_V-LT_PWMVoltage_Output_MoogTSV[LT_MAX_IDX-2])*(LT_PWMCommand_Input_MoogTSV[LT_MAX_IDX-1]-LT_PWMCommand_Input_MoogTSV[LT_MAX_IDX-2])/(LT_PWMVoltage_Output_MoogTSV[LT_MAX_IDX-1]-LT_PWMVoltage_Output_MoogTSV[LT_MAX_IDX-2]);
    } else {
        for (int idx = 1; idx < LT_MAX_IDX - 1; idx++) {
            float ini_x = LT_PWMVoltage_Output_MoogTSV[idx];
            float fin_x = LT_PWMVoltage_Output_MoogTSV[idx + 1];
            float ini_y = LT_PWMCommand_Input_MoogTSV[idx];
            float fin_y = LT_PWMCommand_Input_MoogTSV[idx + 1];
            if (Ref_V >= ini_x && Ref_V < fin_x) {
                PWMCommand = (fin_y - ini_y) / (fin_x - ini_x) * (Ref_V - ini_x) + ini_y;
                break;
            }
        }
    }

    if(sign_inversed) return -PWMCommand;
    else return PWMCommand;
}

/*******************************************************************************
                            TIMER INTERRUPT
*******************************************************************************/

//------------------------------------------------
//     TMR3 : Sensor 20kHz
//------------------------------------------------
float FREQ_TMR3 = (float)FREQ_20k;
long CNT_TMR3 = 0;
float DT_TMR3 = (float)DT_20k;
extern "C" void TIM3_IRQHandler(void) {
if (TIM3->SR & TIM_SR_UIF) {

    float PSEN1 = 0.0f;
    float PSEN2 = 0.0f;
    float CURRENT_SEN = 0.0f;

    /////////////////////////Current////////////////////////////////////////////////////////////////////////////
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    CURRENT_SEN = (float)HAL_ADC_GetValue(&hadc2);
    cur.UpdateSen(((float)CURRENT_SEN - 2047.5f) / 2047.5f * 10.0f, FREQ_TMR3, 500.0f); // unit : mA
    if(Classify_ValveType()==VALVETYPE_MOOG||Classify_ValveType()==VALVETYPE_KNR) {
        valve_pos.UpdateSen(((float)CURRENT_SEN - 2047.5f) / 2047.5f * 10000.0f, FREQ_TMR3, 500.0f); // unit : uA
    }

    /////////////////////////V_EXI////////////////////////////////////////////////////////////////////////////
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    V_EXI = (float)HAL_ADC_GetValue(&hadc2);

    /////////////////////////Encoder////////////////////////////////////////////////////////////////////////////
    //        if (CNT_TMR1 % 2) == 0) {
    ENC_UPDATE();
    //        }

    /////////////////////////Force or Pressure//////////////////////////////////////////////////////////////////
    if (SENSING_MODE == 0) { // Force sensing

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        PSEN1 = (float)HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        PSEN2 = (float)HAL_ADC_GetValue(&hadc1);

        force.UpdateSen((((float)PSEN1) - 2047.5f)/TORQUE_SENSOR_PULSE_PER_TORQUE,FREQ_TMR3, 200.0f); // unit : N //100Hz

    } else if (SENSING_MODE == 1) { // Pressure sensing

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        PSEN1 = (float)HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        PSEN2 = (float)HAL_ADC_GetValue(&hadc1);

        float pres_A_new, pres_B_new;
        pres_A_new = (((float)PSEN1) - PRES_A_NULL_pulse) / PRES_SENSOR_A_PULSE_PER_BAR; // unit : bar
        pres_B_new = (((float)PSEN2) - PRES_B_NULL_pulse) / PRES_SENSOR_B_PULSE_PER_BAR;

        pres_A.UpdateSen(pres_A_new, FREQ_TMR3, 200.0f);
        pres_B.UpdateSen(pres_B_new, FREQ_TMR3, 200.0f);

        if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Actuator
            float torq_new = ((float)PISTON_AREA_A * pres_A.sen - (float)PISTON_AREA_B * pres_B.sen) * 0.0001f;  // mm^3*bar >> Nm
            torq.UpdateSen(torq_new, FREQ_TMR3, 200.0f); // unit : Nm   //1000Hz
        } else if (Classify_ActuatorType() == ACTUATORTYPE_LIN) {     // Linear Actuator
            float force_new = ((float)PISTON_AREA_A * pres_A.sen - (float)PISTON_AREA_B * pres_B.sen) * 0.1f; // mm^2*bar >> N
            force.UpdateSen(force_new, FREQ_TMR3, 200.0f); // unit : N  //1000Hz
        }
    }
    CNT_TMR3++;
}
TIM3->SR = 0x0; // reset the status register
}

//------------------------------------------------
//     TMR4 : LVDT 500Hz
//------------------------------------------------
float LVDT_new = 0.0f;
float LVDT_old = 0.0f;
float LVDT_f_cut = 500.0f;
float LVDT_LPF = 0.0f;
float LVDT_sum = 0.0f;
float FREQ_TMR4 = FREQ_500;
float DT_TMR4 = DT_500;
long CNT_TMR4 = 0;

extern "C" void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {

        if(Classify_ValveType() == VALVETYPE_LIGHTDDV) { // Valve Type is LIGHT's DDV
            float LVDT_OUT = 0.0f;
            LVDT_sum = 0.0f;

            LVDT_L = 0;
            LVDT_H = 1;

            for (int ij = 0; ij < 120; ij++) {
                if (ij < 20) {
                    continue;
                } else if (ij == 20) {
                    //                LED = 1;
                } else if (ij == 100) {
                    LVDT_H = 0;
                    LVDT_L = 0;
                }

                ADC3->CR2 |= 0x40000000;
                LVDT_new = ((float)ADC3->DR); // Unit : pulse (0~4095)
                // if (DIR_VALVE_ENC < 0) LVDT_new = -LVDT_new;
                LVDT_sum = LVDT_sum + LVDT_new;
            }
            // LVDT_H = 0;
            // LVDT_L = 0;

            LVDT_new = LVDT_sum * 0.01f; // average of 100 samples  

            valve_pos.UpdateSen(LVDT_new, FREQ_TMR4, 250.0f); // Unit of 'valve_pos.sen' : pulse (0.0f ~ 4095.0f),

            CNT_TMR4++;
        } else { // if valve type is not DDV, LVDT is not used.
            LVDT_H = 0;
            LVDT_L = 0;

            CNT_TMR4 = 0;
        }
    }
    TIM4->SR = 0x0; // reset the status register
}

//------------------------------------------------
//     TMR2 : Control 5kHz
//------------------------------------------------
float FREQ_TMR2 = (float)FREQ_5k;
float DT_TMR2 = (float)DT_5k;
int cnt_trans = 0;
int cnt_jitter = 0;
int MODE_LED_Display = 0;  
extern "C" void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        
        ////////////////////////////////////////////////////////////////////
        ///////////////////// Frequency Checking LED ///////////////////////
        //////////////////////////////////////////////////////////////////// 
        if(MODE_LED_Display == 0) {
            static int CNT_LED1 = 0;
            if(CNT_LED1 % (int)(FREQ_5k*1.0f) == 0) {  // Period = 1.0 sec
                CNT_LED1 = 0;
                LED = !LED;
            } CNT_LED1++;
        } else if (MODE_LED_Display == 1) {
            static int CNT_LED2 = 0;
            if(CNT_LED2 % (int)(FREQ_5k*0.25f) == 0) { // Period = 0.25 sec
                CNT_LED2 = 0;
                LED = !LED;
            } CNT_LED2++;
        } else if (MODE_LED_Display == 2) { 
            static int CNT_LED3 = 0;
            if(CNT_LED3 % (int)(FREQ_5k*4.0f) == 0) {  // Period = 4.0 sec
                CNT_LED3 = 0;
                LED = !LED;
            } CNT_LED3++;
        } else if (MODE_LED_Display == 3) {
            LED = 1;
        } else if (MODE_LED_Display == 4) {
            LED = 0;
        } else {
            LED = 0;
        }

        // Valve parameter setting
        if (Classify_ValveType() == VALVETYPE_MOOG) {       // Moog two-stage valve parameters
            K_v = 1.03f; // Q = K_v*sqrt(deltaP)*tanh(C_d*Xv);
            C_d = 0.16f;
            mV_PER_mA = 500.0f;    // 5000mV/10mA
            mV_PER_pulse = 0.5f;   // 5000mV/10000pulse
            mA_PER_pulse = 0.001f; // 10mA/10000pulse
        } else if (Classify_ValveType() == VALVETYPE_KNR) {          // KNR two-stage valve parameters
            K_v = 0.5f;            // KNR (LPM >> mA) , 100bar
            mV_PER_mA = 166.6666f; // 5000mV/30mA
            mV_PER_pulse = 0.5f;   // 5000mV/10000pulse
            mA_PER_pulse = 0.003f; // 30mA/10000pulse
        } else if (Classify_ValveType() == VALVETYPE_LIGHTDDV) { // SW's valve
            C_d = 0.001f; // Q = C_d * Valve_pos * sqrt(deltaP*alpha/(1+alpha)) :
                            // Valve_pos = 10000, deltaP = 70, alpha = 1 -> Q = 5
        }

        // =====================================================================
        // CONTROL MODE CLASSIFICATION -----------------------------------------
        // =====================================================================
        int UTILITY_MODE = 0;
        int CONTROL_MODE = 0;

        if (CONTROL_UTILITY_MODE >= 20) {
            UTILITY_MODE = CONTROL_UTILITY_MODE;
            CONTROL_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_UTILMODE;
        } else if (CONTROL_UTILITY_MODE == 0) {
           UTILITY_MODE = MODE_NO_ACT;
            CONTROL_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_NO_ACT;
        } else {
            CONTROL_MODE = CONTROL_UTILITY_MODE;
            UTILITY_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_EXTERNAL;
        }

        ////////////////////////////////////////////////////////////////////
        //////// Reference Update (Selecting External or Internal) /////////
        ////////////////////////////////////////////////////////////////////       
        switch (REFERENCE_MODE) {
            case MODE_REF_NO_ACT: {
                break;
            }
            case MODE_REF_EXTERNAL: {
                pos.UpdateRef(pos.ref_ext, FREQ_5k, 200.0f);
                vel.UpdateRef(vel.ref_ext, FREQ_5k, 200.0f);
                torq.UpdateRef(torq.ref_ext, FREQ_5k, 200.0f);
                force.UpdateRef(force.ref_ext, FREQ_5k, 200.0f);
                valve_pos.UpdateRef(valve_pos.ref_ext, FREQ_5k, 200.0f);
                Vout.UpdateRef(Vout.ref_ext, FREQ_5k, 200.0f);
                break;
            }
            case MODE_REF_UTILMODE: {
                pos.ref = UtilityMode_PosRef;
                vel.ref = 0.0f;
                torq.ref = 0.0f;
                force.ref = 0.0f;
                break;
            }
            default:
                break;
        }
        ////////////////////////////////////////////////////////////////////
        /////////////////// UTILITY MODE ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////
        switch (UTILITY_MODE) {
        case MODE_NO_ACT: 
            break;
        case MODE_TORQUE_SENSOR_NULLING: 
            UtilFunc_TORQUE_SENSOR_NULLING();
            break;
        case MODE_FIND_HOME: 
            CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
            UtilFunc_FIND_HOME();
            break;
        // case MODE_ID_VALVEPOS_VS_PWM: 
        //     CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
        //     UtilFunc_ID_VALVEPOS_VS_PWM();
        //     break;
        // case MODE_ID_DEADZONE_AND_CENTER: 
        //     CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
        //     UtilFunc_ID_DEADZONE_AND_CENTER();
        //     break;
        // case MODE_ID_FLOWRATE_VS_VALVEPOS: 
        //     CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
        //     UtilFunc_ID_FLOWRATE_VS_VALVEPOS();
        //     break;
        default:
            break;
        }

        ////////////////////////////////////////////////////////////////////
        /////////////////// CONTROL MODE ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////

        switch (CONTROL_MODE) {
        case MODE_NO_ACT: {
            V_out = 0.0f;
            break;
        }
        case MODE_JOINT_CONTROL: { 

            // ===============================================================================================================================================
            // Pos. or Force Control Mode Parameter Setting 
            if (MODE_POS_FT_TRANS == 1) { // position control >> force control mode transition
                if (alpha_trans == 1.0f) MODE_POS_FT_TRANS = 2;
                alpha_trans = (float)(1.0f - cos(3.141592f * (float)cnt_trans * DT_TMR2 / 3.0f)) / 2.0f;
                cnt_trans++;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                pos.err_int = 0.0f;
                if ((float)cnt_trans * DT_TMR2 > 3.0f) {
                    MODE_POS_FT_TRANS = 2;
                }
            } else if (MODE_POS_FT_TRANS == 3) { // force control >> position control mode transition
                if (alpha_trans == 0.0f) MODE_POS_FT_TRANS = 0;
                alpha_trans = (float)(1.0f + cos(3.141592f * (float)cnt_trans * DT_TMR2 / 3.0f)) / 2.0f;
                cnt_trans++;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                pos.err_int = 0.0f;
                if ((float)cnt_trans * DT_TMR2 > 3.0f) {
                    MODE_POS_FT_TRANS = 0;
                }
            } else if (MODE_POS_FT_TRANS == 2) { // force control mode
                alpha_trans = 1.0f;
                pos.err_int = 0.0f;
                cnt_trans = 0;
            } else { // MODE_POS_FT_TRANS == 0, position control mode
                alpha_trans = 0.0f;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                cnt_trans = 0;
            }

            float temp_vel_pos = 0.0f; // desired velocity for position feedback control
            float temp_vel_FT = 0.0f;  // desired velocity for force/torque feedback control
            float temp_vel_ff = 0.0f;  // desired velocity for velocity feedforward control
            float temp_vel = 0.0f; // summation of all terms above

            // ===============================================================================================================================================
            // position feedback control command
            float wn_Pos = 2.0f * PI * 5.0f; // f_cut : 5Hz Position Control
            pos.err = pos.ref - pos.sen; // Unit : mm or deg
            vel.err = vel.ref - vel.sen; // Unit : mm/s or deg/s
            pos.err_int += pos.err*DT_5k;       
            float Kp_POS, Ki_POS;
            temp_vel_pos = wn_Pos * pos.err;
            if(fabs(temp_vel_pos)<=4.0f) { // under 5mm/s or 5deg/s
                Kp_POS = 3.0f * P_GAIN_JOINT_POSITION; 
                // Ki_POS = 2.0f * I_GAIN_JOINT_POSITION;
            } else if (fabs(temp_vel_pos)<=8.0f) {
                Kp_POS = (1.0f + 2.0f*(8.0f-fabs(temp_vel_pos))/4.0f) * P_GAIN_JOINT_POSITION; 
                // Ki_POS = (1.0f + 1.0f*(6.0f-fabs(temp_vel_pos))/3.0f) * I_GAIN_JOINT_POSITION;
            } else {
                Kp_POS = 1.0f * P_GAIN_JOINT_POSITION; 
                // Ki_POS = 1.0f * I_GAIN_JOINT_POSITION;
            }
            // Kp_POS = 1.0f * P_GAIN_JOINT_POSITION; 
            Ki_POS = 1.0f * I_GAIN_JOINT_POSITION;

            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                temp_vel_pos = 0.01f * wn_Pos * (Kp_POS * pos.err + Ki_POS * pos.err_int) * PI / 180.0f; // rad/s
                //                        L when P-gain = 100, f_cut = 5Hz
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                pos.err_int = (1.0f-alpha_int)*pos.err_int;
            } else { // Linear Mode
                temp_vel_pos = 0.01f * wn_Pos * (Kp_POS * pos.err + Ki_POS * pos.err_int); // mm/s
                //                        L when P-gain = 100, f_cut = 5Hz
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                pos.err_int = (1.0f-alpha_int)*pos.err_int;
            }

            // ===============================================================================================================================================
            // torque feedback control command
            float alpha_SpringDamper = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 30.0f));
            K_LPF = (1.0f - alpha_SpringDamper) * K_LPF + alpha_SpringDamper * K_SPRING;
            D_LPF = (1.0f - alpha_SpringDamper) * D_LPF + alpha_SpringDamper * D_DAMPER;

            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                float torq_ref_act = torq.ref + K_LPF * pos.err + D_LPF * vel.err; // unit : Nm
                if(torq_ref_act>PRES_SUPPLY*(float)PISTON_AREA_A*0.0001f*0.9f) torq_ref_act = PRES_SUPPLY*(float)PISTON_AREA_A*0.0001f*0.9f;
                if(torq_ref_act<-PRES_SUPPLY*(float)PISTON_AREA_B*0.0001f*0.9f) torq_ref_act = -PRES_SUPPLY*(float)PISTON_AREA_B*0.0001f*0.9f;

                torq.err = torq_ref_act - torq.sen;
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                torq.err_int = (1.0f-alpha_int)*torq.err_int;
                torq.err_int += torq.err / ((float)TMR_FREQ_5k);
                // if (torq.err > 5.0f || torq.err < -5.0f) {
                //     torq.err_int += torq.err / ((float)TMR_FREQ_5k);
                // }
                if(I_GAIN_JOINT_TORQUE*force.err_int>3000.0f) torq.err_int = 3000.0f/I_GAIN_JOINT_TORQUE;
                if(I_GAIN_JOINT_TORQUE*force.err_int<-3000.0f) torq.err_int = -3000.0f/I_GAIN_JOINT_TORQUE;
                temp_vel_FT = 0.001f * (P_GAIN_JOINT_TORQUE * torq.err + I_GAIN_JOINT_TORQUE * torq.err_int); // Nm >> rad/s
            } else { // Linear Mode
                float force_ref_act = force.ref + K_LPF * pos.err + D_LPF * vel.err; // unit : N
                if(force_ref_act>PRES_SUPPLY*(float)PISTON_AREA_A*0.1f*0.9f) force_ref_act = PRES_SUPPLY*(float)PISTON_AREA_A*0.1f*0.9f;
                if(force_ref_act<-PRES_SUPPLY*(float)PISTON_AREA_B*0.1f*0.9f) force_ref_act = -PRES_SUPPLY*(float)PISTON_AREA_B*0.1f*0.9f;

                force.err = force_ref_act - force.sen;
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                force.err_int = (1.0f-alpha_int)*force.err_int;
                // force.err_int += force.err / ((float)TMR_FREQ_5k);
                if (force.err < 20.0f && force.err > -20.0f) {
                    force.err = 0.0f;
                } else {
                    force.err_int += force.err / ((float)TMR_FREQ_5k);
                }
                if(I_GAIN_JOINT_TORQUE*force.err_int>300000.0f) force.err_int = 300000.0f/I_GAIN_JOINT_TORQUE;
                if(I_GAIN_JOINT_TORQUE*force.err_int<-300000.0f) force.err_int = -300000.0f/I_GAIN_JOINT_TORQUE;
                temp_vel_FT = 0.001f * (P_GAIN_JOINT_TORQUE * force.err + I_GAIN_JOINT_TORQUE * force.err_int); // N >> mm/s
            }

            // ========================================================================================================================================
            // velocity feedforward command
            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref * PI / 180.0f; // rad/s
            } else { // Linear Mode
                temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref; // mm/s
            }

            // =================================================================================================================================================
            // command integration
            temp_vel = (1.0f - alpha_trans) * temp_vel_pos + // Position Control
                        alpha_trans * temp_vel_FT + // + Torque Control
                        temp_vel_ff; //  + Velocity Feedforward

            float Qact = 0.0f; // required flow rate
            float ValvePos_JC = 0.0f; // Valve Position Ref for Joint Control
            if (temp_vel > 0.0f) {
                Qact = temp_vel * ((float)PISTON_AREA_A * 0.00006f); // mm^3/sec >> LPM
                if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                    // ValvePos_JC = tanh_inv(Qact /(K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)))) / C_d*1000.0f;
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                } else { // SW valve
                    // ValvePos_JC =  (float)VALVE_CENTER + Qact / (C_d * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)));
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                }
            } else {
                Qact = temp_vel * ((float)PISTON_AREA_B * 0.00006f); // mm^3/sec >> LPM
                if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                    // ValvePos_JC = tanh_inv(Qact / (K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)))) / C_d*1000.0f;
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                  
                } else { // SW valve
                    // ValvePos_JC = (float)VALVE_CENTER + Qact / (C_d * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)));
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                }
            }

            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve                
                // Anti-windup for FT
                if (I_GAIN_JOINT_TORQUE > 0.001f) {
                    float Ka = 0.1f;
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        float ValvePos_rem = ValvePos_JC - (float)VALVE_MAX_POS;
                        ValvePos_JC = (float)VALVE_MAX_POS;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)) * tanh(C_d * ValvePos_rem) / ((float)PISTON_AREA_A * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        float ValvePos_rem = ValvePos_JC - (float)VALVE_MIN_POS;
                        ValvePos_JC = (float)VALVE_MIN_POS;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)) * tanh(C_d * ValvePos_rem) / ((float)PISTON_AREA_B * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    }
                } else {
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        ValvePos_JC = (float)VALVE_MAX_POS;
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        ValvePos_JC = (float)VALVE_MIN_POS;
                    }
                }
                V_out = VALVE_POS_CONTROL(ValvePos_JC);
            } else {  // SW valve
                // Anti-windup for FT
                if (I_GAIN_JOINT_TORQUE > 0.001f) {
                    float Ka = 0.1f;
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        float valve_pos_rem = ValvePos_JC - (float)VALVE_MAX_POS;
                        ValvePos_JC = (float)VALVE_MAX_POS;
                        float temp_vel_rem =  C_d * valve_pos_rem * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)) /((float)PISTON_AREA_A * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        float valve_pos_rem = ValvePos_JC - (float)VALVE_MIN_POS;
                        ValvePos_JC = (float)VALVE_MIN_POS;
                        float temp_vel_rem = C_d * valve_pos_rem * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)) / ((float)PISTON_AREA_B * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    }
                } else {
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        ValvePos_JC = (float)VALVE_MAX_POS;
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        ValvePos_JC = (float)VALVE_MIN_POS;
                    }
                }
                V_out = VALVE_POS_CONTROL(ValvePos_JC);
            }
            break;
        }
        case MODE_VALVE_POSITION_CONTROL: {
            // Two-stage Valve : ValvePosition = ValveInputCurrent (-10000.0f ~ 10000.0f)
            // Direct Drive Valve : ValvePosition = LVDT Value (0.0f ~ 4095.0f)
            V_out = VALVE_POS_CONTROL(valve_pos.ref);
            break;
        }
        
        case MODE_VALVE_OPEN_LOOP: {
            V_out = Vout.ref;
            break;
        }

        default:
            break;
        }

        ////////////////////////////////////////////////////////////////////
        ///////////////////  PWM Command ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////

        if (DIR_VALVE < 0) {
            V_out = -V_out;
        }

        // Output Voltage Linearization & Dead Zone Cancellation (Electrical dead-zone) ----------------------------------------------------
        if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve

            // //  Dead Zone Cancellation & Linearization 
            // V_out = Mapping_OutputVoltage2PWMCommand_MoogTSV(V_out);

        } else if (Classify_ValveType()==VALVETYPE_LIGHTDDV) { //////////////////////////sw valve

            // // // L6205D
            // // if (V_out > 0)
            // //     V_out = 800.0f + V_out * 1.1275f;
            // // else if (V_out < 0)
            // //     V_out = -800.0f + V_out * 1.1275f;
            // // else
            // //     V_out = 0.0f;
            // V_out = Mapping_OutputVoltage2PWMCommand_MoogTSV(V_out);

            // Jitter to remove spool friction
            V_out = V_out + 1000.0f * sin(2.0f * PI * 400.0f * ((float)cnt_jitter) * DT_5k); // 4000, 400Hz
            cnt_jitter++;
            if (cnt_jitter == DT_5k)
                cnt_jitter = 0;
        }

        if (V_out >= VALVE_VOLTAGE_LIMIT * 1000.0f) {
            V_out = VALVE_VOLTAGE_LIMIT * 1000.0f;
        } else if (V_out <= -VALVE_VOLTAGE_LIMIT * 1000.0f) {
            V_out = -VALVE_VOLTAGE_LIMIT * 1000.0f;
        }

        PWM_out = V_out / (SUPPLY_VOLTAGE * 1000.0f); // Unit : -1.0 ~ 1.0

        // Saturation of output voltage
        if (PWM_out > 1.0f)
            PWM_out = 1.0f;
        else if (PWM_out < -1.0f)
            PWM_out = -1.0f;

        if (PWM_out > 0.0f) {
            TIM1->CCR1 = (TMR1_COUNT) * (PWM_out);
            TIM1->CCR2 = (TMR1_COUNT) * (0.0f);
        } else {
            TIM1->CCR1 = (TMR1_COUNT) * (0.0f);
            TIM1->CCR2 = 0.0f - (TMR1_COUNT) * (PWM_out);
        }

        ////////////////////////////////////////////////////////////////////////////
        //////////////////////  Data transmission through CAN //////////////////////
        ////////////////////////////////////////////////////////////////////////////

        //        if (TMR2_COUNT_CAN_TX % (int) ((int) TMR_FREQ_5k/CAN_FREQ) == 0) {
        // if (TMR2_COUNT_CAN_TX % (int)((int)TMR_FREQ_5k / 400) == 0) {
        if (TMR2_COUNT_CAN_TX % 12 == 0) {

            // Position, Velocity, and Torque (ID:1200)
            if (flag_data_request[0]) {
                if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Actuator
                    CAN_TX_POSnFT((int16_t)(pos.sen * 200.0f), 
                                  (int16_t)(vel.sen * 20.0f),
                                  (int16_t)(torq.sen * TORQUE_SENSOR_PULSE_PER_TORQUE * 10.0f));
                } else { // Linear Actuator
                    CAN_TX_POSnFT((int16_t)(pos.sen * 200.0f), 
                                (int16_t)(vel.sen * 20.0f),
                                (int16_t)(force.sen * TORQUE_SENSOR_PULSE_PER_TORQUE * 10.0f));
                }
            }

            // Valve Position (ID:1300)
            if (flag_data_request[1]) {
                CAN_TX_VALVEPOSnPWM((int16_t)(valve_pos.sen), 0, 0);
            }

            // Pressure (ID:1400)
            if (flag_data_request[2]) {
                CAN_TX_PRESSURE((int16_t)(pres_A.sen*100.0f), (int16_t)(pres_B.sen*100.0f));
                // CAN_TX_PRESSURE(1401,1402);
            }

            // // Something for debugging (ID:1500)
            // if (flag_data_request[3]) {
            //     // CAN_TX_SOMETHING((int16_t)DebugVar_Float[0],(int16_t)DebugVar_Float[1],(int16_t)DebugVar_Int[0],(int16_t)DebugVar_Int[1]);
            //     CAN_TX_SOMETHING((int16_t)DebugVar_Float[0],(int16_t)DebugVar_Float[1],(int16_t)DebugVar_Float[2],(int16_t)DebugVar_Float[3]);
            //     // CAN_TX_SOMETHING(1501,1501,1502,1503);
            // }

            // // Transmit Current Board Information or Alart (ID:1600)
            // CAN_TX_ALART();

            TMR2_COUNT_CAN_TX = 0;
        }
        TMR2_COUNT_CAN_TX++;
    }
    TIM2->SR = 0x0; // reset the status register
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ======================================= Utility Functions  ==================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



float Mapping_ValvePos2PWM(float _REF_VALVE_POS) {
    float _PWM = 0.0;
    
    if(_REF_VALVE_POS < VALVE_POS_VS_PWM_Sorted[0]) {
        _PWM = PWMs_for_ValvePosID_Sorted[0]*1000.0f;
        return _PWM;
    }

    if(_REF_VALVE_POS > VALVE_POS_VS_PWM_Sorted[NUM_VALVEPOS_VS_PWM_ID-1]) {
        _PWM = PWMs_for_ValvePosID_Sorted[NUM_VALVEPOS_VS_PWM_ID-1]*1000.0f;
        return _PWM;
    }

    for (int i = 0; i < (NUM_VALVEPOS_VS_PWM_ID-1); i++) {
        if (_REF_VALVE_POS >= VALVE_POS_VS_PWM_Sorted[i] && _REF_VALVE_POS <= VALVE_POS_VS_PWM_Sorted[i+1] ) {
            _PWM = PWMs_for_ValvePosID_Sorted[i] + (PWMs_for_ValvePosID_Sorted[i+1]-PWMs_for_ValvePosID_Sorted[i])*(_REF_VALVE_POS-VALVE_POS_VS_PWM_Sorted[i])/(VALVE_POS_VS_PWM_Sorted[i+1]-VALVE_POS_VS_PWM_Sorted[i]);
            _PWM = _PWM * 1000.0f;
            return _PWM;
        }
    }
}

float Mapping_FlowRate2ValvePos(float _REF_FLOWRATE) {

    float _Ps_ID = 100.0; // Table was obtained at Ps = 100 bar & double rod (No Load) 
    float _Ps = PRES_SUPPLY;
    float _a3 = alpha3;

    float _REF_FLOWRATE_Scaled = 0.0f;
    if(_REF_FLOWRATE > 0.0f) {
        _REF_FLOWRATE_Scaled = sqrt((_a3+1.0f)/(2.0f*_a3)*(_Ps_ID/_Ps))*_REF_FLOWRATE;
    } else {
        _REF_FLOWRATE_Scaled = sqrt((_a3+1.0f)/(2.0f)*(_Ps_ID/_Ps))*_REF_FLOWRATE;
    }

    float _REF_VALVE_POS_OFF = 0.0f;
    float _REF_VALVE_POS = 0.0f;
    
    float FlowRate_VS_ValvePosOff_Masked[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f,};
    
    // float mask[3] = {0.0f,1.0f,0.0f};
    // ArrayMasking_1st(FlowRate_VS_ValvePosOff_Sorted, FlowRate_VS_ValvePosOff_Masked, NUM_FLOWRATE_VS_VALVEPOS_ID, mask);
    float mask[5] = {0.5f,0.0f,0.0f,0.0f,0.5f};
    ArrayMasking_2nd(FlowRate_VS_ValvePosOff_Sorted, FlowRate_VS_ValvePosOff_Masked, NUM_FLOWRATE_VS_VALVEPOS_ID, mask);

    if(DIR_VALVE_ENC > 0) {
        if(_REF_FLOWRATE_Scaled < FlowRate_VS_ValvePosOff_Masked[0]) {
            _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[0];
            _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
            return _REF_VALVE_POS;
        }
        if(_REF_FLOWRATE_Scaled > FlowRate_VS_ValvePosOff_Masked[NUM_FLOWRATE_VS_VALVEPOS_ID-1]) {
            _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID-1];
            _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
            return _REF_VALVE_POS;
        }

        for (int i = 0; i < (NUM_FLOWRATE_VS_VALVEPOS_ID-1); i++) {
            if (_REF_FLOWRATE_Scaled >= FlowRate_VS_ValvePosOff_Masked[i] && _REF_FLOWRATE_Scaled <= FlowRate_VS_ValvePosOff_Masked[i+1] ) {
                _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[i] + (ValvePosOff_for_FlowrateID_Sorted[i+1]-ValvePosOff_for_FlowrateID_Sorted[i])*(_REF_FLOWRATE_Scaled-FlowRate_VS_ValvePosOff_Masked[i])/(FlowRate_VS_ValvePosOff_Masked[i+1]-FlowRate_VS_ValvePosOff_Masked[i]);
                _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
                return _REF_VALVE_POS;
            }
        }
    } else {
        if(_REF_FLOWRATE_Scaled > FlowRate_VS_ValvePosOff_Masked[0]) {
            _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[0];
            _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
            return _REF_VALVE_POS;
        }
        if(_REF_FLOWRATE_Scaled < FlowRate_VS_ValvePosOff_Masked[NUM_FLOWRATE_VS_VALVEPOS_ID-1]) {
            _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID-1];
            _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
            return _REF_VALVE_POS;
        }

        for (int i = 0; i < (NUM_FLOWRATE_VS_VALVEPOS_ID-1); i++) {
            if (_REF_FLOWRATE_Scaled <= FlowRate_VS_ValvePosOff_Masked[i] && _REF_FLOWRATE_Scaled >= FlowRate_VS_ValvePosOff_Masked[i+1] ) {
                _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[i] + (ValvePosOff_for_FlowrateID_Sorted[i+1]-ValvePosOff_for_FlowrateID_Sorted[i])*(_REF_FLOWRATE_Scaled-FlowRate_VS_ValvePosOff_Masked[i])/(FlowRate_VS_ValvePosOff_Masked[i+1]-FlowRate_VS_ValvePosOff_Masked[i]);
                _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
                return _REF_VALVE_POS;
            }
        }
    }
}

float VALVE_POS_CONTROL(float REF_VALVE_POS) {
    
    static float REF_VALVE_POS_OLD = 0.0f;
    float _Vout = 0.0f;
    
    if (REF_VALVE_POS > (float)VALVE_MAX_POS) {
        REF_VALVE_POS = (float)VALVE_MAX_POS;
    } else if (REF_VALVE_POS < (float)VALVE_MIN_POS) {
        REF_VALVE_POS = (float)VALVE_MIN_POS;
    }

    // Valve Spool Position Control for LIGHT's DDV
    if(Classify_ValveType()==VALVETYPE_LIGHTDDV) {
        VALVE_PWM_RAW_FF = 1.0f*Mapping_ValvePos2PWM(REF_VALVE_POS); // Unit : mV

        valve_pos_err = (float)(REF_VALVE_POS - valve_pos.sen);
        valve_pos_err_diff = (valve_pos_err - valve_pos_err_old)*FREQ_5k;
        valve_pos_err_old = valve_pos_err;
        valve_pos_err_sum += valve_pos_err*DT_5k;
        if (valve_pos_err_sum > 1000.0f) valve_pos_err_sum = 1000.0f;
        if (valve_pos_err_sum < -1000.0f) valve_pos_err_sum = -1000.0f;

        VALVE_PWM_RAW_FB = P_GAIN_VALVE_POSITION * valve_pos_err +
                I_GAIN_VALVE_POSITION * valve_pos_err_sum +
                D_GAIN_VALVE_POSITION * valve_pos_err_diff;

        _Vout = VALVE_PWM_RAW_FF + VALVE_PWM_RAW_FB;

        if(I_GAIN_VALVE_POSITION>0.001f) {
            float V_MAX = VALVE_VOLTAGE_LIMIT * 1000.0f; // Maximum Voltage, Unit : mV
            float V_rem = 0.0f;
            float Ka = 0.003f;
            if (_Vout > V_MAX) {
                V_rem = Ka * (_Vout - V_MAX);
                _Vout = V_MAX;
                valve_pos_err_sum = valve_pos_err_sum - V_rem / I_GAIN_VALVE_POSITION;
            } else if (_Vout < -V_MAX) {
                V_rem = Ka * (_Vout - (-V_MAX));
                _Vout = -V_MAX;
                valve_pos_err_sum = valve_pos_err_sum - V_rem / I_GAIN_VALVE_POSITION;
            }
        } else {
            float V_MAX = VALVE_VOLTAGE_LIMIT * 1000.0f; // Maximum Voltage, Unit : mV
            if (_Vout > V_MAX) {
                _Vout = V_MAX;
            } else if (_Vout < -V_MAX) {
                _Vout = -V_MAX;
            }
        }
        // float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 5.0f));
        // valve_pos_err_sum = (1.0f-alpha_int)*valve_pos_err_sum;
    } 

    // Valve Spool Position Control for Two-Stage Valve (TSV)
    else if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) {

        // Moog Valve Current Control Gain
        float R_model = 500.0f; // ohm
        float L_model = 1.2f;
        float w0 = 2.0f * 3.14f * 70.0f;
        float KP_I = 1.0f * L_model * w0;
        float KI_I = 1.0f * R_model * w0;
        // KNR Valve Current Control Gain
        if (Classify_ValveType() == VALVETYPE_KNR) { // KNR Valve
            R_model = 163.0f; // ohm
            L_model = 1.0f;
            w0 = 2.0f * 3.14f * 80.0f;
            KP_I = 1.0f * L_model * w0;
            KI_I = 0.08f * R_model * w0;
        }

        VALVE_PWM_RAW_FF = 1.0f*Mapping_ValvePos2PWM(REF_VALVE_POS); // Unit : mV
        // VALVE_PWM_RAW_FF = 1.0f*(R_model*(REF_VALVE_POS/1000.0f)); // Unit : mV

        valve_pos_err = (float)(REF_VALVE_POS - valve_pos.sen)/1000.0f; // Unit : mA
        valve_pos_err_sum += valve_pos_err*DT_5k;
        if (valve_pos_err_sum > 1000.0f) valve_pos_err_sum = 1000.0f;
        if (valve_pos_err_sum < -1000.0f) valve_pos_err_sum = -1000.0f;

        VALVE_PWM_RAW_FB = KP_I * valve_pos_err + KI_I * valve_pos_err_sum; // Unit : mV

        _Vout = VALVE_PWM_RAW_FF + VALVE_PWM_RAW_FB; // Unit : mV
        // _Vout = VALVE_PWM_RAW_FF; // Unit : mV
        // _Vout = VALVE_PWM_RAW_FB; // Unit : mV

        float V_MAX = VALVE_VOLTAGE_LIMIT * 1000.0f; // Maximum Voltage, Unit : mV
        float V_rem = 0.0f;
        float Ka = 1.0f;
        if (_Vout > V_MAX) {
            V_rem = Ka * (_Vout - V_MAX);
            _Vout = V_MAX;
            valve_pos_err_sum = valve_pos_err_sum - V_rem / KP_I;
        } else if (_Vout < -V_MAX) {
            V_rem = Ka * (_Vout - (-V_MAX));
            _Vout = -V_MAX;
            valve_pos_err_sum = valve_pos_err_sum - V_rem / KP_I;
        }
    }

    return _Vout;
}

float PosCtrl4UtilFunc(float _PosRef)
{
    float temp_vel_UtilFunc = 0.0f;
    float wn_Pos = 2.0f * PI * 3.0f; // f_cut : 5Hz Position Control
    if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
        temp_vel_UtilFunc =  wn_Pos * (_PosRef - pos.sen) * PI / 180.0f; // deg/s >> rad/s
    } else { // Linear Mode
        temp_vel_UtilFunc =  wn_Pos * (_PosRef - pos.sen); // mm/s
    }

    float Qact_UtilFunc = 0.0f;
    float ValvePos_UtilFunc = 0.0f;
    if (temp_vel_UtilFunc > 0.0f) {
        Qact_UtilFunc = temp_vel_UtilFunc * ((float)PISTON_AREA_A * 0.00006f); // mm^3/sec >> LPM
        if(DIR_VALVE_ENC > 0) {
            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                ValvePos_UtilFunc = tanh_inv(Qact_UtilFunc /(K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)))) / C_d*1000.0f;
            } else { // SW valve
                ValvePos_UtilFunc =  (float)VALVE_CENTER + Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)));
            }
        } else {
            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                ValvePos_UtilFunc = -tanh_inv(Qact_UtilFunc /(K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)))) / C_d*1000.0f;
            } else { // SW valve
                ValvePos_UtilFunc =  (float)VALVE_CENTER - Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)));
            }
        }
    } else {
        Qact_UtilFunc = temp_vel_UtilFunc * ((float)PISTON_AREA_B * 0.00006f); // mm^3/sec >> LPM
        if(DIR_VALVE_ENC > 0) {
            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                ValvePos_UtilFunc = tanh_inv(Qact_UtilFunc / (K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)))) / C_d*1000.0f;
            } else { // SW valve
                ValvePos_UtilFunc = (float)VALVE_CENTER + Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)));
            }
        } else {
            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                ValvePos_UtilFunc = -tanh_inv(Qact_UtilFunc / (K_v * sqrt(PRES_SUPPLY  / (alpha3 + 1.0f)))) / C_d*1000.0f;
            } else { // SW valve
                ValvePos_UtilFunc = (float)VALVE_CENTER - Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY  / (alpha3 + 1.0f)));
            }
        }
    }

    if (ValvePos_UtilFunc > (float)VALVE_MAX_POS) {
        ValvePos_UtilFunc = (float)VALVE_MAX_POS;
    } else if (ValvePos_UtilFunc < (float)VALVE_MIN_POS) {
        ValvePos_UtilFunc = (float)VALVE_MIN_POS;
    }

    return VALVE_POS_CONTROL(ValvePos_UtilFunc);
}



void UtilFunc_TORQUE_SENSOR_NULLING(void) 
{
    static float FORCE_pulse_sum = 0.0f;
    static float PresA_pulse_sum = 0.0f;
    static float PresB_pulse_sum = 0.0f;

    // DAC Voltage reference set
    float VREF_TuningGain = -0.000003f;
    if (TMR3_COUNT_TORQUE_NULL < TMR_FREQ_5k * 5) {
        if (SENSING_MODE == 0) { // Force Sensor (Loadcell)
            FORCE_pulse_sum = FORCE_pulse_sum + force.sen * TORQUE_SENSOR_PULSE_PER_TORQUE;
            if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                float FORCE_pluse_mean = FORCE_pulse_sum / 10.0f;
                FORCE_pulse_sum = 0.0f;
                FORCE_VREF += VREF_TuningGain * (0.0f - FORCE_pluse_mean);
                if (FORCE_VREF > 3.3f) FORCE_VREF = 3.3f;
                if (FORCE_VREF < 0.0f) FORCE_VREF = 0.0f;
                dac_1 = FORCE_VREF / 3.3f;
            }
        } else if (SENSING_MODE == 1) { // Pressure Sensor
            PresA_pulse_sum += pres_A.sen * PRES_SENSOR_A_PULSE_PER_BAR;
            PresB_pulse_sum += pres_B.sen * PRES_SENSOR_B_PULSE_PER_BAR;
            if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                float PresA_pluse_mean = PresA_pulse_sum / 10.0f;
                float PresB_pluse_mean = PresB_pulse_sum / 10.0f;
                PresA_pulse_sum = 0.0f;
                PresB_pulse_sum = 0.0f;

                PRES_A_VREF += VREF_TuningGain * (0.0f - PresA_pluse_mean);
                if (PRES_A_VREF > 3.3f) PRES_A_VREF = 3.3f;
                if (PRES_A_VREF < 0.0f) PRES_A_VREF = 0.0f;
                dac_1 = PRES_A_VREF / 3.3f;
                PRES_B_VREF += VREF_TuningGain * (0.0f - PresB_pluse_mean);
                if (PRES_B_VREF > 3.3f) PRES_B_VREF = 3.3f;
                if (PRES_B_VREF < 0.0f) PRES_B_VREF = 0.0f;
                dac_2 = PRES_B_VREF / 3.3f;
            }
        }
        TMR3_COUNT_TORQUE_NULL++;
    } else {
        if (SENSING_MODE == 0) { // Force Sensor (Loadcell)
            FORCE_pulse_sum = 0.0f;
            dac_1 = FORCE_VREF / 3.3f;
            spi_eeprom_write(RID_FORCE_SENSOR_VREF, (int16_t)(FORCE_VREF * 1000.0f));
        } else if (SENSING_MODE == 1) {
            PresA_pulse_sum = 0.0f;
            PresB_pulse_sum = 0.0f;
            dac_1 = PRES_A_VREF / 3.3f;
            dac_2 = PRES_B_VREF / 3.3f;
            spi_eeprom_write(RID_PRES_A_SENSOR_VREF, (int16_t)(PRES_A_VREF * 1000.0f));
            spi_eeprom_write(RID_PRES_B_SENSOR_VREF, (int16_t)(PRES_B_VREF * 1000.0f));
        }
        CONTROL_UTILITY_MODE = MODE_NO_ACT;
        TMR3_COUNT_TORQUE_NULL = 0;
        ALART_FLAG_ON(ALART_UTILFUNC_DONE);
    }
}

void UtilFunc_FIND_HOME(void)
{
    if (FINDHOME_STAGE == FINDHOME_INIT) {
        REFERENCE_MODE = MODE_REF_UTILMODE;
        CNT4UtilityMode = 0;
        cnt_touch_end = 0;
        pos.ref = pos.sen;
        vel.ref = 0.0f;
        UtilityMode_PosRef = pos.sen;
        UtilityMode_Pos = pos.sen;
        UtilityMode_PosOld = pos.sen;
        FINDHOME_STAGE = FINDHOME_GOTOLIMIT;
    } else if (FINDHOME_STAGE == FINDHOME_GOTOLIMIT) {

        float GOTOHOME_SPEED = 20.0f;       // 20mm/s or 20deg/s
        if (HOMEPOS_OFFSET > 0) {
            UtilityMode_PosRef = UtilityMode_PosRef + GOTOHOME_SPEED * DT_5k;
        } else {
            UtilityMode_PosRef = UtilityMode_PosRef - GOTOHOME_SPEED * DT_5k;
        }
        Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

        CNT4UtilityMode++;
        if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz checking
            UtilityMode_Pos = pos.sen;
            UtilityMode_Vel = UtilityMode_Pos - UtilityMode_PosOld;
            UtilityMode_PosOld = UtilityMode_Pos;
            CNT4UtilityMode = 0;
        }

        if(fabs(UtilityMode_Vel) <= GOTOHOME_SPEED*0.1f) {
            cnt_touch_end = cnt_touch_end + 1;
        } else {
            cnt_touch_end = 0;
        }

        if ((cnt_touch_end >= 3 * TMR_FREQ_5k) || CNT4UtilityMode >= 10 * TMR_FREQ_5k) { // wait for 3sec
            ENC_SET((long)((long)HOMEPOS_OFFSET));
            Flag_PosMidFound = false; 
            UtilityMode_PosRef_INIT = ((float)HOMEPOS_OFFSET)/ENC_PULSE_PER_POSITION; // pulse >> deg or mm
            UtilityMode_Pos = 0;
            UtilityMode_PosOld = 0;
            UtilityMode_Vel = 0;

            CNT4UtilityMode = 0;
            cnt_touch_end = 0;
            pos.ref = 0.0f;
            FINDHOME_STAGE = FINDHOME_ZEROPOSE;
        }
    } else if (FINDHOME_STAGE == FINDHOME_ZEROPOSE) {
        int T_move = 2*TMR_FREQ_5k;
        UtilityMode_PosRef = UtilityMode_PosRef_INIT + (0.0f - UtilityMode_PosRef_INIT) * 0.5f *(1.0f - cosf(PI * (float)CNT4UtilityMode / (float)T_move));
        Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

        CNT4UtilityMode++;
        if (CNT4UtilityMode >= T_move) {
            CNT4UtilityMode = 0;
            pos.ref = 0.0f;
            vel.ref = 0.0f;
            Flag_FINDHOME_DONE = true;
            // CONTROL_UTILITY_MODE = MODE_NO_ACT;
            CONTROL_UTILITY_MODE = MODE_JOINT_CONTROL;
            REFERENCE_MODE = MODE_REF_EXTERNAL;
            FINDHOME_STAGE = FINDHOME_INIT;
            ALART_FLAG_ON(ALART_UTILFUNC_DONE);
        }
    }
}
