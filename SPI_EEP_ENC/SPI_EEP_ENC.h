#ifndef _SPI_EEP_ENC_H_
#define _SPI_EEP_ENC_H_

#include "mbed.h"


void spi_eeprom_ready(void);
void spi_eeprom_write(unsigned short add, int32_t data);
int spi_eeprom_read(unsigned short add);
void spi_eeprom_call_data(void);

void spi_enc_set_clear(void);
void spi_enc_set_init(void);
int spi_enc_read(void);

void SPI_VREF_DAC_WRITE_CHANNEL(unsigned int channel, unsigned int mode, unsigned int value);
void SPI_VREF_DAC_WRITE(float VA, float VB, float VC, float VD);
void SPI_VREF_DAC_SET_ZERO(void);

/*******************************************************************************
 * ROM DATA ADDRESS ID
 ******************************************************************************/

#define             RID_BNO                             0
#define             RID_OPERATING_MODE                  1
#define             RID_CAN_FREQ                        2
#define             RID_JOINT_ENC_DIR                   3
#define             RID_VALVE_DIR                       4
#define             RID_VALVE_ENC_DIR                   5
#define             RID_VOLATGE_SUPPLY                  6
#define             RID_VOLTAGE_VALVE                   7

#define             RID_P_GAIN_VALVE_POSITION           8
#define             RID_I_GAIN_VALVE_POSITION           9
#define             RID_D_GAIN_VALVE_POSITION           10

#define             RID_P_GAIN_JOINT_POSITION           11
#define             RID_I_GAIN_JOINT_POSITION           12
#define             RID_D_GAIN_JOINT_POSITION           13

#define             RID_P_GAIN_JOINT_TORQUE             14
#define             RID_I_GAIN_JOINT_TORQUE             15
#define             RID_D_GAIN_JOINT_TORQUE             16

#define             RID_VALVE_DEADZONE_PLUS             17
#define             RID_VALVE_DEADZONE_MINUS            18

#define             RID_VELOCITY_COMP_GAIN              19
//#define             RID_COMPLIANCE_GAIN                 20

#define             RID_VALVE_CENTER                    21

#define             RID_VALVE_FF                        22

#define             RID_BULK_MODULUS                    23

#define             RID_CHAMBER_VOLUME_A                24
#define             RID_CHAMBER_VOLUME_B                25

#define             RID_PISTON_AREA_A                   26
#define             RID_PISTON_AREA_B                   27

#define             RID_PRES_SUPPLY                     28
#define             RID_PRES_RETURN                     29

#define             RID_ENC_LIMIT_PLUS                  30
#define             RID_ENC_LIMIT_MINUS                 31

#define             RID_STROKE                          32

//#define             RID_VALVE_LIMIT_PLUS                34
//#define             RID_VALVE_LIMIT_MINUS               35

#define             RID_ENC_PULSE_PER_POSITION          36
#define             RID_TORQUE_SENSOR_PULSE_PER_TORQUE  37
#define             RID_PRES_SENSOR_A_PULSE_PER_BAR     38
#define             RID_PRES_SENSOR_B_PULSE_PER_BAR     39

#define             RID_FRICTION                        40
#define             RID_HOMEPOS_OFFSET                  41
#define             RID_HOMEPOS_VALVE_OPENING           42

#define             RID_FORCE_SENSOR_VREF               45

#define             RID_PRES_A_SENSOR_VREF              50
#define             RID_PRES_B_SENSOR_VREF              51

#define             RID_VALVE_MAX_POS                   52
#define             RID_VALVE_MIN_POS                   53

#define             RID_VALVE_POS_NUM                   54
//#define             RID_DDV_CENTER                      55
#define             RID_VALVE_CENTER_OFFSET             56
#define             RID_VALVE_ELECTRIC_CENTER           59

#define             RID_VALVE_GAIN_PLUS_1               60
#define             RID_VALVE_GAIN_MINUS_1              61
#define             RID_VALVE_GAIN_PLUS_2               62
#define             RID_VALVE_GAIN_MINUS_2              63
#define             RID_VALVE_GAIN_PLUS_3               64
#define             RID_VALVE_GAIN_MINUS_3              65
#define             RID_VALVE_GAIN_PLUS_4               66
#define             RID_VALVE_GAIN_MINUS_4              67
#define             RID_VALVE_GAIN_PLUS_5               68
#define             RID_VALVE_GAIN_MINUS_5              69

#define             RID_VALVE_POS_VS_PWM_0              70
#define             RID_VALVE_POS_VS_PWM_1              71
#define             RID_VALVE_POS_VS_PWM_2              72
#define             RID_VALVE_POS_VS_PWM_3              73
#define             RID_VALVE_POS_VS_PWM_4              74
#define             RID_VALVE_POS_VS_PWM_5              75
#define             RID_VALVE_POS_VS_PWM_6              76
#define             RID_VALVE_POS_VS_PWM_7              77
#define             RID_VALVE_POS_VS_PWM_8              78
#define             RID_VALVE_POS_VS_PWM_9              79
#define             RID_VALVE_POS_VS_PWM_10              80
#define             RID_VALVE_POS_VS_PWM_11              81
#define             RID_VALVE_POS_VS_PWM_12              82
#define             RID_VALVE_POS_VS_PWM_13              83
#define             RID_VALVE_POS_VS_PWM_14              84
#define             RID_VALVE_POS_VS_PWM_15              85
#define             RID_VALVE_POS_VS_PWM_16              86
#define             RID_VALVE_POS_VS_PWM_17              87
#define             RID_VALVE_POS_VS_PWM_18              88
#define             RID_VALVE_POS_VS_PWM_19              89
#define             RID_VALVE_POS_VS_PWM_20              90
#define             RID_VALVE_POS_VS_PWM_21              91
#define             RID_VALVE_POS_VS_PWM_22              92
#define             RID_VALVE_POS_VS_PWM_23              93
#define             RID_VALVE_POS_VS_PWM_24              94      
#define             RID_VALVE_POS_VS_PWM_25              95
#define             RID_VALVE_POS_VS_PWM_26              96
#define             RID_VALVE_POS_VS_PWM_27              97
#define             RID_VALVE_POS_VS_PWM_28              98
#define             RID_VALVE_POS_VS_PWM_29              99
#define             RID_VALVE_POS_VS_PWM_30              100
#define             RID_VALVE_POS_VS_PWM_31              101
#define             RID_VALVE_POS_VS_PWM_32              102
#define             RID_VALVE_POS_VS_PWM_33              103
#define             RID_VALVE_POS_VS_PWM_34              104
#define             RID_VALVE_POS_VS_PWM_35              105
#define             RID_VALVE_POS_VS_PWM_36              106
#define             RID_VALVE_POS_VS_PWM_37              107
#define             RID_VALVE_POS_VS_PWM_38              108
#define             RID_VALVE_POS_VS_PWM_39              109
#define             RID_VALVE_POS_VS_PWM_40              110
#define             RID_VALVE_POS_VS_PWM_41              111
#define             RID_VALVE_POS_VS_PWM_42              112
#define             RID_VALVE_POS_VS_PWM_43              113
#define             RID_VALVE_POS_VS_PWM_44              114
#define             RID_VALVE_POS_VS_PWM_45              115
#define             RID_VALVE_POS_VS_PWM_46              116
#define             RID_VALVE_POS_VS_PWM_47              117
#define             RID_VALVE_POS_VS_PWM_48              118
#define             RID_VALVE_POS_VS_PWM_49              119
#define             RID_VALVE_POS_VS_PWM_50              120

#define             RID_VALVE_POS_VS_FLOWRATE_0          130
#define             RID_VALVE_POS_VS_FLOWRATE_1          131
#define             RID_VALVE_POS_VS_FLOWRATE_2          132
#define             RID_VALVE_POS_VS_FLOWRATE_3          133
#define             RID_VALVE_POS_VS_FLOWRATE_4          134
#define             RID_VALVE_POS_VS_FLOWRATE_5          135
#define             RID_VALVE_POS_VS_FLOWRATE_6          136
#define             RID_VALVE_POS_VS_FLOWRATE_7          137
#define             RID_VALVE_POS_VS_FLOWRATE_8          138
#define             RID_VALVE_POS_VS_FLOWRATE_9          139
#define             RID_VALVE_POS_VS_FLOWRATE_10         140
#define             RID_VALVE_POS_VS_FLOWRATE_11         141
#define             RID_VALVE_POS_VS_FLOWRATE_12         142
#define             RID_VALVE_POS_VS_FLOWRATE_13         143
#define             RID_VALVE_POS_VS_FLOWRATE_14         144
#define             RID_VALVE_POS_VS_FLOWRATE_15         145
#define             RID_VALVE_POS_VS_FLOWRATE_16         146
#define             RID_VALVE_POS_VS_FLOWRATE_17         147
#define             RID_VALVE_POS_VS_FLOWRATE_18         148
#define             RID_VALVE_POS_VS_FLOWRATE_19         149
#define             RID_VALVE_POS_VS_FLOWRATE_20         150
#define             RID_VALVE_POS_VS_FLOWRATE_21         151
#define             RID_VALVE_POS_VS_FLOWRATE_22         152
#define             RID_VALVE_POS_VS_FLOWRATE_23         153
#define             RID_VALVE_POS_VS_FLOWRATE_24         154
#define             RID_VALVE_POS_VS_FLOWRATE_25         155
#define             RID_VALVE_POS_VS_FLOWRATE_26         156
#define             RID_VALVE_POS_VS_FLOWRATE_27         157
#define             RID_VALVE_POS_VS_FLOWRATE_28         158
#define             RID_VALVE_POS_VS_FLOWRATE_29         159
#define             RID_VALVE_POS_VS_FLOWRATE_30         160
#define             RID_VALVE_POS_VS_FLOWRATE_31         161
#define             RID_VALVE_POS_VS_FLOWRATE_32         162
#define             RID_VALVE_POS_VS_FLOWRATE_33         163
#define             RID_VALVE_POS_VS_FLOWRATE_34         164
#define             RID_VALVE_POS_VS_FLOWRATE_35         165
#define             RID_VALVE_POS_VS_FLOWRATE_36         166
#define             RID_VALVE_POS_VS_FLOWRATE_37         167
#define             RID_VALVE_POS_VS_FLOWRATE_38         168
#define             RID_VALVE_POS_VS_FLOWRATE_39         169
#define             RID_VALVE_POS_VS_FLOWRATE_40         170
#define             RID_VALVE_POS_VS_FLOWRATE_41         171
#define             RID_VALVE_POS_VS_FLOWRATE_42         172
#define             RID_VALVE_POS_VS_FLOWRATE_43         173
#define             RID_VALVE_POS_VS_FLOWRATE_44         174
#define             RID_VALVE_POS_VS_FLOWRATE_45         175
#define             RID_VALVE_POS_VS_FLOWRATE_46         176
#define             RID_VALVE_POS_VS_FLOWRATE_47         177
#define             RID_VALVE_POS_VS_FLOWRATE_48         178
#define             RID_VALVE_POS_VS_FLOWRATE_49         179
#define             RID_VALVE_POS_VS_FLOWRATE_50         180

#define             RID_VALVE_POS_VS_FLOWRATE_0_1         200
#define             RID_VALVE_POS_VS_FLOWRATE_1_1         201
#define             RID_VALVE_POS_VS_FLOWRATE_2_1         202
#define             RID_VALVE_POS_VS_FLOWRATE_3_1         203
#define             RID_VALVE_POS_VS_FLOWRATE_4_1         204
#define             RID_VALVE_POS_VS_FLOWRATE_5_1         205
#define             RID_VALVE_POS_VS_FLOWRATE_6_1         206
#define             RID_VALVE_POS_VS_FLOWRATE_7_1         207
#define             RID_VALVE_POS_VS_FLOWRATE_8_1         208
#define             RID_VALVE_POS_VS_FLOWRATE_9_1         209
#define             RID_VALVE_POS_VS_FLOWRATE_10_1         210
#define             RID_VALVE_POS_VS_FLOWRATE_11_1         211
#define             RID_VALVE_POS_VS_FLOWRATE_12_1         212
#define             RID_VALVE_POS_VS_FLOWRATE_13_1         213
#define             RID_VALVE_POS_VS_FLOWRATE_14_1         214
#define             RID_VALVE_POS_VS_FLOWRATE_15_1         215
#define             RID_VALVE_POS_VS_FLOWRATE_16_1         216
#define             RID_VALVE_POS_VS_FLOWRATE_17_1         217
#define             RID_VALVE_POS_VS_FLOWRATE_18_1         218
#define             RID_VALVE_POS_VS_FLOWRATE_19_1         219
#define             RID_VALVE_POS_VS_FLOWRATE_20_1         220

#define             RID_SENSING_MODE                    221
#define             RID_CURRENT_CONTROL_MODE            222
#define             RID_FLAG_VALVE_DEADZONE             223


#define             RID_K_SPRING                        57
#define             RID_D_DAMPER                        58

#endif //_SPI_H_