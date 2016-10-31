/*
 * commandlist.h
 *
 *  Created on: Oct 21, 2016
 *      Author: epenate
 */

#ifndef COMMANDLIST_H_
#define COMMANDLIST_H_


typedef enum CommandsKindList {
    M2W_IRLEDs_Off                  = 10,
    M2W_IRLED0_On                   = 11,
    M2W_IRLED0_Off                  = 12,
    M2W_IRLED1_On                   = 13,
    M2W_IRLED1_Off                  = 14,
    M2W_IRLED2_On                   = 15,
    M2W_IRLED2_Off                  = 16,
    M2W_IRLED3_On                   = 17,
    M2W_IRLED3_Off                  = 18,
    M2W_IRLED4_On                   = 19,
    M2W_IRLED4_Off                  = 20,
    M2W_IRLED5_On                   = 21,
    M2W_IRLED5_Off                  = 22,

    M2L_Battery_Monitor             = 50,
    M2L_Red_Out                     = 51,
    M2L_Green_Out                   = 52,
    M2L_Blue_Out                    = 53,
    M2L_Red_Middle                  = 54,
    M2L_Green_Middle                = 55,
    M2L_Blue_Middle                 = 56,
    M2L_Red_Center                  = 57,
    M2L_Green_Center                = 58,
    M2L_Blue_Center                 = 59,

    M2L_Acc_Threshold               = 60,
    M2L_Proximity_Threshold         = 61,

    L2M_Battery_Level               = 80,
    L2M_Acc_Status                  = 81,
    L2M_Proximity_Status            = 82,
} CommandsKindList;



typedef struct {
    int tipo;
    int CRC;
}qComandoSimple_t;

typedef struct {
    int tipo;
    int var1;
    int CRC;
}qComando1var_t;

typedef struct {
    int tipo;
    float var1;
    int CRC;
}qComando1float_t;


#endif /* COMMANDLIST_H_ */
