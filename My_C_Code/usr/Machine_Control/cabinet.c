#include "drv/analog.h"
#include "drv/motherboard.h"
//#include "SIL_specific.h"
#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/mb_sensor.h"
#include "usr/Machine_Control/inverter.h"
#include "usr/Machine_Control/hardware.h"
#include "usr/user_config.h"



//PWM CHANNELS
#define CAB_INV1_PHA	(1)
#define CAB_INV1_PHB	(0)

#define CAB_INV2_PHA	(3)
#define CAB_INV2_PHB	(5)
#define CAB_INV2_PHC    (4)

#define CAB_INV3_PHA	(6)
#define CAB_INV3_PHB	(8)
#define CAB_INV3_PHC	(7)

#define CAB_INV4_PHA	(10)
#define CAB_INV4_PHB	(9)

#define CAB_INV5_PHA	(12) 
#define CAB_INV5_PHB	(14)
#define CAB_INV5_PHC	(13)

#define CAB_INV6_PHA	(21)
#define CAB_INV6_PHB	(23)
#define CAB_INV6_PHC	(22)

/*
 * Required ADC connections
 *
 *	CAB_INV4_PHA
 *	CAB_INV4_PHB
 *
 *	CAB_INV5_PHA
 *	CAB_INV5_PHB
 *	CAB_INV5_PHC
 *
 *	POSITION_SENS_X
 *	POSITION_SENS_Y
 *	POSITION_SENS_Z
*/

//Single phase inverter current sensor channels

//Three phase inverter current sensor channels
//Inv2
#define CAB_INV2_PHA_ADC (MB_IN1)
#define CAB_INV2_PHB_ADC (MB_IN3)
#define CAB_INV2_PHC_ADC (MB_IN5)
//Inv3
#define CAB_INV3_PHA_ADC (MB_IN2)
#define CAB_INV3_PHB_ADC (MB_IN4)
#define CAB_INV3_PHC_ADC (MB_IN6)

//Inv5
#define CAB_INV5_PHA_ADC (ANALOG_IN1)
#define CAB_INV5_PHB_ADC (ANALOG_IN2)
#define CAB_INV5_PHC_ADC (ANALOG_IN3)
//Inv6
#define CAB_INV6_PHA_ADC (ANALOG_IN5)
#define CAB_INV6_PHB_ADC (ANALOG_IN6)
#define CAB_INV6_PHC_ADC (ANALOG_IN7)



#define CAB_INV1_ADC (-1)
//#define CAB_INV2_ADC (-1)
//#define CAB_INV3_ADC (-1)
#define CAB_INV4_ADC (-1)


//ADC current sensor gains
//Calibrated by Nicholas Hemenway 01/24/2020:
//https://drive.google.com/drive/u/1/folders/1wPNnbSQbDKlm-QNHGPerimLWiv63sxtn
#define NO_USE (-1)
#define CAB_INV1_ADC_GAIN (-2.96787)
/*#define CAB_INV2_ADC_GAIN (-2.95262)
#define CAB_INV3_ADC_GAIN (-2.98213)*/
#define CAB_INV4_ADC_GAIN (-1.00516)

#define AMDS_CUR_GAIN (-9.69133)
#define CAB_INV2_PHA_ADC_GAIN (-9.69133)//(-9.69133*1.011) //MB1
#define CAB_INV2_PHB_ADC_GAIN (-9.69133)//(-9.69133*1.012) //MB3
#define CAB_INV2_PHC_ADC_GAIN (-9.69133)//(-9.69133*1.015) //MB5

#define CAB_INV3_PHA_ADC_GAIN (-9.69133)//(-9.69133*1.017) //MB2
#define CAB_INV3_PHB_ADC_GAIN (-9.69133)//(-9.69133*1.013) //MB4
#define CAB_INV3_PHC_ADC_GAIN (-9.69133)//(-9.69133*1.013) //MB6

#define CAB_INV5_PHA_ADC_GAIN (-1.508)
#define CAB_INV5_PHB_ADC_GAIN (-1.516)
#define CAB_INV5_PHC_ADC_GAIN (-1.503)

#define CAB_INV6_PHA_ADC_GAIN (-1.5)
#define CAB_INV6_PHB_ADC_GAIN (-1.5)
#define CAB_INV6_PHC_ADC_GAIN (-1.5)

//ADC current sensor offsets

#define CAB_INV1_ADC_OFFSET (0.0751899)
/*#define CAB_INV2_ADC_OFFSET (0.00586923)
#define CAB_INV3_ADC_OFFSET (0.10395)*/
#define CAB_INV4_ADC_OFFSET (0.00866636)

#define CAB_INV2_PHA_ADC_OFFSET (0.1023)//(9.69133*0.1023) 
#define CAB_INV2_PHB_ADC_OFFSET (0.1014)//(9.69133*0.1014)
#define CAB_INV2_PHC_ADC_OFFSET (0.09866)//(9.69133*0.09866)

#define CAB_INV3_PHA_ADC_OFFSET (0.05318)//(-9.69133*0.05318)
#define CAB_INV3_PHB_ADC_OFFSET (0.08723)//(9.69133*0.08723)
#define CAB_INV3_PHC_ADC_OFFSET (0.04513)//(9.69133*0.04513)

#define CAB_INV5_PHA_ADC_OFFSET (0.004828)
#define CAB_INV5_PHB_ADC_OFFSET (0.05004)
#define CAB_INV5_PHC_ADC_OFFSET (-0.05253)

#define CAB_INV6_PHA_ADC_OFFSET (-0.0635924)
#define CAB_INV6_PHB_ADC_OFFSET (0.153899)
#define CAB_INV6_PHC_ADC_OFFSET (0.0968906)

//Position sensor info
#define POSX_ADC (ANALOG_IN7)
#define POSY_ADC (ANALOG_IN4)
#define POSZ_ADC (ANALOG_IN8)

#define POSX_ADC_GAIN (7.6152839e-04)  // m/V
#define POSY_ADC_GAIN (8.0204556e-04)  // m/V
#define POSZ_ADC_GAIN (1.9927181e-04) // m/V

#define POSX_ADC_OFFSET (-1.9193977e-03) // m
#define POSY_ADC_OFFSET (-2.0003745e-03) // m
#define POSZ_ADC_OFFSET (-9.9529024e-04) // m

//THREE PHASE INVERTERS
static const HW_three_phase_t three_phase_lookup[CABINET_NUM_3PHASE] = {

    {.pwm = {.pwmIdxA = CAB_INV2_PHA, .pwmIdxB = CAB_INV2_PHB, .pwmIdxC = CAB_INV2_PHC},

     .sensor = {.enable = 0,
                .Ia = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .Ib = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .Ic = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE}},
                
     .mb_csensor = {.enable = 1,
                .mb_Ia = {.mbCh = CAB_INV2_PHA_ADC, .adcGain = CAB_INV2_PHA_ADC_GAIN, .adcOffset = CAB_INV2_PHA_ADC_OFFSET},
                .mb_Ib = {.mbCh = CAB_INV2_PHB_ADC, .adcGain = CAB_INV2_PHB_ADC_GAIN, .adcOffset = CAB_INV2_PHB_ADC_OFFSET},
                .mb_Ic = {.mbCh = CAB_INV2_PHC_ADC, .adcGain = CAB_INV2_PHC_ADC_GAIN, .adcOffset = CAB_INV2_PHC_ADC_OFFSET}}},

    {.pwm = {.pwmIdxA = CAB_INV3_PHA, .pwmIdxB = CAB_INV3_PHB, .pwmIdxC = CAB_INV3_PHC},

     .sensor = {.enable = 0,
                .Ia = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .Ib = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .Ic = {.adcCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE}},
                
    .mb_csensor = {.enable = 1,
                .mb_Ia = {.mbCh = CAB_INV3_PHA_ADC, .adcGain = CAB_INV3_PHA_ADC_GAIN, .adcOffset = CAB_INV3_PHA_ADC_OFFSET},
                .mb_Ib = {.mbCh = CAB_INV3_PHB_ADC, .adcGain = CAB_INV3_PHB_ADC_GAIN, .adcOffset = CAB_INV3_PHB_ADC_OFFSET},
                .mb_Ic = {.mbCh = CAB_INV3_PHC_ADC, .adcGain = CAB_INV3_PHC_ADC_GAIN, .adcOffset = CAB_INV3_PHC_ADC_OFFSET}}},

    {.pwm = {.pwmIdxA = CAB_INV5_PHA, .pwmIdxB = CAB_INV5_PHB, .pwmIdxC = CAB_INV5_PHC},

     .sensor = {.enable = 1,
                .Ia = {.adcCh = CAB_INV5_PHA_ADC, .adcGain = CAB_INV5_PHA_ADC_GAIN, .adcOffset = CAB_INV5_PHA_ADC_OFFSET},
                .Ib = {.adcCh = CAB_INV5_PHB_ADC, .adcGain = CAB_INV5_PHB_ADC_GAIN, .adcOffset = CAB_INV5_PHB_ADC_OFFSET},
                .Ic = {.adcCh = CAB_INV5_PHC_ADC, .adcGain = CAB_INV5_PHC_ADC_GAIN, .adcOffset = CAB_INV5_PHC_ADC_OFFSET}},
     
     .mb_csensor = {.enable = 0,
                .mb_Ia = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .mb_Ib = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .mb_Ic = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE}}},
    {.pwm = {.pwmIdxA = CAB_INV6_PHA, .pwmIdxB = CAB_INV6_PHB, .pwmIdxC = CAB_INV6_PHC},

     .sensor = {.enable = 1,
                .Ia = {.adcCh = CAB_INV6_PHA_ADC, .adcGain = CAB_INV6_PHA_ADC_GAIN, .adcOffset = CAB_INV6_PHA_ADC_OFFSET},
                .Ib = {.adcCh = CAB_INV6_PHB_ADC, .adcGain = CAB_INV6_PHB_ADC_GAIN, .adcOffset = CAB_INV6_PHB_ADC_OFFSET},
                .Ic = {.adcCh = CAB_INV6_PHC_ADC, .adcGain = CAB_INV6_PHC_ADC_GAIN, .adcOffset = CAB_INV6_PHC_ADC_OFFSET}},
                
    .mb_csensor = {.enable = 0,
                .mb_Ia = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .mb_Ib = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE},
                .mb_Ic = {.mbCh = NO_USE, .adcGain = NO_USE, .adcOffset = NO_USE}}}

};

//SINGLE PHASE INVERTERS
static const HW_single_phase_t single_phase_lookup[CABINET_NUM_1PHASE] = {
    {.pwm = {.pwmIdxA = CAB_INV1_PHA, .pwmIdxB = CAB_INV1_PHB},
     .sensor = {.Iz = {.adcCh = CAB_INV1_ADC, .adcGain = CAB_INV1_ADC_GAIN, .adcOffset = CAB_INV1_ADC_OFFSET}}},

    /*{.pwm = {.pwmIdxA = CAB_INV2_PHA, .pwmIdxB = CAB_INV2_PHB},
     .sensor = {.Iz = {.adcCh = CAB_INV2_ADC, .adcGain = CAB_INV2_ADC_GAIN, .adcOffset = CAB_INV2_ADC_OFFSET}}},

    {.pwm = {.pwmIdxA = CAB_INV3_PHA, .pwmIdxB = CAB_INV3_PHB},
     .sensor = {.Iz = {.adcCh = CAB_INV3_ADC, .adcGain = CAB_INV3_ADC_GAIN, .adcOffset = CAB_INV3_ADC_OFFSET}}},*/

    {.pwm = {.pwmIdxA = CAB_INV4_PHA, .pwmIdxB = CAB_INV4_PHB},
     .sensor = {.Iz = {.adcCh = CAB_INV4_ADC, .adcGain = CAB_INV4_ADC_GAIN, .adcOffset = CAB_INV4_ADC_OFFSET}}}

};

//POSITION SENSORS
const analog_sensor_t   pos_sensors[NUM_POS_SENSORS] = {

    {.adcCh = POSX_ADC, .adcGain = POSX_ADC_GAIN, .adcOffset = POSX_ADC_OFFSET},
    {.adcCh = POSY_ADC, .adcGain = POSY_ADC_GAIN, .adcOffset = POSY_ADC_OFFSET},
    {.adcCh = POSZ_ADC, .adcGain = POSZ_ADC_GAIN, .adcOffset = POSZ_ADC_OFFSET}

};


//create inverters and function to return them out of this file

InverterSinglePhase_t inverters_single[CABINET_NUM_1PHASE];
InverterThreePhase_t inverters_three[CABINET_NUM_3PHASE];

InverterSinglePhase_t *get_single_phase_inverter(int inverter){

    InverterSinglePhase_t *inv = &(inverters_single[inverter]); //get address
    return inv;
}

InverterThreePhase_t *get_three_phase_inverter(int inverter){

    InverterThreePhase_t *inv = &(inverters_three[inverter]); //get address
    return inv;
}

void default_inverter_setup(double Vdc){

    //we must assign hardware to each inverter and set Vdc for each inverter
    for (int i = 0; i < CABINET_NUM_1PHASE; i++) {
        InverterSinglePhase_t *inv = get_single_phase_inverter(i);
        inv->Vdc = Vdc;
        inv->HW = &(single_phase_lookup[i]);
    }

    for (int i = 0; i < CABINET_NUM_3PHASE; i++) {
        InverterThreePhase_t *inv = get_three_phase_inverter(i);
        inv->Vdc = Vdc;
        inv->HW = &(three_phase_lookup[i]);
    }
}


int find_invIdex_invID (int inverter){
    int dataId;
    switch(inverter) {
        case 1  :
            dataId = 0;
            break; 
        case 2  :
            dataId = 0;
            break;
        case 3  :
            dataId = 1;
            break; 
        case 4  :
            dataId = 1;
            break;
        case 5  :
            dataId = 2;
            break; 
        case 6  :
            dataId = 3;
            break;
        default : 
            dataId = -1;
            
    }
    return dataId;

}
