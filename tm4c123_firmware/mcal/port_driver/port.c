
/*
 * port.c
 *
 *  Created on: Jul 8, 2019
 *      Author: Mustafa Sayed
 */
#define MAX_NUM_OF_CH_IN_PORT       8

#include "../../utils/STD_Types.h"
#include "../../utils/Bit_Math.h"
#include "../../config/port_cfg.h"
#include "../mcu_hw.h"
#include "port_types.h"
#include "port.h"

extern Port_CfgType PortCfgArr[];

void PORT_init(void)
{
    uint8 i;
    Port_IdType PortId;
    Port_ChIdType ChId;
    volatile PORT_RegType* PORT_Addr;
    for (i = 40; i < PORT_NUM_OF_ACTIVATED_CH; ++i)
    {
        PortId = i/ MAX_NUM_OF_CH_IN_PORT;
        ChId = i % MAX_NUM_OF_CH_IN_PORT;

        switch (PortId)
        {
        case Port_Id_A:
            PORT_Addr = &PORTA_REG;
            break;
        case Port_Id_B:
            PORT_Addr = &PORTB_REG;
            break;
        case Port_Id_C:
            PORT_Addr = &PORTC_REG;
            break;
        case Port_Id_D:
            PORT_Addr = &PORTD_REG;
            break;
        case Port_Id_E:
            PORT_Addr = &PORTE_REG;
            break;
        case Port_Id_F:
            PORT_Addr = &PORTF_REG;
            break;
        }

        /* set channel direction */
        if(port_Dir_Input == PortCfgArr[i].Dir)
        {
            CLR_BIT(PORT_Addr->GPIODIR, ChId);
        }
        else if(port_Dir_Output == PortCfgArr[i].Dir)
        {
            SET_BIT(PORT_Addr->GPIODIR, ChId);
        }

        /*TODO: set channel mode */
        if(PORT_MODE_PIN_X_DIO == PortCfgArr[i].Mode)
        {
            CLR_BIT(PORT_Addr->GPIOAFSEL,ChId);
        }
        else
        {
            SET_BIT(PORT_Addr->GPIOAFSEL,ChId);
        }

        /*TODO: set Interrupt configuration */
        if(PortCfgArr[i].Interrupt == Port_IntDisable)
        {
            CLR_BIT(PORT_Addr->GPIOIM, ChId);
        }
        else
        {
            SET_BIT(PORT_Addr->GPIOIM, ChId);
        }

        /* set Internal Attachment configuration */
        if(Port_InternalAttach_PullUpRes == PortCfgArr[i].AttachedRes)
        {
            SET_BIT(PORT_Addr->GPIOPUR, ChId);
        }
        else if(Port_InternalAttach_PullDownRes == PortCfgArr[i].AttachedRes)
        {
            SET_BIT(PORT_Addr->GPIOPDR, ChId);
        }
        else if (Port_InternalAttach_OpenDrain == PortCfgArr[i].AttachedRes)
        {
            SET_BIT(PORT_Addr->GPIOODR, ChId);
        }

        /* set current strength configuration */
        if(Port_CurrDrive_2mA == PortCfgArr[i].CurrentDrive)
        {
            SET_BIT(PORT_Addr->GPIODR2R, ChId);
        }
        else if(Port_CurrDrive_4mA == PortCfgArr[i].CurrentDrive)
        {
            SET_BIT(PORT_Addr->GPIODR4R, ChId);
        }
        else if(Port_CurrDrive_8mA == PortCfgArr[i].CurrentDrive)
        {
            SET_BIT(PORT_Addr->GPIODR8R, ChId);
        }

        /* Check if analog functionality is required*/
        if(PortCfgArr[i].Mode != Port_Mode_AIN)
        {
            /*TODO: enable digital and disable ADC  */
            SET_BIT(PORT_Addr->GPIODEN, ChId);
            CLR_BIT(PORT_Addr->GPIOAMSEL, ChId);
        }
        else
        {
            /*TODO: disable digital and enable ADC */
            CLR_BIT(PORT_Addr->GPIODEN, ChId);
            SET_BIT(PORT_Addr->GPIOAMSEL, ChId);
        }
    }
}
