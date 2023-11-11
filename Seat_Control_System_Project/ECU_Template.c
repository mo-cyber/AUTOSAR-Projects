/**
 *
 * \file ECU_Template.c
 * \brief Rte Component Template for AUTOSAR SWC: ECU
 *
 * \author Sprints AUTOSAR Authoring Tool (SAAT) v1.0.2
 * Generated on 11/4/2023 06:01 PM
 *
 * For any inquiries: hassan.m.farahat@gmail.com
 *
 */

#include "Rte_ECU.h"


/**
 *
 * Runnable ECU_MainFunction
 *
 * Triggered By:
 *  - TimingEventImpl.TE_ECU_MainFunction_100ms
 *
 */

void ECU_MainFunction (void)
{
	Std_ReturnType status;
	uint8 Height;
	uint8 Incline;
	uint8 Slide;

	/* Data Send Points */
	status = Rte_Write_ppSeatCtrlData_Height(Height);
	status = Rte_Write_ppSeatCtrlData_Incline(Incline);
	status = Rte_Write_ppSeatCtrlData_Slide(Slide);
	
}

