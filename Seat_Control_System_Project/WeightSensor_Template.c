/**
 *
 * \file WeightSensor_Template.c
 * \brief Rte Component Template for AUTOSAR SWC: WeightSensor
 *
 * \author Sprints AUTOSAR Authoring Tool (SAAT) v1.0.2
 * Generated on 11/4/2023 06:02 PM
 *
 * For any inquiries: hassan.m.farahat@gmail.com
 *
 */

#include "Rte_WeightSensor.h"


/**
 *
 * Runnable WeightSensor_GetWeight
 *
 * Triggered By:
 *  - OperationInvokedEventImpl.OIE_ppWeightSensor_GetWeight
 *
 */

void WeightSensor_GetWeight (SensorWeightType* weight)
{
	Std_ReturnType status;
	IoWeightSensorReadingType weight;

	/* Server Call Points */
	status = Rte_Call_rpIOGetWeight_IOGet(&weight);
	
}

