#include "Energy_meter.h"




char Time[10];
char Date[10];

struct tm t;
time_t t_of_day;

RTC_TimeTypeDef gTime = {0};
RTC_DateTypeDef gDate = {0};

uint32_t adc_val = 0;
Energy_Meter meter;

int v_index = 0;
int i_index = 0;

int n_sample = 0;

float Phase_diff = 0;
float volt_sqr_mean = 0;
float volt_min = 0;
float curr_sqr_mean = 0;
float curr_min = 0;
float v_rms = 0;
float v_rms_avg = 0;
float i_rms = 0;
float i_rms_avg = 0;
int cycle_count = 0;

float temp_vrms = 0;

int printDelay = 0;
int EnergyCalcDelay = 0;

extern __IO uint32_t LocalTime;

extern float V_calibration;
extern float I_calibration;





void ADC_init()
{
	meter.status = V_I_COMPUTED;

	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		Error_Handler();

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
}



float Read_voltage()
{
	float volt_offset = 0;
	Moving_Average_Filter(meter.v_inst, meter.v_inst_filt, NUM_OF_SAMPLES, 10);

	for(int i=0; i<NUM_OF_SAMPLES; i++)
	{
		volt_offset += meter.v_inst_filt[i];
	}

	volt_offset /= NUM_OF_SAMPLES;

	return volt_offset;
}


float Read_current()
{
	float curr_offset = 0;
	Moving_Average_Filter(meter.i_inst, meter.i_inst_filt, NUM_OF_SAMPLES, 10 );
	for(int i=0; i<NUM_OF_SAMPLES; i++)
	{

		curr_offset += meter.i_inst_filt[i];
	}


	curr_offset /= NUM_OF_SAMPLES;

	return curr_offset;
}


float Measure_power_factor(float V_reading, float I_reading)
{
	volt_min = V_reading;
	curr_min = I_reading;

	for(int i=0; i<NUM_OF_SAMPLES/2; i++)
	{
		if(volt_min > meter.v_inst_filt[i])
		{
			if((meter.v_inst_filt[i] < (V_reading+100)) && (meter.v_inst_filt[i] > (V_reading-100)) )
			{
				volt_min = meter.v_inst_filt[i];
				v_index = i;
			}
		}
		if(curr_min > meter.i_inst_filt[i])
		{
			if((meter.i_inst_filt[i] < (I_reading+100)) && (meter.i_inst_filt[i] > (I_reading-100)) )
			{
				curr_min = meter.i_inst_filt[i];
				i_index = i;
			}

		}
	}
	Phase_diff = (float)(v_index - i_index)*0.076*PI/10;

	return  cos(Phase_diff);
}


void Calc_RMS_values(float V_reading, float I_reading)
{


	volt_sqr_mean = 0;
	for(int i=0; i<NUM_OF_SAMPLES; i++)
	{
		meter.v_inst_filt[i] -= V_reading;
		volt_sqr_mean += meter.v_inst_filt[i] * meter.v_inst_filt[i];
	}

	volt_sqr_mean /= NUM_OF_SAMPLES;
	v_rms += sqrt(volt_sqr_mean);


	curr_sqr_mean = 0;
	for(int i=0; i<NUM_OF_SAMPLES; i++)
	{
		meter.i_inst_filt[i] -= I_reading;
		curr_sqr_mean += meter.i_inst_filt[i] * meter.i_inst_filt[i];
	}

	curr_sqr_mean /= NUM_OF_SAMPLES;
	i_rms += sqrt(curr_sqr_mean);


	meter.status = V_I_COMPUTED;
	if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		Error_Handler();

	cycle_count++;
	if(cycle_count == 20)
	{
		cycle_count =0;
		v_rms_avg = v_rms/20;
		i_rms_avg = i_rms/20;

		temp_vrms = V_calibration*v_rms_avg;
		meter.vrms = ( (temp_vrms > 200.0f) && (temp_vrms < 270.0f))?temp_vrms:meter.vrms;
		meter.vrms = (temp_vrms < 50)? 0: meter.vrms;
		meter.irms = I_calibration*i_rms_avg;
		meter.irms = (meter.irms > 0.1f)?meter.irms:0;
		v_rms = 0;
		i_rms = 0;
	}

}


void Moving_Average_Filter(float * inputSignal, float * outputSignal, uint16_t arrSize, uint16_t nPoint)
{
	uint16_t index = 0;

	for(uint16_t i = 0; i < arrSize; i++)
	{
		for(uint16_t j=0; j < nPoint; j++)
		{
			if((i-j) < 0)
			{
				index = arrSize + (i-j);
			}
			else
			{
				index = i-j;
			}
			outputSignal[i] += inputSignal[index];
		}

		outputSignal[i] /= nPoint;
	}
}

void Read_FRAM(float data)
{
	if( HAL_I2C_Mem_Read(&hi2c1, FRAM_ADDRESS , ENERGY_ADDRESS, I2C_MEMADD_SIZE_16BIT,(uint8_t *) &data, 4, 100) != HAL_OK)
	{
		printf("Read Failure/r/n");
	}
}


void Write_FRAM(float data)
{
	if(HAL_I2C_Mem_Write(&hi2c1, FRAM_ADDRESS , ENERGY_ADDRESS, I2C_MEMADD_SIZE_16BIT,(uint8_t *)  &data, 4, 100) != HAL_OK)
	{
		printf("Write Failure/r/n");
	}

}

