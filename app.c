#include "app.h"



extern Energy_Meter meter;
extern __IO uint32_t LocalTime;
extern int printDelay;
extern int EnergyCalcDelay;

float V_sensor_reading;
float I_sensor_reading;

float V_calibration = 1.27;
float I_calibration = 0.0095;





void App()
{
	ADC_init();
	while(1) {

		if(meter.status == DATA_RECEIVED){
			V_sensor_reading = Read_voltage();
			I_sensor_reading = Read_current();
			Calc_RMS_values(V_sensor_reading,I_sensor_reading);
			meter.power_factor = Measure_power_factor(V_sensor_reading,I_sensor_reading);
			Calc_power_energy(meter.vrms,meter.irms,meter.power_factor);
			UART_Transmit();
		}
	}

}






void Calc_power_energy(float vrms, float irms, float pf)
{
	if((LocalTime - EnergyCalcDelay) > 1000)
	{

		meter.power = vrms*irms*pf;
		Read_FRAM(meter.Energy);
		meter.Energy+=(meter.power/3600);
		Write_FRAM(meter.Energy);



		EnergyCalcDelay = LocalTime;
	}
}


void UART_Transmit()
{

	if((LocalTime - printDelay) > 3000)
	{
		printDelay = LocalTime;
		meter.power_factor = (meter.power == 0)? 0 : meter.power_factor;


		printf("%06.2f",meter.vrms);
		printf("%06.2f",meter.irms);
		printf("%06.2f",meter.power_factor);
		printf("%06.2f",meter.power);
		printf("%06.2f\r\n",meter.Energy);





	}
}
