#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <driver/adc.h>


void adcConfig() {
	// set the resolution to 10-bit
	adc1_config_width(ADC_WIDTH_BIT_10);
	// configure each ADC1 channel, including attenuation
	for (adc1_channel_t channel_no = ADC1_CHANNEL_0; channel_no <= ADC1_CHANNEL_7; channel_no++) {
		adc1_config_channel_atten(channel_no, ADC_ATTEN_DB_0);
	}

	printf(">> OK: ADC configured\n");
}


int adcRead(uint8_t channel) {
	return adc1_get_raw(channel);
}
