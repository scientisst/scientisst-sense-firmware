#ifndef _CONFIG_H
#define _CONFIG_H

#define NO_EXT_ADC 0  // External ADC disabled
#define ADC_ADS 1     // ADS
#define ADC_MCP 2
#define _ADC_EXT_ NO_EXT_ADC

#define HW_VERSION_LEGACY 0
#define HW_VERSION_NANO 1
#define HW_VERSION HW_VERSION_LEGACY

#define _TIMESTAMP_ 0  // 0: no timestamp, 1: timestamp

#if _TIMESTAMP_ == 1 && _ADC_EXT_ != NO_EXT_ADC
#error timestamp requires that no external adc is enabled
#endif

#endif