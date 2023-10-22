# Binary File Format

The [SDCarFileConversion.py](../SDCardFileConverter/sdcardfileconversion.py) is a Python script designed to convert
binary data from the the SD Card into a CSV format akin to the one
produced by the APIs. Below is a description of the binary data format used.

## Binary Data Format : Revision 1

Before the data, a "binary header" is written so the conversion scripts can process the rest of the file: It needs to
know the acquisiton configurations, adc characeristics to perform conversions, sample rate, active channels, etc.

Below is the header format, described as a byte array:

- [0]     : File revision
- [1]     : Number of Bytes of the sequence number
- [2]     : Is seq num fused with Digital channels? (4 bits for DIO, 12 bits for sequence number)
- [3]     :  Number of digital inputs
- [4]     : Number of Bytes for each digital input
- [5]     : Number of digital outputs
- [6]     : Number of Bytes for each digital output
- [7]     : Number of internal ADC channels
- [8]     : Number of Bytes for each internal ADC channel
- [9]     : Number of external ADC channels
- [10]    : Number of Bytes for each external ADC channel
- [11]    : Internal ADC data type ID (Internal ADC or IMU)
- [12]    : External ADC data type ID
- [13]    : Is timestamp present
- [14]    : Number of Bytes for timestamp
- [15-16] : Sample rate
- [17-20] : ADC1 coeff_a
- [21-24] : ADC1 coeff_b
