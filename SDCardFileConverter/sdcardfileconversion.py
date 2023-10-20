import os

# Constants used for calculations related to ADC values.
LIN_COEFF_A_SCALE = 65536
LIN_COEFF_A_ROUND = LIN_COEFF_A_SCALE / 2
VOLT_DIVIDER_FACTOR = 3.399


def revision1(input_file, output_file):
    # Parse the header bytes

    nSeq_bytes = int.from_bytes(input_file.read(1), byteorder="little", signed=False)  # # of Bytes of SeqNum
    seqNumFusedWithDIO = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # Is seq num fused with DIO? (4 bits for DIO, 12 bits for seq num)
    num_digital_input = int.from_bytes(input_file.read(1), byteorder="little", signed=False)  # # of digital inputs
    digital_input_bytes = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of Bytes for each digital input
    num_digital_output = int.from_bytes(input_file.read(1), byteorder="little", signed=False)  # # of digital outputs
    digital_output_bytes = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of Bytes for each digital output
    num_internal_channels = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of internal ADC channels
    internal_channels_bytes = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of Bytes for each internal ADC channel
    num_external_channels = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of external ADC channels
    external_channels_bytes = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # # of Bytes for each external ADC channel
    internal_channel_data_id = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # Internal ADC data type ID
    external_channel_data_id = int.from_bytes(
        input_file.read(1), byteorder="little", signed=False
    )  # External ADC data type ID
    timestamp_active = int.from_bytes(input_file.read(1), byteorder="little", signed=False)  # Timestamp enable?
    timestamp_bytes = int.from_bytes(input_file.read(1), byteorder="little", signed=False)  # # of Bytes for timestamp
    frequency = int.from_bytes(input_file.read(2), byteorder="little", signed=False)  # Sample rate
    coeff_a = int.from_bytes(input_file.read(4), byteorder="little", signed=False)  # ADC1 coeff_a
    coeff_b = int.from_bytes(input_file.read(4), byteorder="little", signed=False)  # ADC1 coeff_b

    print("Number of Bytes of SeqNum:", nSeq_bytes)
    print("Is seq num fused with DIO? (4 bits for DIO, 12 bits for seq num):", seqNumFusedWithDIO)
    print("Number of digital inputs:", num_digital_input)
    print("Number of Bytes for each digital input:", digital_input_bytes)
    print("Number of digital outputs:", num_digital_output)
    print("Number of Bytes for each digital output:", digital_output_bytes)
    print("Number of internal ADC channels:", num_internal_channels)
    print("Number of Bytes for each internal ADC channel:", internal_channels_bytes)
    print("Number of external ADC channels:", num_external_channels)
    print("Number of Bytes for each external ADC channel:", external_channels_bytes)
    print("Internal ADC data type ID:", internal_channel_data_id)
    print("External ADC data type ID:", external_channel_data_id)
    print("Timestamp enable?", timestamp_active)
    print("Number of Bytes for timestamp:", timestamp_bytes)
    print("Sample rate:", frequency)
    print("ADC1 coeff_a:", coeff_a)
    print("ADC1 coeff_b:", coeff_b)

    # Write the header to the CSV file
    if timestamp_active and num_external_channels == 2:
        header_string = (
            "{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [6, 8, 10, 12, 14, 16, 18, 20], "
            "'Channels indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], "
            "'Channels labels': ['AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv', 'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], "
            "'Device': 'NULL', 'Firmware version': 'NULL', "
            "'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', 'AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv', 'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv', 'Timestamp'], "
            "'ISO 8601': 'NULL', "
            "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12, 24, 24, 64], "
            "'Sampling rate (Hz)': " + str(frequency) + ", 'Timestamp(us)': 0.0}\n"
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_mv\tAX7_raw\tAX7_mv\tAX8_raw\tAX8_mv\tTimestamp\n"
        )
    elif timestamp_active and num_external_channels == 0:
        header_string = (
            "{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [6, 8, 10, 12, 14, 16, 18, 20], "
            "'Channels indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], "
            "'Channels labels': ['AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv'], "
            "'Device': 'NULL', 'Firmware version': 'NULL', "
            "'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', 'AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv', 'Timestamp'], "
            "'ISO 8601': 'NULL', "
            "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12, 64], "
            "'Sampling rate (Hz)': " + str(frequency) + ", 'Timestamp(us)': 0.0}\n"
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_mv\tTimestamp\n"
        )
    else:
        print("Error: Wrong or corrupted file.")
        return

    output_file.write(header_string)

    while True:
        # Initialize a list to store the row's values
        row_values = []

        if seqNumFusedWithDIO == 1:
            # Read sequence number
            seq_number = int.from_bytes(input_file.read(nSeq_bytes), byteorder="little", signed=False)

            # Parse digital inputs
            digital_io = seq_number & 0x000F
            seq_number = seq_number >> 4
            row_values = [seq_number & 0x0FFF]
            for _ in range(num_digital_input + num_digital_output):
                digital_input_value = digital_io & 0x0001
                digital_io = digital_io >> 1
                row_values.append(digital_input_value)
        else:
            # Read sequence number
            seq_number = int.from_bytes(input_file.read(nSeq_bytes), byteorder="little", signed=False)
            row_values.append(seq_number)

            # Read digital inputs
            for _ in range(num_digital_input):
                digital_input_value = int.from_bytes(
                    input_file.read(digital_input_bytes), byteorder="little", signed=False
                )
                row_values.append(digital_input_value)

            # Read digital outputs
            for _ in range(num_digital_output):
                digital_output_value = int.from_bytes(
                    input_file.read(digital_output_bytes), byteorder="little", signed=False
                )
                row_values.append(digital_output_value)

        # Read internal channels
        for _ in range(num_internal_channels):
            internal_channel_value = int.from_bytes(
                input_file.read(internal_channels_bytes),
                byteorder="little",
                signed=False,
            )
            row_values.append(internal_channel_value)
            row_values.append(
                int(
                    (int(((coeff_a * internal_channel_value) + LIN_COEFF_A_ROUND) / LIN_COEFF_A_SCALE) + coeff_b)
                    * VOLT_DIVIDER_FACTOR
                )
            )

        # Read external channels
        for _ in range(num_external_channels):
            external_channel_value = int.from_bytes(
                input_file.read(external_channels_bytes),
                byteorder="little",
                signed=False,
            )
            row_values.append(external_channel_value)
            row_values.append(round(((external_channel_value * (3.3 * 2)) / (pow(2, 24) - 1)) * 1000, 3))

        # Read timestamp if active
        if timestamp_active:
            timestamp_value = int.from_bytes(input_file.read(timestamp_bytes), byteorder="little", signed=False)
            row_values.append(timestamp_value)

        # Write the row to the CSV file
        for value in range(0, len(row_values) - 1):
            output_file.write(str(row_values[value]) + "\t")
        output_file.write(str(row_values[len(row_values) - 1]) + "\n")

        # Check for EOF
        if not input_file.read(1):
            break
        input_file.seek(input_file.tell() - 1)


def main(input_filename, output_filename):
    # Check if input file exists and is a .bin file
    if not os.path.exists(input_filename):
        print("Error: Input file does not exist.")
        return
    if not input_filename.endswith(".bin"):
        print("Error: Input file is not a .bin file.")
        return

    # Check if output file has a .csv extension
    if not output_filename.endswith(".csv"):
        print("Error: Output file should have a .csv extension.")
        return

    # Check if output file exists
    if os.path.exists(output_filename):
        overwrite = input(f"{output_filename} already exists. Do you want to overwrite it? (yes/no) ").lower()
        if overwrite != "yes":
            output_filename = input("Please provide a new name for the output file: ")
            if not output_filename.endswith(".csv"):
                print("Error: Output file should have a .csv extension.")
                return

    # Process input file and write to output file
    with open(input_filename, "rb") as input_file, open(output_filename, "w") as output_file:
        first_byte = input_file.read(1)
        if first_byte == b"\x01":
            revision1(input_file, output_file)
        else:
            print("Error: Wrong or corrupted file.")


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print("Usage: python3 sdCardFileConversion.py [input file name] [output file name] [arguments]")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
