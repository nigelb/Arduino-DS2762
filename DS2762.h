/*
    Arduino DS2762 Library
    Copyright (C) 2013  nigelb, Scott Mills

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef DS2762_H_
#define DS2762_H_


/*
	DS2762 - High-Precision Li+ Battery Monitor With Alerts

					   __   ┌-------------┐
					   CC --+ 1°       16 +-- Vin
							|             |
					  PLS --+ 2        15 +-- Vdd
					   __   |             |
					   DC --+ 3        14 +-- PIO
							|             |
					  SNS --+ 4        13 +-- Vss
							|             |
					  SNS --+ 5        12 +-- Vss
							|             |
					  SNS --+ 6        11 +-- Vss
							|             |   __
					   DQ --+ 7        10 +-- PS
							|             |
					  1S2 --+ 8        9  +-- IS1
							└-------------┘


					  1         CC - Charge Protection Control Output
					  2        PLS - Battery Pack Positive Terminal Input.
					  3         DC - Discharge Protection Control Output.
					  4,5,6    SNS - Sense Resistor Connection
					  7         DQ - Data Input/Out, 1-Wire data line
					  8        IS2 - Current-Sense Input
					  9        IS1 - Current-Sense Input
					  10        PS - Power Switch Sense Input
					  11,12,13 Vss - Device Ground
					  14       PIO - Programmable I/O Pin
					  15       Vdd - Power-Supply Input
					  16       Vin - Voltage Sense Input

 */
#include <OneWire.h>


/* Function Commands */
#define DS2762_READ_DATA               0x69 /* [0x69, XX] Read data starting from memory address XX, max 256 bytes. */
#define DS2762_WRITE_DATA              0x6C /* [0x6c, XX] Write data to memory starting at XX, max 256 bytes*/
#define DS2762_COPY_DATA               0x48 /* [0x48, XX] Copies the shadow RAM to EEPROM, for the 16 byte block containing XX */
#define DS2762_RECALL_DATA             0xB8 /* [0xB8, XX] Copies the 16 byte EEPROM block containing XX to shadow RAM */
#define DS2762_LOCK	                   0x6A /* [0x6a, XX] WARNING - this command permanently write-protects the 16 byte block of EEPROM containing XX*/
#define DS2762_SWAP                    0xAA /* [0xAA]     See datasheet */

/* Memory Map P. 14-17 */

//Protection Register
#define DS2762_PROTECTION_REG          0x00  /* Memory address of the Protection Register */

//Protection Register Masks
#define DS2762_PR_OV_FLAG           (1 << 7) // Overvoltage flag
#define DS2762_PR_UV_FLAG           (1 << 6) // Undervoltage flag
#define DS2762_PR_COC_FLAG          (1 << 5) // Charge Overcurrent flag
#define DS2762_PR_DOC_FLAG          (1 << 4) // Discharge Overcurrent flag
#define DS2762_PR_CCPIN_FLAG        (1 << 3) // CC Pine Mirror flag
#define DS2762_PR_DCPIN_FLAG        (1 << 2) // DC Pin Mirror flag
#define DS2762_PR_CE_FLAG           (1 << 1) // Charge Enable flag
#define DS2762_PR_DE_FLAG                 1  // Discharge Enable flag

//Status Register
#define DS2762_STATUS_REG              0x01  /* Memory address of the Status Register*/

//Status Register Masks
#define DS2762_STATUS_IE            (1 << 2) /* Interrupt Enable - Enables the PIO pin when the thresholds on the accumulated current and temperature interupts are met or exceeded - enabled (1) disabled (0) Factory default (0) */
#define DS2762_STATUS_SWEN          (1 << 3) /* SWAP Command Enable - Enables the ability to control the flow of power into or out of the battery pack using control pins CC (Charge Protection Control Output using an external P-channel high-side charge protection FET) and DC (Discharge Protection Control Output using an external P-channel high-side charge protection FET - enabled (1) disabled (0) factory default (0) */
#define DS2762_STATUS_RNAOP         (1 << 4) /* Read Net Address Opcode -  */
#define DS2762_STATUS_PMOD          (1 << 5) /* Sleep Mode Enable - enabled (1) disabled (0) factory default (0) */

//EEPROM Register
#define DS2762_EEPROM_REG              0x07  /* Memory address of the EEPROM Register */
#define DS2762_EEPROM_EEC           (1 << 7) /* EEPROM Copy flag */
#define DS2762_EEPROM_LOCK          (1 << 6) /* EEPROM Lock Enable flag */
#define DS2762_EEPROM_BL1           (1 << 1) /* EEPROM Block 1 Lock flag */
#define DS2762_EEPROM_BL0                 1  /* EEPROM Block 0 Lock flag */

//Special Feature Register
#define DS2762_SPECIAL_FEATURE_REG     0x08  /* Memory address of the Special Feature Register */

//Special Feature Masks
#define DS2762_SF_PS_FLAG           (1 << 7) /* PS Pin Latch */
#define DS2762_SF_PIO_FLAG          (1 << 6) /* PIO Pin Sense and Control */
#define DS2762_SF_MSTR_FLAG         (1 << 5) /* SWAP Master Status flag */

/* 0x09-0x0B Reserved */

#define DS2762_VOLTAGE_MSB             0x0c /* Voltage Register MSB */
#define DS2762_VOLTAGE_LSB             0x0d /* Voltage Register LSB */
#define DS2762_CURRENT_MSB             0x0e /* Current Register MSB */
#define DS2762_CURRENT_LSB             0x0f /* Current Register LSB */
#define DS2762_CURRENT_ACCUM_MSB       0x10 /* Accumulated Current Register MSB  */
#define DS2762_CURRENT_ACCUM_LSB       0x11 /* Accumulated Current Register LSB  */

/* 0x12-0x17 Reserved */

#define DS2762_TEMP_MSB                0x18 /* Temperature Register MSB */
#define DS2762_TEMP_LSB                0x19 /* Temperature Register LSB  */

/* 0x1A-0x1F Reserved */

#define DS2762_EEPROM_BLOCK0           0x20/*to 0x2F*/
#define DS2762_EEPROM_BLOCK1           0x30/*to 0x3F*/

/* 0x40-0x7F Reserved */

//Shadow RAM
#define CURRENT_ACCUM_INT_HIGH_MSB     0x80 /* Optional Accumulated Current Interrupt High Threshold MSB */
#define CURRENT_ACCUM_INT_HIGH_LSB     0x81 /* Optional Accumulated Current Interrupt High Threshold LSB */
#define CURRENT_ACCUM_INT_LOW_MSB      0x82 /* Optional Accumulated Current Interrupt Low Threshold MSB */
#define CURRENT_ACCUM_INT_LOW_LSB      0x83 /* Optional Accumulated Current Interrupt Low Threshold LSB */
#define TEMP_INT_HIGH                  0x84 /* Optional Temperature Interrupt High Threshold */
#define TEMP_INT_LOW                   0x85 /* Optional Temperature Interrupt Low Threshold */
#define SRAM                           0x86 /*to 0x8F*/

/* 0x90-0xFF Reserved */

//EEPROM Addresses
#define DS2762_PROTECTION_WRITE_RE     0x30 /* Protection Register EEPROM address */
#define DS2762_STATUS_WRITE_REG        0x31 /* Status Register EEPROM address */
#define DS2762_CURRENT_OFFSET_BIAS_REG 0x33 /* Current Offset Register EEPROM address */


enum DS2762_EEPROM {
	BLOCK0 = DS2762_EEPROM_BLOCK0,
	BLOCK1 = DS2762_EEPROM_BLOCK1
};


class DS2762
{
private:
	OneWire* bus;
	uint8_t* address;
	uint8_t* memory;
	void _read_device(uint8_t* buf, uint8_t start, uint8_t count);
	void _write_device(uint8_t* buf, uint8_t start, uint8_t count);
	boolean _has_buffer();
	uint8_t _read_register_bit(uint8_t register_address, uint8_t pin);
	void _set_register_bit(uint8_t read_register_address, uint8_t write_register_address, uint8_t bit, DS2762_EEPROM block, boolean enabled);

public:
	DS2762(OneWire*, uint8_t* address, boolean read_all = false);
	~DS2762();

    /*
     * readDevice will read the entire contents of the DS2762 memory block into
     * a local array, after this all read operations will be carried out from the
     * local copy.
     */
	void readDevice();

	/*
	 * readTempC retrieves the current temperature from the DS2762's temperature
	 * sensor.
	 */
	double readTempC();

	/*
	 *
	 */
	double readADC();

	/*
	 * copyEEPROMBlock issues the DS2762_COPY_DATA command to the
	 * causing the DS2762 to copy data from the shadow RAM to the EEPROM.
	 */
	void copyEEPROMBlock(DS2762_EEPROM block);

	/*
	 * copyEEPROMBlock issues the DS2762_RECALL_DATA command to the
	 * causing the DS2762 to copy data from the EEPROM to the shadow RAM.
	 */
	void recallEEPROMBlock(DS2762_EEPROM block);

	/*
	 * writeEEPROM will write to the general purpose EEPROM (BLOCK0).
	 *
	 * address       - the address of the 1-wire device
	 * buf           - the data you want to write
	 * length        - the number of bytes to write.
	 * eeprom_offset - the start position in the EEPROM.
	 *
	 * returns       - the number of bytes written
	 */
	uint32_t  writeEEPROM(byte* buf, uint32_t length, uint32_t eeprom_offset);

	/*
	 * writeEEPROM will write to the general purpose EEPROM (BLOCK0).
	 *
	 * address       - the address of the 1-wire device
	 * buf           - the data you want to write
	 * length        - the number of bytes to write.
	 * eeprom_offset - the start position in the EEPROM.
	 *
	 * returns       - the number of bytes read
	 */
	uint32_t  readEEPROM(byte* buf, uint32_t length, uint32_t eeprom_offset);

	/*
	 * reset frees the memory allocated by the readDevice method
	 */
	void reset();

	/*
	 * resetProtectionRegister resets the DS2762_PR_OV_FLAG, DS2762_PR_UV_FLAG, DS2762_PR_COC_FLAG, DS2762_PR_DOC_FLAG flags
	 */
	void resetProtectionRegister();

    //Protection Register
    boolean isOverVoltage();
    boolean isUnderVoltage();
    boolean isChargeOverCurrent();
    boolean isDischargeOverCurrent();
    boolean isCCPin();
    boolean isDCPin();
    boolean isChargeEnable();
    boolean isDischargeEnable();

    //Status Register
    boolean isSleepModeEnabled();
    boolean isReadNetAddressOpcode();
    boolean isSWAPEnabled();
    boolean isInterruptEnabled();
    void setSWAPEnabled(boolean enabled);
    void setSleepMode(boolean enabled);

    //Special Feature Register
    boolean isPSPinLatch();
    boolean isPIO();
    boolean isSWAPMasterStatusBit();



};

#endif /* DS2762_H_ */
