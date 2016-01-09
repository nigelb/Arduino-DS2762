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

#include "DS2762.h"

DS2762::DS2762(OneWire* bus, uint8_t* address, boolean read_all)
{
	this->bus = bus;
	this->address = address;
	this->memory = NULL;
	if(read_all)
	{
		readDevice();
	}
}

DS2762::~DS2762()
{
	reset();
}

void DS2762::_read_device(uint8_t* buf, uint8_t start, uint8_t count)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_READ_DATA);
	this->bus->write(start);
	this->bus->read_bytes(buf, count);
}

void DS2762::_write_device(uint8_t* buf, uint8_t start, uint8_t count)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_WRITE_DATA);
	this->bus->write(start);
	this->bus->write_bytes(buf, count);
	reset();
}

void DS2762::readDevice()
{
	memory = (uint8_t*)calloc(sizeof(uint8_t), 255);
	_read_device(memory, DS2762_PROTECTION_REG, 255);
}

boolean DS2762::_has_buffer()
{
	return this->memory != NULL;
}

uint16_t DS2762::_read_uint16(uint8_t addr_msb, uint8_t addr_lsb)
{
	uint8_t MSB, LSB;
	if(_has_buffer()){
		MSB = this->memory[addr_msb];
		LSB = this->memory[addr_lsb];

	}
	else
	{
		uint8_t data[2];
		_read_device(data, addr_msb, 2);
		MSB = data[0];
		LSB = data[1];
	}
	uint16_t count = ((MSB << 3 ) | (LSB >> 5)) ;
}

double DS2762::readADC()
{
	uint16_t count = _read_uint16(DS2762_VOLTAGE_MSB, DS2762_VOLTAGE_LSB);
	return (double(count) * 4880)/1000000;
}

uint16_t DS2762::readTempRaw()
{
	return _read_uint16(DS2762_TEMP_MSB, DS2762_TEMP_LSB);
}

double DS2762::readTempC()
{
	return (double(readTempRaw()) * 125) / 1000;

}
void DS2762::resetProtectionRegister()
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[DS2762_PROTECTION_REG];
	}else
	{
		_read_device(&_register, DS2762_PROTECTION_REG, 1);
	}
	_register = _register & (!(DS2762_PR_OV_FLAG | DS2762_PR_UV_FLAG | DS2762_PR_COC_FLAG | DS2762_PR_DOC_FLAG));
	_write_device(&_register, DS2762_PROTECTION_REG, 1);
}

void DS2762::reset()
{
	if(memory != NULL)
	{
		free (memory);
		memory = NULL;
	}
}

void DS2762::copyEEPROMBlock(DS2762_EEPROM block)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_COPY_DATA);
	this->bus->write(block);
	delay(5);
}
void DS2762::recallEEPROMBlock(DS2762_EEPROM block)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_RECALL_DATA);
	this->bus->write(block);
}



uint8_t DS2762::_read_register_bit(uint8_t register_address, uint8_t pin)
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[register_address];
	}else
	{
		_read_device(&_register, register_address, 1);
	}
	return _register & pin ;
}

boolean DS2762::isOverVoltage(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_OV_FLAG) > 0;
}

boolean DS2762::isUnderVoltage(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_UV_FLAG) > 0;
}

boolean DS2762::isChargeOverCurrent(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_COC_FLAG) > 0;
}

boolean DS2762::isDischargeOverCurrent(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_DOC_FLAG) > 0;
}

boolean DS2762::isCCPin(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_CCPIN_FLAG) > 0;
}

boolean DS2762::isDCPin(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_DCPIN_FLAG) > 0;
}

boolean DS2762::isChargeEnable(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_CE_FLAG) > 0;
}

boolean DS2762::isDischargeEnable(){
	return _read_register_bit(DS2762_PROTECTION_REG, DS2762_PR_DE_FLAG) > 0;
}


boolean DS2762::isSleepModeEnabled(){
	return _read_register_bit(DS2762_STATUS_REG, DS2762_STATUS_PMOD) > 0;
}
boolean DS2762::isReadNetAddressOpcode(){
	return _read_register_bit(DS2762_STATUS_REG, DS2762_STATUS_RNAOP) > 0;
}
boolean DS2762::isSWAPEnabled(){
	return _read_register_bit(DS2762_STATUS_REG, DS2762_STATUS_SWEN) > 0;
}
boolean DS2762::isInterruptEnabled(){
	return _read_register_bit(DS2762_STATUS_REG, DS2762_STATUS_IE) > 0;
}

boolean DS2762::isPSPinLatch(){
	return _read_register_bit(DS2762_SPECIAL_FEATURE_REG, DS2762_SF_PS_FLAG) > 0;
}

boolean DS2762::isPIO(){
	return _read_register_bit(DS2762_SPECIAL_FEATURE_REG, DS2762_SF_PIO_FLAG) > 0;
}
boolean DS2762::isSWAPMasterStatusBit(){
	return _read_register_bit(DS2762_SPECIAL_FEATURE_REG, DS2762_SF_MSTR_FLAG) > 0;
}


void DS2762::setSWAPEnabled(boolean enabled)
{
	_set_register_bit(DS2762_STATUS_REG, DS2762_STATUS_WRITE_REG, DS2762_STATUS_SWEN, BLOCK1, enabled);
}

void DS2762::setSleepMode(boolean enabled)
{
	_set_register_bit(DS2762_STATUS_REG, DS2762_STATUS_WRITE_REG, DS2762_STATUS_PMOD, BLOCK1, enabled);
}

void DS2762::_set_register_bit(uint8_t read_register_address, uint8_t write_register_address, uint8_t bit, DS2762_EEPROM block, boolean enabled)
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[read_register_address];
	}else
	{
		_read_device(&_register, read_register_address, 1);
	}

	if(enabled)
	{
		_register = _register | bit;
	}else
	{
		_register = _register & (!bit);
	}

	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_WRITE_DATA);
	this->bus->write(write_register_address);
	this->bus->write(_register);

	copyEEPROMBlock(BLOCK1);
	recallEEPROMBlock(BLOCK1);


	this->address = NULL;
}

uint32_t  DS2762::writeEEPROM(byte* buf, uint32_t length, uint32_t eeprom_offset)
{
	if((DS2762_EEPROM_BLOCK0 + length + eeprom_offset) > (0x2F + 1))
	{
		return 0;
	}
	_write_device(buf, DS2762_EEPROM_BLOCK0 + eeprom_offset, length);
	copyEEPROMBlock(BLOCK0);
	recallEEPROMBlock(BLOCK0);
	return length;
}

uint32_t  DS2762::readEEPROM(byte* buf, uint32_t length, uint32_t eeprom_offset)
{
	Serial.println(DS2762_EEPROM_BLOCK0 + length + eeprom_offset, HEX);
	if((DS2762_EEPROM_BLOCK0 + length + eeprom_offset) > (0x2F + 1))
	{
		return 0;
	}
	recallEEPROMBlock(BLOCK0);
	_read_device(buf, DS2762_EEPROM_BLOCK0 + eeprom_offset, length);
	return length;
}


