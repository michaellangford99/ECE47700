/*
  This is a library written for the AMS TMF-8801 Time-of-flight sensor
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/17716

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 15th, 2021
  This file is the core of the TMF-8801 ToF sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f4xx.h"
#include "SparkFun_TMF8801_Arduino_Library.h"
#include "SparkFun_TMF8801_IO.h"

#include "fir.h"

// Constants definitions

const uint8_t DEFAULT_I2C_ADDR = 0x41;
const uint8_t CPU_READY_TIMEOUT = 200;
const uint8_t APPLICATION_READY_TIMEOUT = 200;
const uint8_t CHIP_ID_NUMBER = 0x07;
const uint8_t APPLICATION = 0xc0;
const uint8_t BOOTLOADER = 0x80;
const uint8_t COMMAND_CALIBRATION = 0x0b;
const uint8_t COMMAND_FACTORY_CALIBRATION = 0x0a;
const uint8_t COMMAND_MEASURE = 0x02;
const uint8_t COMMAND_RESULT = 0x55;
const uint8_t COMMAND_SERIAL = 0x47;
const uint8_t COMMAND_STOP = 0xff;
const uint8_t INTERRUPT_MASK = 0x01;
const uint8_t CONTENT_CALIBRATION = 0x0a;

// Values below were taken from AN000597, pp 22
const uint8_t ALGO_STATE[11] = { 0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

float lidar_buffer[24];
struct fir_filter lidar_filter;
uint16_t distance;
float filtered_distance;

void init_TMF8801(TMF8801_t* dev)
{
	bool devInit = _TMF8801_init(dev);
	nano_wait(1000);

	if(devInit == true)
	{
			enableInterrupt();
			nano_wait(1000);
			getSerialNumber();
			nano_wait(1000);
			getHardwareVersion();
			nano_wait(1000);
			getApplicationVersionMajor();
			nano_wait(1000);
			getApplicationVersionMinor();
			nano_wait(1000);
	}
	else
	{
		getStatus();
	}

	if(!isConnected())
	{
		wakeUpDevice();
	}
	nano_wait(1000);

	lidar_filter.impulse_response = hanning_8;
	lidar_filter.length = 8;
	lidar_filter.first_element = 0;
	lidar_filter.circular_buffer = lidar_buffer;
}

#define UPDATE_PERIOD 100
static int l;
void update_TMF8801(TMF8801_t* dev)
{
	l++;
	if (l > UPDATE_PERIOD)
	{
		l = 0;

		read_distance(dev);

		//do some filtering here
		float fdist = (float)distance;
		shift_filter(&lidar_filter, &fdist, 1);
		filtered_distance = compute_filter(&lidar_filter);
	}
}

uint16_t get_distance_TMF8801(TMF8801_t* dev)
{
	return distance;//filtered_distance;
}

void read_distance(TMF8801_t* dev) {
	if(dataAvailable())
	{
		distance = getDistance();
	}
}
bool _TMF8801_init(TMF8801_t* dev) {


	activeDev = dev;
	activeDev->address = 0x41;

	((activeDev->commandDataValues))[0] = 0x03;
	((activeDev->commandDataValues))[1] = 0x23;
	(activeDev->commandDataValues)[2] = 0x44;
	(activeDev->commandDataValues)[3] = 0x00;
	(activeDev->commandDataValues)[4] = 0x00;
	(activeDev->commandDataValues)[5] = 0x64;
	(activeDev->commandDataValues)[6] = 0xD8;
	(activeDev->commandDataValues)[7] = 0xA4;
	activeDev->gpio1_prog = MODE_LOW_OUTPUT;
	activeDev->gpio0_prog = MODE_LOW_OUTPUT;
	activeDev->calibrationData[0] = 0xC1;
	activeDev->calibrationData[1] = 0x22;
	activeDev->calibrationData[2] = 0x00;
	activeDev->calibrationData[3] = 0x1C;
	activeDev->calibrationData[4] = 0x9;
	activeDev->calibrationData[5] = 0x40;
	activeDev->calibrationData[6] = 0x8C;
	activeDev->calibrationData[7] = 0x98;
	activeDev->calibrationData[8] = 0xA;
	activeDev->calibrationData[9] = 0x15;
	activeDev->calibrationData[10] = 0xCE;
	activeDev->calibrationData[11] = 0x9C;
	activeDev->calibrationData[12] = 0x1;
	activeDev->calibrationData[13] = 0xFC;
	return begin(activeDev->address);
}

bool begin(uint8_t address)
{
	// Initialize the selected I2C interface 
	bool ready;

	ready = isConnected();
	// If the interface is not ready or TMF8801 is unreacheable return false
	if (ready == false)
	{
		activeDev->lastError = ERROR_I2C_COMM_ERROR;
		return false;
	}

	// Reset TMF8801. Since it clears itself, we don't need to clear it
	nano_wait(1000);
	setRegisterBit(REGISTER_ENABLE_REG, CPU_RESET);


	nano_wait(1000);
	ready = cpuReady();
	if (ready == false)
	{
		activeDev->lastError = ERROR_CPU_RESET_TIMEOUT;
		return false;
	}

	// Are we really talking to a TMF8801 ?
	nano_wait(1000);
	uint8_t value = readSingleByte(REGISTER_ID);
	if (value != CHIP_ID_NUMBER)
	{
		activeDev->lastError = ERROR_WRONG_CHIP_ID;
		return false;
	}

	// Load the measurement application and wait until it's ready
	nano_wait(1000);
	writeSingleByte(REGISTER_APPREQID, APPLICATION);
	ready = applicationReady();
	if (ready == false)
	{
		activeDev->lastError = ERROR_CPU_LOAD_APPLICATION_ERROR;
		return false;
	}

	// Set calibration data
	nano_wait(1000);
	writeSingleByte(REGISTER_COMMAND, COMMAND_CALIBRATION);
	nano_wait(1000);
	writeMultipleBytes(REGISTER_FACTORY_CALIB_0, activeDev->calibrationData, sizeof(activeDev->calibrationData));
	nano_wait(1000);
	writeMultipleBytes(REGISTER_STATE_DATA_WR_0, ALGO_STATE, sizeof(ALGO_STATE));

	// Configure the application - values were taken from AN0597, pp. 22
	nano_wait(1000);
	updateCommandData8();	

	// Start the application
	nano_wait(1000);
	writeSingleByte(REGISTER_COMMAND, COMMAND_MEASURE);

	//delay(10);
	nano_wait(100);

	// Set lastError no NONE
	activeDev->lastError = ERROR_NONE;
	return true;
}

void TMF8801_setDevice(TMF8801_t* device)
{
  activeDev= device;
}

uint32_t ticksTMF = 0;

bool cpuReady()
{
	short counter = 0;

	// Wait for CPU_READY_TIMEOUT mSec until TMF8801 is ready
	do
	{
		bool ready = isBitSet(REGISTER_ENABLE_REG, CPU_READY);
		if (ready == false)
		{
			counter++;
			//delay(100);
			nano_wait(1000);
		}
		else
		{
			return true;
		}
	} while (counter < CPU_READY_TIMEOUT);

	// If TMF8801 CPU is not ready, return false
	return false;
}

bool dataAvailable()
{
	// Returns true if REGISTER_CONTENTS is 0x55
	uint8_t result = readSingleByte(REGISTER_REGISTER_CONTENTS);
	return result == COMMAND_RESULT;
}

bool isConnected()
{
	// Polls I2C interface
	//bool twiConnected = isConnected();
	//if (!twiConnected)
	//	return false;

	// Returns true if TMF8801 ID returned id is 0x07
	return (readSingleByte(REGISTER_ID) == CHIP_ID_NUMBER);
}

bool applicationReady()
{
	short counter = 0;

	// Wait for APPLICATION_READY_TIMEOUT mSec until TMF8801 is ready
	do
	{
		bool ready = (readSingleByte(REGISTER_APPID) == APPLICATION);
		if (ready == false)
		{
			counter++;
			//delay(100);
			nano_wait(1000);
		}
		else
		{
			return true;
		}
	} while (counter < APPLICATION_READY_TIMEOUT);

	// If application is not ready, return false
	return false;
}

uint8_t getLastError()
{
	return activeDev->lastError;
}

bool getCalibrationData(uint8_t* calibrationResults)
{
	writeSingleByte(REGISTER_COMMAND, 0xff);
	//delay(50);
	nano_wait(500);

	// Returns device's calibration data values (14 bytes)
	activeDev->lastError = ERROR_NONE;
	uint32_t calibrationStart = millis();

	uint8_t value;
	do
	{
		writeSingleByte(REGISTER_COMMAND, COMMAND_FACTORY_CALIBRATION);
		//delay(10);
		nano_wait(100);
		value = readSingleByte(REGISTER_REGISTER_CONTENTS);
		if (value == CONTENT_CALIBRATION)
		{
			//delay(10);
			nano_wait(100);
			readMultipleBytes(REGISTER_FACTORY_CALIB_0, calibrationResults, CALIBRATION_DATA_LENGTH);
			return true;
		}
		//delay(50);
		nano_wait(500);
	} while (millis() - calibrationStart < 30000);
	
	// returns false and writes the lastError if TMF8801 calibration data read operation fails
	activeDev->lastError = ERROR_FACTORY_CALIBRATION_ERROR;
	return false;
}

void setCalibrationData(const uint8_t* newCalibrationData)
{
	// Copies passed array into calibrationData
	memcpy(activeDev->calibrationData, newCalibrationData, CALIBRATION_DATA_LENGTH);

	// Reset device with updated values
	resetDevice();
}

uint8_t getApplicationVersionMajor()
{
	return readSingleByte(REGISTER_APPREV_MAJOR);
}

uint8_t getApplicationVersionMinor()
{
	return readSingleByte(REGISTER_APPREV_MINOR);
}

uint8_t getHardwareVersion()
{
	return readSingleByte(REGISTER_REVID);
}

short getSerialNumber()
{
	short serial = 0;
	uint8_t value[2];
	uint8_t result;
	// Request serial number to device
	do
	{
		writeSingleByte(REGISTER_COMMAND, COMMAND_SERIAL);
		//delay(50);
		nano_wait(500);
		result = readSingleByte(REGISTER_REGISTER_CONTENTS);
		//delay(10);
		nano_wait(100);
	} while (result != COMMAND_SERIAL);

	// Read two bytes and combine them as a single int
	readMultipleBytes(REGISTER_STATE_DATA_0, value, 2);
	serial = value[1];
	serial = serial << 8;
	serial |= value[0];
	return serial;
}

uint8_t getMeasurementReliability()
{
	// Returns result info without measurement status bits
	return (activeDev->resultInfo & 0x3f);
}

uint8_t getMeasurementStatus()
{
	// returns resultInfo without measurement reliability bits
	return (activeDev->resultInfo >> 6);
}

uint8_t getMeasurementNumber()
{
	return activeDev->resultNumber;
}

void resetDevice()
{
	// Applies newly updated array into main application
	setRegisterBit(REGISTER_ENABLE_REG, CPU_RESET);

	// Checks if CPU is ready
	bool ready = false;
	do
	{
		ready = cpuReady();
	} while (!ready);

	// Loads measurement application
	writeSingleByte(REGISTER_APPREQID, APPLICATION);
	ready = false;
	do
	{
		ready = applicationReady();
	} while (!ready);

	// Write calibration data and algorithm state into device
	writeSingleByte(REGISTER_COMMAND, COMMAND_CALIBRATION);
	writeMultipleBytes(REGISTER_FACTORY_CALIB_0, activeDev->calibrationData, sizeof(activeDev->calibrationData));
	writeMultipleBytes(REGISTER_STATE_DATA_WR_0, ALGO_STATE, sizeof(ALGO_STATE));

	// Updates CMD_DATA_7 to CMD_DATA_0
	updateCommandData8();

	// Start measurements application
	writeSingleByte(REGISTER_COMMAND, COMMAND_MEASURE);

	// Wait 50 msec then return
	//delay(50);
	nano_wait(500);
}

void wakeUpDevice()
{
	uint8_t result;
	// Write ENABLE_REG to bring device back to operation and wait until it's back
	do
	{
			writeSingleByte(REGISTER_ENABLE_REG, 0x01);
			result = readSingleByte(REGISTER_ENABLE_REG);
			//delay(100);
			nano_wait(1000);
	} while (result != 0x41);
}

uint8_t getStatus()
{
	return readSingleByte(REGISTER_STATUS);
}

void doMeasurement()
{
	uint8_t buffer[4];
	readMultipleBytes(REGISTER_RESULT_NUMBER, buffer, sizeof(buffer));
	activeDev->resultNumber = buffer[0];
	activeDev->resultInfo = buffer[1];
	activeDev->distancePeak = buffer[3];
	activeDev->distancePeak = activeDev->distancePeak << 8;
	activeDev->distancePeak += buffer[2];

	if (activeDev->distancePeak == 0)
		activeDev->distancePeak = 660;
}

int getDistance()
{
	// Returns interrupt pin to open drain
	//clearInterruptFlag();
	// Reads measurement data
	doMeasurement();
	return activeDev->distancePeak;
}

void enableInterrupt()
{
	uint8_t registerValue = readSingleByte(REGISTER_INT_ENAB);
	registerValue |= INTERRUPT_MASK;
	writeSingleByte(REGISTER_INT_ENAB, registerValue);
	//delay(10);
	nano_wait(100);
	doMeasurement();
}

void disableInterrupt()
{
	uint8_t registerValue = readSingleByte(REGISTER_INT_ENAB);
	registerValue &= ~INTERRUPT_MASK;
	writeSingleByte(REGISTER_INT_ENAB, registerValue);
}

void clearInterruptFlag()
{
	uint8_t registerValue = readSingleByte(REGISTER_INT_STATUS);
	registerValue |= INTERRUPT_MASK;
	writeSingleByte(REGISTER_INT_STATUS, registerValue);
}

void updateCommandData8()
{
	// Writes commandDataValues array into CMD_DATA_7 to CMD_DATA_0 registers
	writeMultipleBytes(REGISTER_CMD_DATA7, (activeDev->commandDataValues), sizeof((activeDev->commandDataValues)));
}

bool measurementEnabled()
{
	// Returns true if resultInfo 7:6 are both zeroed
	uint8_t result = activeDev->resultInfo;
	result = result >> 6;
	return result == 0;
}

void setGPIO0Mode(uint8_t gpioMode)
{
	// Does not allow invalid values to be set into register
	if (gpioMode > MODE_HIGH_OUTPUT)
		return;

	uint8_t currentRegisterValue;

	// Read current value and change only GPIO0 values
	currentRegisterValue = readSingleByte(REGISTER_CMD_DATA0);
	currentRegisterValue &= 0xf0;
	currentRegisterValue += gpioMode;
	(activeDev->commandDataValues)[CMD_DATA_5] = currentRegisterValue;

	// Send command to device
	uint8_t buffer[2];
	buffer[0] = currentRegisterValue;
	buffer[1] = 0x0f;
	writeMultipleBytes(REGISTER_CMD_DATA0, buffer, 2);
}

uint8_t getGPIO0Mode()
{
	// Read REGISTER_CMD_DATA0 and mask accordingly
	uint8_t currentRegisterValue;
	currentRegisterValue = readSingleByte(REGISTER_CMD_DATA0);
	return (currentRegisterValue & 0x0f);
}

void setGPIO1Mode(uint8_t gpioMode)
{	
	// Does not allow invalid values to be set into register
	if (gpioMode > MODE_HIGH_OUTPUT)
		return;

	uint8_t currentRegisterValue;

	// Read current value and change only GPIO1 values
	currentRegisterValue = readSingleByte(REGISTER_CMD_DATA0);
	currentRegisterValue &= 0x0f;
	currentRegisterValue += (gpioMode << 4);
	(activeDev->commandDataValues)[CMD_DATA_5] = currentRegisterValue;

	// Send command to device
	uint8_t buffer[2];
	buffer[0] = currentRegisterValue;
	buffer[1] = 0x0f;
	writeMultipleBytes(REGISTER_CMD_DATA0, buffer, 2);
}

uint8_t getGPIO1Mode()
{
	// Read REGISTER_CMD_DATA0 and shift accordingly
	uint8_t currentRegisterValue;
	currentRegisterValue = readSingleByte(REGISTER_CMD_DATA0);
	return (currentRegisterValue >> 4);
}

uint8_t getRegisterValue(uint8_t reg)
{
	return readSingleByte(reg);
}

void setRegisterValue(uint8_t reg, uint8_t value)
{
	writeSingleByte(reg, value);
}

void getRegisterMultipleValues(uint8_t reg, uint8_t* buffer, uint8_t length)
{
	readMultipleBytes(reg, buffer, length);
}

void setRegisterMultipleValues(uint8_t reg, const uint8_t* buffer, uint8_t length)
{
	writeMultipleBytes(reg, buffer, length);
}

bool isConnectedTMF8801()
{
	I2CstartWrite();
	I2Cstop();
	/*if (_i2cPort->endTransmission() != 0)
		return (false);*/
	return (true);
}

void writeMultipleBytes(uint8_t registerAddress, uint8_t* buffer, uint16_t packetLength)
{
	 I2CstartWrite();
	 I2CbeginTransmission( activeDev->address);
	 I2Cwrite(registerAddress);
	 for (uint8_t i = 0; i < packetLength; i++)
		 I2Cwrite((uint8_t)buffer[i]);
	 I2Cstop();
}

void readMultipleBytes(uint8_t registerAddress, uint8_t* buffer, uint16_t packetLength)
{
	I2CstartWrite();
	I2CbeginTransmission(activeDev->address);
	//I2Cwrite(registerAddress >> 8);
	I2Cwrite(registerAddress);
	I2Cstop();

	I2CstartRead();

	I2CrequestFrom(activeDev->address, buffer, packetLength);
	/*I2Cs->requestFrom(_address, packetLength);
	for (byte i = 0; (i < packetLength) && _i2cPort->available(); i++)
		buffer[i] = _i2cPort->read();*/
	I2Cstop();
}

uint8_t readSingleByte(uint8_t registerAddress)
{
	uint8_t result;
	I2CstartWrite();
	nano_wait(100);
	I2CbeginTransmission((activeDev->address));
	I2Cwrite(registerAddress);
	I2Cstop();
	I2CstartRead();
	uint8_t buffer[2];
	I2CrequestFrom(activeDev->address, buffer, (uint8_t)2);
	result = buffer[0];
	I2Cstop();
	return result;
}

void writeSingleByte(uint8_t registerAddress, uint8_t value)
{
	I2CstartWrite();
	nano_wait(100);
	I2CbeginTransmission(activeDev->address);
	//I2Cwrite(registerAddress >> 8);
	I2Cwrite(registerAddress);
	I2Cwrite(value);
	I2Cstop();
}

void setRegisterBit(uint8_t registerAddress, uint8_t const bitPosition)
{
		uint8_t value = readSingleByte(registerAddress);
		value |= (1 << bitPosition);
		writeSingleByte(registerAddress, value);
}

void clearRegisterBit(uint8_t registerAddress, uint8_t bitPosition)
{
	uint8_t value = readSingleByte(registerAddress);
	value &= ~(1 << bitPosition);
	writeSingleByte(registerAddress, value);
}

bool isBitSet(uint8_t registerAddress, uint8_t bitPosition)
{
	uint8_t value = readSingleByte(registerAddress);
	uint8_t mask = 1 << bitPosition;
	if (value & mask)
		return true;
	else
		return false;
}


