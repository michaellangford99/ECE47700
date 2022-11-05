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

bool begin(uint8_t address)
{
	// Setting up the system clock to count the number of ticks
	#define LED_GPIO GPIOA
	#define LED_PIN 5

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//PC13 is the LED on test board
	//PA5 is the LED on the Nucleo
	LED_GPIO->MODER |= 0x1 << (LED_PIN * 2);
	LED_GPIO->MODER &= ~(0x2 << (LED_PIN * 2));

	//LED_GPIO->ODR |= 0x1 << LED_PIN;

	//SysTick->LOAD = 500 * (16000 - 1);
	SysTick->LOAD = 10*(10000 - 1);
	SysTick->VAL = 0;
	SysTick->CTRL |= (1<<0);

	SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_ENABLE_Msk |
			SysTick_CTRL_CLKSOURCE_Msk);


	// Initialize the selected I2C interface 
	bool ready = begin(address);
	
	// If the interface is not ready or TMF8801 is unreacheable return false
	if (ready == false)
	{
		activeDev->lastError = ERROR_I2C_COMM_ERROR;
		return false;
	}

	// Reset TMF8801. Since it clears itself, we don't need to clear it
	setRegisterBit(REGISTER_ENABLE_REG, CPU_RESET);

	ready = cpuReady();
	if (ready == false)
	{
		activeDev->lastError = ERROR_CPU_RESET_TIMEOUT;
		return false;
	}

	// Are we really talking to a TMF8801 ?
	uint8_t value = readSingleByte(REGISTER_ID);
	if (value != CHIP_ID_NUMBER)
	{
		activeDev->lastError = ERROR_WRONG_CHIP_ID;
		return false;
	}

	// Load the measurement application and wait until it's ready
	writeSingleByte(REGISTER_APPREQID, APPLICATION);
	ready = applicationReady();
	if (ready == false)
	{
		activeDev->lastError = ERROR_CPU_LOAD_APPLICATION_ERROR;
		return false;
	}

	// Set calibration data
	writeSingleByte(REGISTER_COMMAND, COMMAND_CALIBRATION);
	writeMultipleBytes(REGISTER_FACTORY_CALIB_0, activeDev->calibrationData, sizeof(activeDev->calibrationData));
	writeMultipleBytes(REGISTER_STATE_DATA_WR_0, ALGO_STATE, sizeof(ALGO_STATE));

	// Configure the application - values were taken from AN0597, pp. 22
	updateCommandData8();	

	// Start the application
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

uint32_t ticks = 0;
void SysTick_Handler(void)
{
	ticks++;
	LED_GPIO->ODR ^= 0x1 << LED_PIN;
	//LED_GPIO->ODR ^= 0x1 << LED_PIN;
}

uint32_t millis()
{
	return ticks;
}

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
	bool twiConnected = isConnected();
	if (!twiConnected)
		return false;

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
		nano_wait(100);
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
}

int getDistance()
{
	// Returns interrupt pin to open drain
	clearInterruptFlag();
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
	writeMultipleBytes(REGISTER_CMD_DATA7, activeDev->commandDataValues, sizeof(activeDev->commandDataValues));
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
	activeDev->commandDataValues[CMD_DATA_5] = currentRegisterValue;

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
	activeDev->commandDataValues[CMD_DATA_5] = currentRegisterValue;

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


