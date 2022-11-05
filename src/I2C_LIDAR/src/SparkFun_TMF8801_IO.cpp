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

#include "SparkFun_TMF8801_IO.h"
#include "SparkFun_TMF8801_Constants.h"
#include "SparkFun_TMF8801_Arduino_Library.h"
#include "i2c.h"

/*bool begin(byte address)
{
	_i2cPort = &wirePort;
	_address = address;
	return isConnected();
}
*/
bool isConnected()
{
	I2CstartWrite();
	I2Cstop();
	/*if (_i2cPort->endTransmission() != 0)
		return (false);*/
	return (true); 
}

void writeMultipleBytes(uint8_t registerAddress, const uint8_t* buffer, uint8_t const packetLength)
{
	 I2CstartWrite();
	 I2CbeginTransmission( activeDev->address);
	 for (uint8_t i = 0; i < packetLength; i++)
		 I2Cwrite((uint8_t)buffer[i]);
	 I2Cstop();
}

void readMultipleBytes(uint16_t registerAddress, uint8_t* buffer, uint8_t const packetLength)
{
	I2CbeginTransmission(activeDev->address);
	I2Cwrite(registerAddress >> 8);
	I2Cwrite(registerAddress);
	I2Cstop();

	I2CstartRead();
	I2CrequestFrom(activeDev->address, buffer, (uint8_t)17);
	/*I2Cs->requestFrom(_address, packetLength);
	for (byte i = 0; (i < packetLength) && _i2cPort->available(); i++)
		buffer[i] = _i2cPort->read();*/
}

uint8_t readSingleByte(uint8_t registerAddress)
{
	uint8_t result;
	I2CbeginTransmission(activeDev->address);
	I2Cwrite(registerAddress >> 8);
	I2Cwrite(registerAddress);
	I2Cstop();
	I2CstartRead();
	uint8_t buffer[17];
	I2CrequestFrom(activeDev->address, buffer, (uint8_t)17);
	result = buffer[0];
	return result;
}

void writeSingleByte(uint8_t registerAddress, uint8_t const value)
{
	I2CbeginTransmission(activeDev->address);
	I2Cwrite(registerAddress >> 8);
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

void clearRegisterBit(uint8_t registerAddress, uint8_t const bitPosition)
{
	uint8_t value = readSingleByte(registerAddress);
	value &= ~(1 << bitPosition);
	writeSingleByte(registerAddress, value);
}

bool isBitSet(uint8_t registerAddress, uint8_t const bitPosition)
{
	uint8_t value = readSingleByte(registerAddress);
	uint8_t mask = 1 << bitPosition;
	if (value & mask)
		return true;
	else
		return false;
}
