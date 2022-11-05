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

#ifndef __TMF8801_LIBRARY_IO__
#define __TMF8801_LIBRARY_IO__

#include "i2c.h"
#include "SparkFun_TMF8801_Constants.h"

	// Starts two wire interface.
bool begin(uint8_t address);

// Returns true if we get a reply from the I2C device.
bool isConnected();

// Read a single byte from a register.
uint8_t readSingleByte(uint16_t registerAddress);

// Writes a single byte into a register.
void writeSingleByte(uint16_t registerAddress, uint8_t value);

// Reads multiple bytes from a register into buffer byte array.
void readMultipleBytes(uint16_t registerAddress, uint16_t* buffer, uint16_t packetLength);

// Writes multiple bytes to register from buffer byte array.
void writeMultipleBytes(uint16_t registerAddress, const uint16_t* buffer, uint16_t packetLength);

// Sets a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
void setRegisterBit(uint16_t registerAddress, uint16_t bitPosition);

// Clears a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
void clearRegisterBit(uint16_t registerAddress, uint8_t bitPosition);

// Returns true if a specific bit is set in a register. Bit position ranges from 0 (lsb) to 7 (msb).
bool isBitSet(uint16_t registerAddress, uint8_t bitPosition);

