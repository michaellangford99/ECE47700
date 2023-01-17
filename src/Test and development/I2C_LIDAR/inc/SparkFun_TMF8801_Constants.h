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

#ifndef __TMF8801_CONSTANTS__
#define __TMF8801_CONSTANTS__

/*// Constants definitions

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
const uint8_t ALGO_STATE[11] = { 0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };*/

// Error constants
enum ERRORCONSTS {
	ERROR_NONE = 0x0,
	ERROR_I2C_COMM_ERROR = 0x01,
	ERROR_CPU_RESET_TIMEOUT = 0x02,
	ERROR_WRONG_CHIP_ID = 0x03,
	ERROR_CPU_LOAD_APPLICATION_ERROR = 0x04,
	ERROR_FACTORY_CALIBRATION_ERROR = 0x05
};

// GPIO mode
enum GPIOMODE
{
	MODE_INPUT = 0x0,
	MODE_LOW_INPUT = 0x01,
	MODE_HIGH_INPUT = 0x02,
	MODE_VCSEL = 0x03,
	MODE_LOW_OUTPUT = 0x04,
	MODE_HIGH_OUTPUT = 0x05,
};

// COMMAND constants
enum COMMAND
{
	CMD_DATA_7 = 0x0,
	CMD_DATA_6 = 0x01,
	CMD_DATA_5 = 0x02,
	CMD_DATA_4 = 0x03,
	CMD_DATA_3 = 0x04,
	CMD_DATA_2 = 0x05,
	CMD_DATA_1 = 0x06,
	CMD_DATA_0 = 0x07
};

// CPU status
enum CPUSTAT
{
	CPU_RESET= 0X07,
	CPU_READY = 0X06
};

// Registers definitions
enum regAddr2
{
	REGISTER_APPID = 0x00,
	REGISTER_APPREQID = 0x02,
	REGISTER_APPREV_MAJOR = 0x01,
	REGISTER_APPREV_MINOR = 0x12,
	REGISTER_APPREV_PATCH = 0x13,
	REGISTER_CMD_DATA9 = 0x06,
	REGISTER_CMD_DATA8 = 0x07,
	REGISTER_CMD_DATA7 = 0x08,
	REGISTER_CMD_DATA6 = 0x09,
	REGISTER_CMD_DATA5 = 0x0A,
	REGISTER_CMD_DATA4 = 0x0B,
	REGISTER_CMD_DATA3 = 0x0C,
	REGISTER_CMD_DATA2 = 0x0D,
	REGISTER_CMD_DATA1 = 0x0E,
	REGISTER_CMD_DATA0 = 0x0F,
	REGISTER_COMMAND = 0x10,
	REGISTER_PREVIOUS = 0x11,
	REGISTER_STATUS = 0x1D,
	REGISTER_REGISTER_CONTENTS = 0x1E,
	REGISTER_TID = 0x1F,
	REGISTER_RESULT_NUMBER = 0x20,
	REGISTER_RESULT_INFO = 0x21,
	REGISTER_DISTANCE_PEAK_0 = 0x22,
	REGISTER_DISTANCE_PEAK_1 = 0x23,
	REGISTER_SYS_CLOCK_0 = 0x24,
	REGISTER_SYS_CLOCK_1 = 0x25,
	REGISTER_SYS_CLOCK_2 = 0x26,
	REGISTER_SYS_CLOCK_3 = 0x27,
	REGISTER_STATE_DATA_0 = 0x28,
	REGISTER_STATE_DATA_1 = 0x29,
	REGISTER_STATE_DATA_2 = 0x2A,
	REGISTER_STATE_DATA_3 = 0x2B,
	REGISTER_STATE_DATA_4 = 0x2C,
	REGISTER_STATE_DATA_5 = 0x2D,
	REGISTER_STATE_DATA_6 = 0x2E,
	REGISTER_STATE_DATA_7 = 0x2F,
	REGISTER_STATE_DATA_8_XTALK_MSB = 0x30,
	REGISTER_STATE_DATA_9_XTALK_LSB = 0x31,
	REGISTER_STATE_DATA_10_TJ = 0x32,
	REGISTER_REFERENCE_HITS_0 = 0x33,
	REGISTER_REFERENCE_HITS_1 = 0x34,
	REGISTER_REFERENCE_HITS_2 = 0x35,
	REGISTER_REFERENCE_HITS_3 = 0x36,
	REGISTER_OBJECT_HITS_0 = 0x37,
	REGISTER_OBJECT_HITS_1 = 0x38,
	REGISTER_OBJECT_HITS_2 = 0x39,
	REGISTER_OBJECT_HITS_3 = 0x3A,
	REGISTER_FACTORY_CALIB_0 = 0x20,
	REGISTER_STATE_DATA_WR_0 = 0x2E,
	REGISTER_ENABLE_REG = 0xE0,
	REGISTER_INT_STATUS = 0xE1,
	REGISTER_INT_ENAB = 0xE2,
	REGISTER_ID = 0xE3,
	REGISTER_REVID = 0xE4
};

// Calibration data
enum calibrationDataLength
{
 CALIBRATION_DATA_LENGTH = 14
};
#endif