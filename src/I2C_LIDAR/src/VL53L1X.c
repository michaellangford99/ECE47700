//
// Most of the functionality of this library is based on the VL53L1X API
// provided by ST (STSW-IMG007), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2356), and
// VL53L1X datasheet.

#include "stm32f4xx.h"
#include "VL53L1X.h"
#include "i2c.h"

// Constructors ////////////////////////////////////////////////////////////////

bool init();

bool VL53L1X_init(VL53L1X_t* device)
{
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




  active_device = device;

  active_device->dev_address = AddressDefault;
  active_device->dev_io_timeout = 0; // no timeout
  active_device->dev_did_timeout = false;
  active_device->dev_calibrated = false;
  active_device->dev_saved_vhv_init = 0;
  active_device->dev_saved_vhv_timeout = 0;
  active_device->dev_distance_mode = Unknown;

  VL53L1X_setTimeout(500);

  return init();
}

void VL53L1X_setDevice(VL53L1X_t* device)
{
  active_device = device;
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

// Public Methods //////////////////////////////////////////////////////////////

void VL53L1X_setAddress(uint8_t new_addr)
{
  writeReg(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
  active_device->dev_address = new_addr;
}

uint8_t VL53L1X_getAddress()
{
	return active_device->dev_address;
}

// Initialize sensor using settings taken mostly from VL53L1_DataInit() and
// VL53L1_StaticInit().
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool init()
{
  nano_wait(1000);
  // check model ID and module type registers (values specified in datasheet)
  uint16_t x = 0x00000000;
  while(x != 0xEACC) {
	  x = readReg16Bit(IDENTIFICATION__MODEL_ID);
  }
  if (x != 0xEACC) { return false; }

  // VL53L1_software_reset() begin

  writeReg(SOFT_RESET, 0x00);
  nano_wait(1000);
  writeReg(SOFT_RESET, 0x01);

  // give it some time to boot; otherwise the sensor NACKs during the readReg()
  // call below and the Arduino 101 doesn't seem to handle that well
  nano_wait(10000);

  // VL53L1_poll_for_boot_completion() begin

  startTimeout();

  // check last_status in case we still get a NACK to try to deal with it correctly
  while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || active_device->dev_last_status != 0)
  {
    if (checkTimeoutExpired())
    {
      active_device->dev_did_timeout = true;
      return false;
    }
  }
  // VL53L1_poll_for_boot_completion() end

  // VL53L1_software_reset() end

  // VL53L1_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  
  
    writeReg(PAD_I2C_HV__EXTSUP_CONFIG,
      readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
  

  // store oscillator info for later use
  active_device->dev_fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
  active_device->dev_osc_calibrate_val = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end

  // VL53L1_StaticInit() begin

  // Note that the API does not actually apply the configuration settings below
  // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
  // register contents in memory and doesn't actually write them until a
  // measurement is started. Writing the configuration here means we don't have
  // to keep it all in memory and avoids a lot of redundant writes later.

  // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
  // VL53L1_set_preset_mode() begin

  // VL53L1_preset_mode_standard_ranging() begin

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // (API uses these in VL53L1_init_tuning_parm_storage_struct())

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
  // that? (seems like it would disable 2V8 mode)
  writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
  writeReg(GPIO__TIO_HV_STATUS, 0x02);
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
  writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  writeReg(ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
  writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

  // general config
  writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
  writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
  writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
  writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config

  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  writeReg(SD_CONFIG__QUANTIFIER, 2); // tuning parm default

  // VL53L1_preset_mode_standard_ranging() end

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
  // and things don't seem to work if we don't set GPH back to 0 (which the API
  // does here).
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  writeReg(SYSTEM__SEED_CONFIG, 1); // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
  writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

  // VL53L1_set_preset_mode() end

  // default to long range, 50 ms timing budget
  // note that this is different than what the API defaults to
  VL53L1X_setDistanceMode(Long);
  setMeasurementTimingBudget(50000);

  // VL53L1_StaticInit() end

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
    readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);

  return true;
}


// Write an 8-bit register
void writeReg(uint16_t reg, uint8_t value)
{
  I2CstartWrite();
  I2CbeginTransmission(active_device->dev_address);
  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
  I2Cwrite((uint8_t)(reg));      // reg low byte
  I2Cwrite(value);
  I2Cstop();
  //active_device->dev_last_status = bus->endTransmission();
}

// Write a 16-bit register
void writeReg16Bit(uint16_t reg, uint16_t value)
{
  I2CstartWrite();
  I2CbeginTransmission(active_device->dev_address);
  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
  I2Cwrite((uint8_t)(reg));      // reg low byte
  I2Cwrite((uint8_t)(value >> 8));
  I2Cwrite((uint8_t)(value));
  I2Cstop();
  //active_device->dev_last_status = bus->endTransmission();
}

// Write a 32-bit register
void writeReg32Bit(uint16_t reg, uint32_t value)
{
  I2CstartWrite();
  I2CbeginTransmission(active_device->dev_address);
  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
  I2Cwrite((uint8_t)(reg));
  I2Cwrite((uint8_t)(value >> 24)); // value highest byte
  I2Cwrite((uint8_t)(value >> 16));
  I2Cwrite((uint8_t)(value >> 8));
  I2Cwrite((uint8_t)(value));
  I2Cstop();
}

// Read an 8-bit register
uint8_t readReg(uint16_t reg)
{
	  uint16_t value;

		  I2CstartWrite();
		  I2Caddress(active_device->dev_address);
		  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
		  I2Cwrite((uint8_t)(reg));
		  I2Cstop();
		  //nano_wait(100);
		  uint8_t buffer[17];
		  I2CstartRead();
		  I2CrequestFrom((active_device->dev_address), buffer, (uint8_t)17);
		  value = (buffer)[0];      // All value
		  I2Cstop();

	  return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(uint16_t reg)
{
  uint16_t value;


	  I2CstartWrite();
	  I2Caddress(active_device->dev_address);
	  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
	  I2Cwrite((uint8_t)(reg));
	  I2Cstop();
	  //nano_wait(100);
	  uint8_t buffer[17];
	  I2CstartRead();
	  I2CrequestFrom((active_device->dev_address), buffer, (uint8_t)17);
	  value = (uint16_t)((buffer)[0] <<8) | (uint16_t)(buffer)[1];      // All value
	  I2Cstop();

  return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(uint16_t reg)
{
  uint32_t value;

  I2CstartWrite();
  I2CbeginTransmission(active_device->dev_address);
  I2Cwrite((uint8_t)(reg >> 8)); // reg high byte
  I2Cwrite((uint8_t)(reg));
  I2Cstop();

  uint8_t buffer[4];
  I2CstartRead();
  I2CrequestFrom(active_device->dev_address, buffer, (uint8_t)4);
  value = (uint32_t)(buffer[0] << 24)| (uint32_t)(buffer[1] << 16) | (uint16_t)(buffer[2] <<8) | (uint16_t)buffer[3];      // All value
  I2Cstop();

  return value;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool VL53L1X_setDistanceMode(DistanceMode_t mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();

  switch (mode)
  {
    case Short:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x07);
      writeReg(SD_CONFIG__WOI_SD1, 0x05);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x0B);
      writeReg(SD_CONFIG__WOI_SD1, 0x09);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x0F);
      writeReg(SD_CONFIG__WOI_SD1, 0x0D);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  // reapply timing budget
  setMeasurementTimingBudget(budget_us);

  // save mode so it can be returned by getDistanceMode()
  active_device->dev_distance_mode = mode;

  return true;
}

DistanceMode_t VL53L1X_getDistanceMode()
{
	return active_device->dev_distance_mode;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool setMeasurementTimingBudget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t getMeasurementTimingBudget()
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
    readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}

// Set the width and height of the region of interest
// based on VL53L1X_SetROI() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully.
void setROISize(uint8_t width, uint8_t height)
{
  if ( width > 16) {  width = 16; }
  if (height > 16) { height = 16; }

  // Force ROI to be centered if width or height > 10, matching what the ULD API
  // does. (This can probably be overridden by calling setROICenter()
  // afterwards.)
  if (width > 10 || height > 10)
  {
    writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
  }

  writeReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
           (height - 1) << 4 | (width - 1));
}

// Get the width and height of the region of interest (ROI)
// based on VL53L1X_GetROI_XY() from STSW-IMG009 Ultra Lite Driver
void getROISize(uint8_t * width, uint8_t * height)
{
  uint8_t reg_val = readReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
  *width = (reg_val & 0xF) + 1;
  *height = (reg_val >> 4) + 1;
}

// Set the center SPAD of the region of interest (ROI)
// based on VL53L1X_SetROICenter() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully. Here is a table of SPAD locations from
// UM2555 (199 is the default/center):
//
// 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
// 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
// 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
// 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
// 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
// 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
// 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
// 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255
//
// 127,119,111,103, 95, 87, 79, 71,   63, 55, 47, 39, 31, 23, 15,  7
// 126,118,110,102, 94, 86, 78, 70,   62, 54, 46, 38, 30, 22, 14,  6
// 125,117,109,101, 93, 85, 77, 69,   61, 53, 45, 37, 29, 21, 13,  5
// 124,116,108,100, 92, 84, 76, 68,   60, 52, 44, 36, 28, 20, 12,  4
// 123,115,107, 99, 91, 83, 75, 67,   59, 51, 43, 35, 27, 19, 11,  3
// 122,114,106, 98, 90, 82, 74, 66,   58, 50, 42, 34, 26, 18, 10,  2
// 121,113,105, 97, 89, 81, 73, 65,   57, 49, 41, 33, 25, 17,  9,  1
// 120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0 <- Pin 1
//
// This table is oriented as if looking into the front of the sensor (or top of
// the chip). SPAD 0 is closest to pin 1 of the VL53L1X, which is the corner
// closest to the VDD pin on the Pololu VL53L1X carrier board:
//
//   +--------------+
//   |             O| GPIO1
//   |              |
//   |             O|
//   | 128    248   |
//   |+----------+ O|
//   ||+--+  +--+|  |
//   |||  |  |  || O|
//   ||+--+  +--+|  |
//   |+----------+ O|
//   | 120      0   |
//   |             O|
//   |              |
//   |             O| VDD
//   +--------------+
//
// However, note that the lens inside the VL53L1X inverts the image it sees
// (like the way a camera works). So for example, to shift the sensor's FOV to
// sense objects toward the upper left, you should pick a center SPAD in the
// lower right.
void setROICenter(uint8_t spadNumber)
{
  writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNumber);
}

// Get the center SPAD of the region of interest
// based on VL53L1X_GetROICenter() from STSW-IMG009 Ultra Lite Driver
uint8_t getROICenter()
{
  return readReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD);
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void VL53L1X_startContinuous(uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * active_device->dev_osc_calibrate_val);

  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  writeReg(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Stop continuous measurements
// based on VL53L1_stop_range()
void VL53L1X_stopContinuous()
{
  writeReg(SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  active_device->dev_calibrated = false;

  // "restore vhv configs"
  if (active_device->dev_saved_vhv_init != 0)
  {
    writeReg(VHV_CONFIG__INIT, active_device->dev_saved_vhv_init);
  }
  if (active_device->dev_saved_vhv_timeout != 0)
  {
     writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, active_device->dev_saved_vhv_timeout);
  }

  // "remove phasecal override"
  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}

// Returns a range reading in millimeters when continuous mode is active. If
// blocking is true (the default), this function waits for a new measurement to
// be available. If blocking is false, it will try to return data immediately.
// (VL53L1X_readSingle() also calls this function after starting a single-shot range
// measurement)

uint16_t last[10];
int l = 0;


uint16_t VL53L1X_read()
{
  
  startTimeout();
  while (!VL53L1X_dataReady())
  {
    if (checkTimeoutExpired())
    {
      active_device->dev_did_timeout = true;
      return 0;
    }
  }

  readResults();

  if (!active_device->dev_calibrated)
  {
    setupManualCalibration();
    active_device->dev_calibrated = true;
  }

  updateDSS();

  getRangingData();

  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

  return active_device->dev_ranging_data.range_mm;
}

uint16_t VL53L1X_readRangeContinuousMillimeters()
{
	return VL53L1X_read();
}

// Starts a single-shot range measurement. If blocking is true (the default),
// this function waits for the measurement to finish and returns the reading.
// Otherwise, it returns 0 immediately.
uint16_t VL53L1X_readSingle()
{
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  writeReg(SYSTEM__MODE_START, 0x10); // mode_range__single_shot

  return VL53L1X_read();
}

uint16_t VL53L1X_readRangeSingleMillimeters()
{
	return VL53L1X_readSingle();
}

bool VL53L1X_dataReady()
{
	return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0;
}
// convert a RangeStatus_t to a readable string
// Note that on an AVR, these strings are stored in RAM (dynamic memory), which
// makes working with them easier but uses up 200+ bytes of RAM (many AVR-based
// Arduinos only have about 2000 bytes of RAM). You can avoid this memory usage
// if you do not call this function in your sketch.
const char * VL53L1X_rangeStatusToString(RangeStatus_t status)
{
  switch (status)
  {
    case RangeValid:
      return "range valid";

    case SigmaFail:
      return "sigma fail";

    case SignalFail:
      return "signal fail";

    case RangeValidMinRangeClipped:
      return "range valid, min range clipped";

    case OutOfBoundsFail:
      return "out of bounds fail";

    case HardwareFail:
      return "hardware fail";

    case RangeValidNoWrapCheckFail:
      return "range valid, no wrap check fail";

    case WrapTargetFail:
      return "wrap target fail";

    case XtalkSignalFail:
      return "xtalk signal fail";

    case SynchronizationInt:
      return "synchronization int";

    case MinRangeFail:
      return "min range fail";

    case None:
      return "no update";

    default:
      return "unknown status";
  }
}

void VL53L1X_setTimeout(uint16_t timeout)
{
	active_device->dev_io_timeout = timeout;
}

uint16_t VL53L1X_getTimeout()
{
	return active_device->dev_io_timeout;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L1X_timeoutOccurred()
{
  bool tmp = active_device->dev_did_timeout;
  active_device->dev_did_timeout = false;
  return tmp;
}

void startTimeout()
{
	active_device->dev_timeout_start_ms = millis();
}

bool checkTimeoutExpired()
{
	return (active_device->dev_io_timeout > 0) && ((uint16_t)(millis() - active_device->dev_timeout_start_ms) > active_device->dev_io_timeout);
}

// Private Methods /////////////////////////////////////////////////////////////

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
void setupManualCalibration()
{
  // "save original vhv configs"
  active_device->dev_saved_vhv_init = readReg(VHV_CONFIG__INIT);
  active_device->dev_saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  writeReg(VHV_CONFIG__INIT, active_device->dev_saved_vhv_init & 0x7F);

  // "set loop bound to tuning param"
  writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (active_device->dev_saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

  // "override phasecal"
  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
  writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
}

// read measurement results into buffer
void readResults()
{
  I2CstartWrite();
  I2Caddress (active_device->dev_address);
  I2Cwrite((uint8_t)(RESULT__RANGE_STATUS >> 8)); // reg high byte
  I2Cwrite((uint8_t)(RESULT__RANGE_STATUS));      // reg low byte
  I2Cstop();
  uint8_t buffer[17];
  I2CstartRead();
  I2CrequestFrom(active_device->dev_address, buffer, (uint8_t) 17);
//  bus->requestFrom(active_device->dev_address, buffer, (uint8_t)17);

  active_device->dev_results.range_status = buffer[16];

 // bus->read(); // report_status: not used

  active_device->dev_results.stream_count = buffer[14];

  active_device->dev_results.dss_actual_effective_spads_sd0  = (uint16_t)((uint16_t)buffer[13]<< 8); // high byte
  active_device->dev_results.dss_actual_effective_spads_sd0 |=           buffer[12];      // low byte

//  bus->read(); // peak_signal_count_rate_mcps_sd0: not used
//  bus->read();

  active_device->dev_results.ambient_count_rate_mcps_sd0  = (uint16_t)((uint16_t)buffer[9] << 8); // high byte
  active_device->dev_results.ambient_count_rate_mcps_sd0 |=           (buffer[8]);      // low byte

//  bus->read(); // sigma_sd0: not used
//  bus->read();

//  bus->read(); // phase_sd0: not used
//  bus->read();

  active_device->dev_results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)((uint16_t)buffer[3] << 8); // high byte
  active_device->dev_results.final_crosstalk_corrected_range_mm_sd0 |=           (buffer[2] << 8);      // low byte

  active_device->dev_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)((uint16_t)buffer[1]<< 8); // high byte
  active_device->dev_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           (buffer[0]);      // low byte
  I2Cstop();

  /*for (int i = 0; i < 17; i++)
  {
	  printf("%x, ", buffer[i]);
  }*/
  uint32_t sum = 0;
  for (int i = 0; i < 10-1; i++)
  {
	  last[i] = last[i+1];
	  sum += last[i];
  }
  last[9]=active_device->dev_results.dss_actual_effective_spads_sd0;
  sum += last[9];

  printf("%d, ", sum);


  printf("%d, ", active_device->dev_results.ambient_count_rate_mcps_sd0);
  printf("%d, ", active_device->dev_results.dss_actual_effective_spads_sd0);
  printf("%d, ", active_device->dev_results.final_crosstalk_corrected_range_mm_sd0);
  printf("%d, ", active_device->dev_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  printf("%d, ", active_device->dev_results.range_status);
  printf("%d, ", active_device->dev_results.stream_count);
  printf("%d, ", active_device->dev_ranging_data.range_mm);

  printf("\n");
}

// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
void updateDSS()
{
  uint16_t spadCount = active_device->dev_results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)active_device->dev_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      active_device->dev_results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
void getRangingData()
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = active_device->dev_results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  active_device->dev_ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(active_device->dev_results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      active_device->dev_ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      active_device->dev_ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      active_device->dev_ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      active_device->dev_ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      active_device->dev_ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      active_device->dev_ranging_data.range_status = SigmaFail;
      break;

    case 7: // PHASECONSISTENCY
      active_device->dev_ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      active_device->dev_ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      active_device->dev_ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (active_device->dev_results.stream_count == 0)
      {
        active_device->dev_ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        active_device->dev_ranging_data.range_status = RangeValid;
      }
      break;

    default:
      active_device->dev_ranging_data.range_status = None;
  }

  // from SetSimpleData()
  active_device->dev_ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(active_device->dev_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  active_device->dev_ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(active_device->dev_results.ambient_count_rate_mcps_sd0);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / active_device->dev_fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

float countRateFixedToFloat(uint16_t count_rate_fixed)
{
	return (float)count_rate_fixed / (1 << 7);
}

