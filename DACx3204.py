#!/usr/bin/python
import ft4222
import ft4222.I2CMaster
from ft4222.SPI import Cpha, Cpol
from ft4222.SPIMaster import Mode, Clock, SlaveSelect
from ft4222.GPIO import Dir, Port, Output
import time

class DACx3204:

# ======================================================================
	ADDR_DAC_0_MARGIN_HIGH         = 0x01
	ADDR_DAC_0_MARGIN_LOW          = 0x02	
	ADDR_DAC_1_MARGIN_HIGH         = 0x07
	ADDR_DAC_1_MARGIN_LOW          = 0x08
	ADDR_DAC_2_MARGIN_HIGH         = 0x0D
	ADDR_DAC_2_MARGIN_LOW          = 0x0E
	ADDR_DAC_3_MARGIN_HIGH         = 0x13
	ADDR_DAC_3_MARGIN_LOW          = 0x14
	
	OFFSET_DAC_MARGIN              = 4
	WIDTH_DAC_MARGIN               = 12 # shorter widths have zero in the upper bits
# ======================================================================
	ADDR_DAC_0_VOUT_CMP_CONFIG     = 0x03
	ADDR_DAC_1_VOUT_CMP_CONFIG     = 0x09
	ADDR_DAC_2_VOUT_CMP_CONFIG     = 0x0F
	ADDR_DAC_3_VOUT_CMP_CONFIG     = 0x15

	OFFSET_CMP_EN          = 0 # ENABLE comparator mode
	OFFSET_CMP_INV_EN      = 1 # INVERT comparator output
	OFFSET_CMP_HIZ_IN_DIS  = 2 # FBx pin is resistor divider
	OFFSET_CMP_OUT_EN      = 3 # DRIVE TO OUTx pin
	OFFSET_CMP_OD_EN       = 4 # OPEN DRAIN ENABLE
	OFFSET_VOUT_GAIN       = 10
	WIDTH_VOUT_GAIN        = 3
	ENC_VOUT_GAIN_1X_VREF  = 0
	ENC_VOUT_GAIN_1X_VDD   = 1
	ENC_VOUT_GAIN_1P5X_INT = 2
	ENC_VOUT_GAIN_2X_INT   = 3
	ENC_VOUT_GAIN_3X_INT   = 4
	ENC_VOUT_GAIN_4X_INT   = 5
# ======================================================================
	ADDR_DAC_0_IOUT_MISC_CONFIG    = 0x04
	ADDR_DAC_1_IOUT_MISC_CONFIG    = 0x0A
	ADDR_DAC_2_IOUT_MISC_CONFIG    = 0x10
	ADDR_DAC_3_IOUT_MISC_CONFIG    = 0x16
	
	OFFSET_IOUT_RANGE    = 9
	WIDTH_IOUT_RANGE     = 4
	ENC_IOUT_RANGE_25UA  = 8
	ENC_IOUT_RANGE_50UA  = 9
	ENC_IOUT_RANGE_125UA = 10
	ENC_IOUT_RANGE_250UA = 11
# ======================================================================
	ADDR_DAC_0_CMP_MODE_CONFIG     = 0x05
	ADDR_DAC_1_CMP_MODE_CONFIG     = 0x0B
	ADDR_DAC_2_CMP_MODE_CONFIG     = 0x11
	ADDR_DAC_3_CMP_MODE_CONFIG     = 0x17
	
	OFFSET_CMP_MODE = 10
	WIDTH_CMP_MODE  = 2
	ENC_CMP_MODE_NO_HYST_NO_WINDOW   = 0
	ENC_CMP_MODE_HYST_FROM_MARGIN    = 1
	ENC_CMP_MODE_WINDOWS_FROM_MARGIN = 2
# ======================================================================
	ADDR_DAC_0_FUNC_CONFIG         = 0x06
	ADDR_DAC_1_FUNC_CONFIG         = 0x0C
	ADDR_DAC_2_FUNC_CONFIG         = 0x12
	ADDR_DAC_3_FUNC_CONFIG         = 0x18
	
	OFFSET_SLEW_RATE         = 0
	WIDTH_SLEW_RATE          = 4
	ENC_SLEW_RATE_0US        = 0 # invalid for waveform generation
	ENC_SLEW_RATE_4US        = 1
	ENC_SLEW_RATE_8US        = 2
	ENC_SLEW_RATE_12US       = 3
	ENC_SLEW_RATE_18S        = 4
	ENC_SLEW_RATE_27US       = 5
	ENC_SLEW_RATE_40US       = 6
	ENC_SLEW_RATE_60US       = 7
	ENC_SLEW_RATE_91US       = 8
	ENC_SLEW_RATE_136US      = 9
	ENC_SLEW_RATE_239US      = 10
	ENC_SLEW_RATE_418US      = 11
	ENC_SLEW_RATE_732US      = 12
	ENC_SLEW_RATE_1282US     = 13
	ENC_SLEW_RATE_2563US     = 14
	ENC_SLEW_RATE_5127US     = 15
	OFFSET_CODE_STEP         = 4
	WIDTH_CODE_STEP          = 3
	ENC_CODE_STEP_1LSB       = 0
	ENC_CODE_STEP_2LSB       = 1
	ENC_CODE_STEP_3LSB       = 2
	ENC_CODE_STEP_4LSB       = 3
	ENC_CODE_STEP_6LSB       = 4
	ENC_CODE_STEP_8LSB       = 5
	ENC_CODE_STEP_16LSB      = 6
	ENC_CODE_STEP_32LSB      = 7
	OFFSET_LOG_SLEW_EN       = 7
	OFFSET_FUNC_CONFIG       = 8
	WIDTH_FUNC_CONFIG        = 3
	ENC_FUNC_CONFIG_TRIANGLE = 0
	ENC_FUNC_CONFIG_SAWTOOTH = 1
	ENC_FUNC_CONFIG_INV_SAW  = 2
	ENC_FUNC_CONFIG_SINE     = 4
	ENC_FUNC_CONFIG_DISABLE  = 7
	OFFSET_PHASE_SEL         = 11
	WIDTH_PHASE_SEL          = 2
	ENC_PHASE_SEL_0DEG       = 0
	ENC_PHASE_SEL_120DEG     = 1
	ENC_PHASE_SEL_240DEG     = 2
	ENC_PHASE_SEL_90DEG      = 3
	OFFSET_BRD_CONFIG        = 13
	ENC_BRD_CONFIG_NO_UPDATE = 0
	ENC_BRD_CONFIG_UPDATE    = 1
	OFFSET_SYNC_CONFIG       = 14
	ENC_SYNC_CONFIG_UP_WRITE = 0
	ENC_SYNC_CONFIG_UP_LDAC  = 1
	OFFSET_CLR_SEL           = 15
	ENC_CLR_SEL_ZERO_SCALE   = 0
	ENC_CLR_SEL_MID_SCALE    = 1
# ======================================================================
	ADDR_DAC_0_DATA                = 0x19
	ADDR_DAC_1_DATA                = 0x1A
	ADDR_DAC_2_DATA                = 0x1B
	ADDR_DAC_3_DATA                = 0x1C
	
	OFFSET_DAC_DATA = 4
	WIDTH_DAC_DATA  = 12 # shorter widths have zero in the upper bits
# ======================================================================
	ADDR_COMMON_CONFIG             = 0x1F
	
	OFFSET_IOUT_PDN_0          = 0
	OFFSET_VOUT_PDN_0          = 1
	OFFSET_IOUT_PDN_1          = 3
	OFFSET_VOUT_PDN_1          = 4
	OFFSET_IOUT_PDN_2          = 6
	OFFSET_VOUT_PDN_2          = 7
	OFFSET_IOUT_PDN_3          = 9
	OFFSET_VOUT_PDN_3          = 10
	WIDTH_VOUT_PDN             = 2
	ENC_VOUT_PDN_PWR_UP        = 0
	ENC_VOUT_PDN_PWR_DOWN_10K  = 1
	ENC_VOUT_PDN_PWR_DOWN_100K = 2
	ENC_VOUT_PDN_PWR_DOWN_HIZ  = 3
	OFFSET_EN_INT_REF          = 12
	OFFSET_EN_READ_ADDR        = 13
	ENC_EN_READ_ADDR_0         = 0
	ENC_EN_READ_ADDR_1         = 1
	OFFSET_DEV_LOCK            = 14
	OFFSET_WIN_LATCH_EN        = 15
# ======================================================================
	ADDR_COMMON_TRIGGER            = 0x20
	
	OFFSET_NVM_RELOAD       = 0
	OFFSET_NVM_PROG         = 1
	OFFSET_READ_ONE_TRIG    = 2
	OFFSET_PROTECT          = 3
	OFFSET_FAULT_DUMP       = 4
	OFFSET_CLR              = 6
	OFFSET_LDAC             = 7
	OFFSET_RESET            = 8
	WIDTH_RESET             = 4
	ENC_RESET_PASSWORD      = 10
	OFFSET_DEV_UNLOCK       = 12
	WIDTH_DEV_UNLOCK        = 4
	ENC_DEV_UNLOCK_PASSWORD = 5
# ======================================================================
	ADDR_COMMON_DAC_TRIG           = 0x21
	
	OFFSET_START_FUNC_3     = 0
	OFFSET_TRIG_MAR_HI_3    = 1
	OFFSET_TRIG_MAR_LOW_3   = 2
	OFFSET_RESET_CMP_FLAG_3 = 3
	OFFSET_START_FUNC_2     = 4
	OFFSET_TRIG_MAR_HI_2    = 5
	OFFSET_TRIG_MAR_LOW_2   = 6
	OFFSET_RESET_CMP_FLAG_2 = 7
	OFFSET_START_FUNC_1     = 8
	OFFSET_TRIG_MAR_HI_1    = 9
	OFFSET_TRIG_MAR_LOW_1   = 10
	OFFSET_RESET_CMP_FLAG_1 = 11
	OFFSET_START_FUNC_0     = 12
	OFFSET_TRIG_MAR_HI_0    = 13
	OFFSET_TRIG_MAR_LOW_0   = 14
	OFFSET_RESET_CMP_FLAG_0 = 15
# ======================================================================
	ADDR_GENERAL_STATUS            = 0x22
	
	OFFSET_VERSION_ID        = 0
	WIDTH_VERSION_ID         = 2
	OFFSET_DEVICE_ID         = 2
	WIDTH_DEVICE_ID          = 6
	ENC_DEVICE_ID_DAC63204   = 1
	ENC_DEVICE_ID_DAC53204   = 2
	ENC_DEVICE_ID_DAC43204   = 3
	OFFSET_DAC_0_BUSY        = 9
	OFFSET_DAC_1_BUSY        = 10
	OFFSET_DAC_2_BUSY        = 11
	OFFSET_DAC_3_BUSY        = 12
	OFFSET_NVM_CRC_FAIL_USER = 14
	OFFSET_NVM_CRC_FAIL_INT  = 15
# ======================================================================
	ADDR_CMP_STATUS                = 0x23
	
	OFFSET_CMP_FLAG_0   = 0
	OFFSET_CMP_FLAG_1   = 1
	OFFSET_CMP_FLAG_2   = 2
	OFFSET_CMP_FLAG_3   = 3
	OFFSET_WIN_CMP_0    = 4
	OFFSET_WIN_CMP_1    = 5
	OFFSET_WIN_CMP_2    = 6
	OFFSET_WIN_CMP_3    = 7
	OFFSET_PROTECT_FLAG = 8
# ======================================================================
	ADDR_GPIO_CONFIG               = 0x24
	
	OFFSET_GPI_EN                 = 0
	OFFSET_GPI_CONFIG             = 1
	WIDTH_GPI_CONFIG              = 4
	ENC_GPI_CONFIG_FAULT_DUMP     = 2
	ENC_GPI_CONFIG_IOUT_UP_DOWN   = 3
	ENC_GPI_CONFIG_VOUT_UP_DOWN   = 4
	ENC_GPI_CONFIG_CLR            = 7
	ENC_GPI_CONFIG_LDAC           = 8
	ENC_GPI_CONFIG_START_STOP     = 9
	ENC_GPI_CONFIG_TRIGGER_MARGIN = 10
	ENC_GPI_CONFIG_RESET          = 11
	ENC_GPI_CONFIG_NVM_PROTECT    = 12
	ENC_GPI_CONFIG_REG_MAP_LOCK   = 13
	OFFSET_GPI_CH_0_SEL           = 5
	OFFSET_GPI_CH_1_SEL           = 6
	OFFSET_GPI_CH_2_SEL           = 7
	OFFSET_GPI_CH_3_SEL           = 8
	OFFSET_GPO_CONFIG             = 9
	WIDTH_GPO_CONFIG              = 4
	ENC_GPO_CONFIG_NVM_BUSY       = 1
	ENC_GPO_CONFIG_DAC_0_BUSY     = 4
	ENC_GPO_CONFIG_DAC_1_BUSY     = 5
	ENC_GPO_CONFIG_DAC_2_BUSY     = 6
	ENC_GPO_CONFIG_DAC_3_BUSY     = 7
	ENC_GPO_CONFIG_WIN_CMP_0_BUSY = 8
	ENC_GPO_CONFIG_WIN_CMP_0_BUSY = 9
	ENC_GPO_CONFIG_WIN_CMP_0_BUSY = 10
	ENC_GPO_CONFIG_WIN_CMP_0_BUSY = 11
	OFFSET_GPO_EN                 = 13
	OFFSET_GF_EN                  = 15
# ======================================================================
	ADDR_DEVICE_MODE_CONFIG        = 0x25
	
	OFFSET_PROTECT_CONFIG                        = 8
	WIDTH_PROTECT_CONFIG                         = 2
	ENC_PROTECT_CONFIG_HIZ_PWR_DOWN              = 0
	ENC_PROTECT_CONFIG_NVM_DAC_HIZ_PWR_DOWN      = 1
	ENC_PROTECT_CONFIG_MAR_LOW_DAC_HIZ_PWR_DOWN  = 2
	ENC_PROTECT_CONFIG_MAR_HIGH_DAC_HIZ_PWR_DOWN = 3
	OFFSET_DIS_MODE_IN                           = 13
# ======================================================================
	ADDR_INTERFACE_CONFIG          = 0x26
	
	OFFSET_SDO_EN     = 0
	OFFSET_FSDO_EN    = 2
	OFFSET_EN_PMBUS   = 8
	OFFSET_TIMEOUT_EN = 12
# ======================================================================
	ADDR_SRAM_CONFIG               = 0x2B
	
	OFFSET_SRAM_ADDR = 0
	WIDTH_SRAM_ADDR  = 8
# ======================================================================
	ADDR_SRAM_DATA                 = 0x2C
	
	OFFSET_SRAM_DATA = 0
	WIDTH_SRAM_DATA  = 16


	def __init__(self, bus='i2c', use_gpio=False, gpio_direction='input', gpio_initial_level='high', debug=False):
		self.debug        = debug
		self.bus          = bus
		self.devA         = ft4222.openByDescription('FT4222 A')
		self.i2c_dev_addr = 0x47
		if (bus == 'i2c'): # Make sure that jumper J5 is pins 2,3 (touching usb end of board)
			self.devA.i2cMaster_Init(400000)
			self.devA.i2cMaster_Reset()
			if (self.debug):
				print("I2C bus initialized")
		else: # Make sure that jumper J5 is pins 1,2 (not touching usb end of board), J6 jumper is removed for SPI reading
			# DIV_32 is 2 MHz, DIV_16 is 4MHz, DIV_8 is 8MHz, DIV_4 is 16MHz, DIV_2 is 32MHz, SPI READS can only be done at DIV_64 1MHz!!
			self.devA.spiMaster_Init(Mode.SINGLE, Clock.DIV_64, Cpol.IDLE_LOW, Cpha.CLK_TRAILING, SlaveSelect.SS0)
			if (self.debug):
				print("SPI bus initialized")
		if (use_gpio): # Make sure that J6 is connected, SDO-EN is not set in NVM (this is default with new board)
			self.devA.setWakeUpInterrupt(False)
			if (gpio_direction == 'input'):
				self.devA.gpio_Init(gpio3 = Dir.INPUT)
				if (self.debug):
					print("GPIO initialized as INPUT")
			else:
				self.devA.gpio_Init(gpio3 = Dir.OUTPUT)
				if (gpio_initial_level == 'high'):
					self.devA.gpio_Write(Port.P3, True)
					if (self.debug):
						print("GPIO initialized as OUTPUT, initial value HIGH")
				else:
					self.devA.gpio_Write(Port.P3, False)
					if (self.debug):
						print("GPIO initialized as OUTPUT, initial value LOW")


	def ft4222_show_devices(self):
		nbDev = ft4222.createDeviceInfoList()
		for i in range(nbDev):
			print(ft4222.getDeviceInfoDetail(i, False))
		
	def gpio_set_high(self):
		self.devA.gpio_Write(Port.P3, True)
		if (self.debug):
			print("Set GPIO high")
		
	def gpio_set_low(self):
		self.devA.gpio_Write(Port.P3, False)
		if (self.debug):
			print("Set GPIO low")

	def gpio_read(self):
		val = self.devA.gpio_Read(Port.P3)
		if (self.debug):
			if (val):
				print("GPIO is high")
			else:
				print("GPIO is LOW")
		return val

	def ft4222_i2cdetect(self):
		list_of_slave_id = []
		for slave_dev_id in range(1,128):
			self.devA.i2cMaster_ReadEx(slave_dev_id,ft4222.I2CMaster.Flag.START_AND_STOP,1)
			status = str(self.devA.i2cMaster_GetStatus())
			if (status == NACK_FOUND or status == '32'):
				list_of_slave_id.append(slave_dev_id)
				print("I2C Slave ID found 0x{:02x}".format(slave_dev_id))
			self.devA.i2cMaster_Reset()
		return list_of_slave_id

	def i2c_read_word(self, read_addr=0x00):
		self.devA.i2cMaster_Write(self.i2c_dev_addr, read_addr)
		word_data = int.from_bytes(self.devA.i2cMaster_Read(self.i2c_dev_addr,2),"big")
		if (self.debug):
			print("I2C Read Address 0x{:02x} Data 0x{:04x}".format(read_addr,word_data))
		return word_data

	def i2c_write_word(self, write_addr=0x00, write_data=0x0000):
		if (self.debug):
			print("I2C Write Address 0x{:02x} Data 0x{:04x}".format(write_addr,write_data))
		byte0 = write_data & 0xFF
		byte1 = (write_data >> 8) & 0xFF
		self.devA.i2cMaster_Write(self.i2c_dev_addr, bytearray([write_addr, byte1, byte0]))
		
	def spi_read_word(self, read_addr=0x00): # NOTE: the SDO-EN bit must already be set in NVM and J6 jumper is removed
		self.devA.spiMaster_Init(Mode.SINGLE, Clock.DIV_64, Cpol.IDLE_LOW, Cpha.CLK_TRAILING, SlaveSelect.SS0) # SLOW SPI CLOCK for reads at 1MHz
		self.devA.spiMaster_SingleReadWrite(bytearray([0x80+read_addr, 0x00, 0x00]), True) #first write the address, 24bits total
		byte_list = list(self.devA.spiMaster_SingleReadWrite(bytearray([0x00, 0x00, 0x00]), True)) # read data is in the last 16bits, 24 bits total
		self.devA.spiMaster_Init(Mode.SINGLE, Clock.DIV_2, Cpol.IDLE_LOW, Cpha.CLK_TRAILING, SlaveSelect.SS0) # RETURN TO FAST 32MHz writes
		word_value = byte_list[1] * 256 + byte_list[2]
		if (self.debug):
			print("SPI Read Address 0x{:02x} Data 0x{:04x}".format(read_addr,word_value))
		return word_value

	def spi_write_word(self, write_addr=0x00, write_data=0x0000):
		byte0 =  write_data       & 0xFF
		byte1 = (write_data >> 8) & 0xFF
		self.devA.spiMaster_SingleReadWrite(bytearray([0x00+write_addr, byte1, byte0]), True)
		if (self.debug):
			print("SPI Write Address 0x{:02x} Data 0x{:02x}".format(write_addr,write_data))


	def read_word(self, read_addr=0x00):
		if (self.bus == 'i2c'):
			return self.i2c_read_word(read_addr=read_addr)
		else:
			return self.spi_read_word(read_addr=read_addr)
	
	def write_word(self, write_addr=0x00, write_data=0x0000):
		if (self.bus == 'i2c'):
			self.i2c_write_word(write_addr=write_addr, write_data=write_data)
		else:
			self.spi_write_word(write_addr=write_addr, write_data=write_data)

	def set_bits(self, addr, offset, width, bit_value):
		if (bit_value > 2 ** width -1):
			print("Value out of range")
			return
		else:
			read_val = self.read_word(read_addr=addr)
			bit_mask = (2 ** width - 1) << offset
			mod_val = (read_val & (~bit_mask)) + (bit_value << offset)
			if (self.debug):
				print("Set bits at address 0x{:02x} mod_val = 0x{:04x}".format(addr,mod_val))
			self.write_word(write_addr=addr, write_data=mod_val)
	
	def read_bits(self, addr, offset, width):
		read_val = self.read_word(read_addr=addr)
		mod_val = (read_val >> offset) & (2 ** width -1)
		if (self.debug):
			print("Read bits from address 0x{:02x} value is 0x{:04x}".format(addr,mod_val))
		return mod_val
		
	def device_unlock(self):
		self.set_bits(addr=self.ADDR_COMMON_TRIGGER, offset=self.OFFSET_DEV_UNLOCK, width=self.WIDTH_DEV_UNLOCK, bit_value=self.ENC_DEV_UNLOCK_PASSWORD)
		unlock_status = self.read_bits(addr=self.ADDR_COMMON_CONFIG, offset=self.OFFSET_DEV_LOCK, width=1)
		if (unlock_status == 1):
			print("ERROR unable to unlock the device")

	def por_reset(self):
		self.set_bits(addr=self.ADDR_COMMON_TRIGGER, offset=self.OFFSET_RESET, width=self.WIDTH_RESET, bit_value=self.ENC_RESET_PASSWORD)
		time.sleep(1)
	
	def nvm_prog(self):
		self.set_bits(addr=self.ADDR_COMMON_TRIGGER, offset=self.OFFSET_NVM_PROG, width=1, bit_value=1)
		time.sleep(1)

	def check_spi_read_sdo_en_bit(self):
		sdo_en_val = self.read_bits(addr=self.ADDR_INTERFACE_CONFIG, offset=self.OFFSET_SDO_EN, width=1)
		if (sdo_en_val == 1):
			print("SDO_EN bit is high, SPI reads will work now")
		else:
			print("SDO_EN bit is low, please set the bit and program to NVM")
		
	def set_sdo_en_bit_in_nvm(self, por_reset_en=1):
		if (por_reset_en == 1):
			self.por_reset()
		self.device_unlock()
		self.set_bits(addr=self.ADDR_INTERFACE_CONFIG, offset=self.OFFSET_SDO_EN, width=1, bit_value=1)
		self.nvm_prog()
	
	def clear_sdo_en_bit_in_nvm(self, por_reset_en=1):
		if (por_reset_en == 1):
			self.por_reset()
		self.device_unlock()
		self.set_bits(addr=self.ADDR_INTERFACE_CONFIG, offset=self.OFFSET_SDO_EN, width=1, bit_value=0)
		self.nvm_prog()

	def write_margin_high(self, channel=0, margin_val=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_MARGIN_HIGH
		elif (channel == 1):
			addr = self.ADDR_DAC_1_MARGIN_HIGH
		elif (channel == 2):
			addr = self.ADDR_DAC_2_MARGIN_HIGH
		elif (channel == 3):
			addr = self.ADDR_DAC_3_MARGIN_HIGH
		self.set_bits(addr=addr, offset=self.OFFSET_DAC_MARGIN, width=self.WIDTH_DAC_MARGIN, bit_value=margin_val)
	
	def write_margin_low(self, channel=0, margin_val=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_MARGIN_LOW
		elif (channel == 1):
			addr = self.ADDR_DAC_1_MARGIN_LOW
		elif (channel == 2):
			addr = self.ADDR_DAC_2_MARGIN_LOW
		elif (channel == 3):
			addr = self.ADDR_DAC_3_MARGIN_LOW
		self.set_bits(addr=addr, offset=self.OFFSET_DAC_MARGIN, width=self.WIDTH_DAC_MARGIN, bit_value=margin_val)
	
	def write_vout_comp_config(self, channel=0, enc_vout_gain=0, cmp_en=0, cmp_inv_en=0, cmp_hiz_in_dis=0, cmp_out_en=0, cmp_od_en=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_VOUT_CMP_CONFIG
		elif (channel == 1):
			addr = self.ADDR_DAC_1_VOUT_CMP_CONFIG
		elif (channel == 2):
			addr = self.ADDR_DAC_2_VOUT_CMP_CONFIG
		elif (channel == 3):
			addr = self.ADDR_DAC_3_VOUT_CMP_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_VOUT_GAIN,      width=self.WIDTH_VOUT_GAIN, bit_value=enc_vout_gain)
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_EN,         width=1, bit_value=cmp_en)
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_INV_EN,     width=1, bit_value=cmp_inv_en)
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_HIZ_IN_DIS, width=1, bit_value=cmp_his_in_dis)
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_OUT_EN,     width=1, bit_value=cmp_out_en)
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_OD_EN,      width=1, bit_value=cmp_od_en)
	
	def write_iout_range(self, channel=0, enc_iout_range=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_IOUT_MISC_CONFIG
		elif (channel == 1):
			addr = self.ADDR_DAC_1_IOUT_MISC_CONFIG
		elif (channel == 2):
			addr = self.ADDR_DAC_2_IOUT_MISC_CONFIG
		elif (channel == 3):
			addr = self.ADDR_DAC_3_IOUT_MISC_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_IOUT_RANGE, width=self.WIDTH_IOUT_RANGE, bit_value=enc_iout_range)
		
	def write_cmp_mode(self, channel=0, enc_cmp_mode=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_CMP_MODE_CONFIG
		elif (channel == 1):
			addr = self.ADDR_DAC_1_CMP_MODE_CONFIG
		elif (channel == 2):
			addr = self.ADDR_DAC_2_CMP_MODE_CONFIG
		elif (channel == 3):
			addr = self.ADDR_DAC_3_CMP_MODE_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_CMP_MODE, width=self.WIDTH_CMP_MODE, bit_value=enc_cmp_mode)
	
	def write_func_config(self, channel=0, enc_slew_rate=0, enc_code_step=0, log_slew_en=0, enc_func_config=0, enc_phase_sel=0, enc_brd_config=0, enc_sync_config=0, enc_clr_sel=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_FUNC_CONFIG
		elif (channel == 1):
			addr = self.ADDR_DAC_1_FUNC_CONFIG
		elif (channel == 2):
			addr = self.ADDR_DAC_2_FUNC_CONFIG
		elif (channel == 3):
			addr = self.ADDR_DAC_3_FUNC_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_SLEW_RATE,   width=self.WIDTH_SLEW_RATE,   bit_value=enc_slew_rate)
		self.set_bits(addr=addr, offset=self.OFFSET_CODE_STEP,   width=self.WIDTH_CODE_STEP,   bit_value=enc_code_step)
		self.set_bits(addr=addr, offset=self.OFFSET_LOG_SLEW_EN, width=1,                      bit_value=log_slew_en)
		self.set_bits(addr=addr, offset=self.OFFSET_FUNC_CONFIG, width=self.WIDTH_FUNC_CONFIG, bit_value=enc_func_config)
		self.set_bits(addr=addr, offset=self.OFFSET_PHASE_SEL,   width=self.WIDTH_PHASE_SEL,   bit_value=enc_phase_sel)
		self.set_bits(addr=addr, offset=self.OFFSET_BRD_CONFIG,  width=1,                      bit_value=enc_brd_config)
		self.set_bits(addr=addr, offset=self.OFFSET_SYNC_CONFIG, width=1,                      bit_value=enc_sync_config)
		self.set_bits(addr=addr, offset=self.OFFSET_CLR_SEL,     width=1,                      bit_value=enc_clr_sel)

	def write_data(self, channel=0, data=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_DATA
		elif (channel == 1):
			addr = self.ADDR_DAC_1_DATA
		elif (channel == 2):
			addr = self.ADDR_DAC_2_DATA
		elif (channel == 3):
			addr = self.ADDR_DAC_3_DATA
		self.set_bits(addr=addr, offset=self.OFFSET_DAC_DATA, width=self.WIDTH_DAC_DATA, bit_value=data)

	def write_common_config_1(self, channel=0, iout_pdn=1, enc_vout_pdn=3): # POR values
		addr = self.ADDR_COMMON_CONFIG
		if (channel == 0):
			self.set_bits(addr=addr, offset=self.OFFSET_IOUT_PDN_0, width=1, bit_value=iout_pdn)
			self.set_bits(addr=addr, offset=self.OFFSET_VOUT_PDN_0, width=self.WIDTH_VOUT_PDN, bit_value=enc_vout_pdn)
		elif (channel == 1):
			self.set_bits(addr=addr, offset=self.OFFSET_IOUT_PDN_1, width=1, bit_value=iout_pdn)
			self.set_bits(addr=addr, offset=self.OFFSET_VOUT_PDN_1, width=self.WIDTH_VOUT_PDN, bit_value=enc_vout_pdn)
		elif (channel == 2):
			self.set_bits(addr=addr, offset=self.OFFSET_IOUT_PDN_2, width=1, bit_value=iout_pdn)
			self.set_bits(addr=addr, offset=self.OFFSET_VOUT_PDN_2, width=self.WIDTH_VOUT_PDN, bit_value=enc_vout_pdn)
		elif (channel == 3):
			self.set_bits(addr=addr, offset=self.OFFSET_IOUT_PDN_3, width=1, bit_value=iout_pdn)
			self.set_bits(addr=addr, offset=self.OFFSET_VOUT_PDN_3, width=self.WIDTH_VOUT_PDN, bit_value=enc_vout_pdn)
			
	def write_common_config_2(self, en_int_ref=0, dev_lock=0, win_latch_en=0):
		addr = self.ADDR_COMMON_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_EN_INT_REF,   width=1, bit_value=en_int_ref)
		self.set_bits(addr=addr, offset=self.OFFSET_DEV_LOCK,     width=1, bit_value=dev_lock)
		self.set_bits(addr=addr, offset=self.OFFSET_WIN_LATCH_EN, width=1, bit_value=win_latch_en)
	
	def write_gpio_config(self, gpi_en=0, enc_gpi_config=0, gpi_ch0_sel=0, gpi_ch1_sel=0, gpi_ch2_sel=0, gpi_ch3_sel=0, enc_gpo_config=0, gpo_en=0, gf_en=0):
		addr = self.ADDR_GPIO_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_EN,       width=1,                     bit_value=gpi_en)
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_CONFIG,   width=self.WIDTH_GPI_CONFIG, bit_value=enc_gpi_config)
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_CH_0_SEL, width=1,                     bit_value=gpi_ch0_sel)
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_CH_1_SEL, width=1,                     bit_value=gpi_ch1_sel)
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_CH_2_SEL, width=1,                     bit_value=gpi_ch2_sel)
		self.set_bits(addr=addr, offset=self.OFFSET_GPI_CH_3_SEL, width=1,                     bit_value=gpi_ch3_sel)
		self.set_bits(addr=addr, offset=self.OFFSET_GPO_CONFIG,   width=self.WIDTH_GPO_CONFIG, bit_value=enc_gpo_config)
		self.set_bits(addr=addr, offset=self.OFFSET_GPO_EN,       width=1,                     bit_value=gpo_en)
		self.set_bits(addr=addr, offset=self.OFFSET_GF_EN,        width=1,                     bit_value=gf_en)
	
	def write_vout_gain(self, channel=0, enc_vout_gain=0):
		if (channel == 0):
			addr = self.ADDR_DAC_0_VOUT_CMP_CONFIG
		elif (channel == 1):
			addr = self.ADDR_DAC_1_VOUT_CMP_CONFIG
		elif (channel == 2):
			addr = self.ADDR_DAC_2_VOUT_CMP_CONFIG
		elif (channel == 3):
			addr = self.ADDR_DAC_3_VOUT_CMP_CONFIG
		self.set_bits(addr=addr, offset=self.OFFSET_VOUT_GAIN, width=self.WIDTH_VOUT_GAIN, bit_value=enc_vout_gain)
	
	def start_function(self, channel=0):
		if (channel == 0):
			offset = self.OFFSET_START_FUNC_0
		elif (channel == 1):
			offset = self.OFFSET_START_FUNC_1
		elif (channel == 2):
			offset = self.OFFSET_START_FUNC_2
		elif (channel == 3):
			offset = self.OFFSET_START_FUNC_3
		self.set_bits(addr=self.ADDR_COMMON_DAC_TRIG, offset=offset, width=1, bit_value=1)
	
	def stop_function(self, channel=0):
		if (channel == 0):
			offset = self.OFFSET_START_FUNC_0
		elif (channel == 1):
			offset = self.OFFSET_START_FUNC_1
		elif (channel == 2):
			offset = self.OFFSET_START_FUNC_2
		elif (channel == 3):
			offset = self.OFFSET_START_FUNC_3
		self.set_bits(addr=self.ADDR_COMMON_DAC_TRIG, offset=offset, width=1, bit_value=0)
