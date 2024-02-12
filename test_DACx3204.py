from DACx3204 import DACx3204
import time

dac = DACx3204(bus='i2c', debug=False) # remember to replug USB each time the bus is changed i2c <=> spi
expected_general_status = 0x2004
read_general_status = dac.read_word(read_addr=dac.ADDR_GENERAL_STATUS)
if (read_general_status != expected_general_status):
	print("Expected General Status register value 0x{:04x}, read value is 0x{:04x}, check your bus declaration or jumper J5".format())
dac.check_spi_read_sdo_en_bit()
#dac.set_sdo_en_bit_in_nvm()

