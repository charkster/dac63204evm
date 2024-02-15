from DACx3204 import DACx3204
import time
from matplotlib import pyplot as plt

dac = DACx3204(bus='i2c', debug=False) # remember to replug USB each time the bus is changed i2c <=> spi
expected_general_status = 0x2004
read_general_status = dac.read_word(read_addr=dac.ADDR_GENERAL_STATUS)
if (read_general_status != expected_general_status):
	print("Expected General Status register value 0x{:04x}, read value is 0x{:04x}, check your bus declaration or jumper J5".format(expected_general_status,read_general_status))
dac.check_spi_read_sdo_en_bit()
#dac.set_sdo_en_bit_in_nvm()
dac.device_unlock()
dac.write_margin_high(channel=0, margin_val=0xFFF)
dac.write_margin_low(channel=0, margin_val=0x3FF)
dac.write_common_config_1(channel=0, enc_vout_pdn=dac.ENC_VOUT_PDN_PWR_UP) # this is very important to remember
dac.write_func_config(channel=0, enc_slew_rate=dac.ENC_SLEW_RATE_239US, enc_code_step=dac.ENC_CODE_STEP_1LSB, enc_func_config=dac.ENC_FUNC_CONFIG_SAWTOOTH)
dac.start_function(channel=0)
sample_list = []
for x in range(0,4000):
	sample_list.append(dac.read_word(read_addr=dac.ADDR_DAC_0_DATA))
print("DAC_0_MARGIN_HIGH is 0x{:04x}".format(dac.read_word(read_addr=dac.ADDR_DAC_0_MARGIN_HIGH)))
print("DAC_0_MARGIN_LOW is 0x{:04x}".format(dac.read_word(read_addr=dac.ADDR_DAC_0_MARGIN_LOW)))
print("DAC_0_FUNC_CONFIG register is 0x{:04x}".format(dac.read_word(read_addr=dac.ADDR_DAC_0_FUNC_CONFIG)))
print("COMMON_DAC_TRIG register is 0x{:04x}".format(dac.read_word(read_addr=dac.ADDR_COMMON_DAC_TRIG)))
print("COMMON_CONFIG register is 0x{:04x}".format(dac.read_word(read_addr=dac.ADDR_COMMON_CONFIG)))
#time.sleep(5)
dac.stop_function(channel=0)
plt.plot(sample_list)
plt.title('Sawtooth, Slew 239us, 1 LSB step')
plt.show()
