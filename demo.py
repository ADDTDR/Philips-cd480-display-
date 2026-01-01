import smbus
import time
from font import Font5x7_full, Font5x7_180
from datetime import datetime

CONF_REG = 0x00
PWM_REG = 0x19
PWM_CYCLE = 96
COLUMN_UPDATE_REG = 0x0C
RESET_REG = 0xFF
LIGHT_EFFECT_REG = 0x0D

LED_DRIVER_ADDRESS = 0x60
LED_DRIVER2_ADDRESS = 0x61

# CONFIGURE matdrix mode, SSD, DM, A_EN, ADM
CONFIG_DATA = 0b00011000

# SET  intensity control settings
LIGHT_CONFIG = 0b00001110
# Initialize bus
bus = smbus.SMBus(1)


def driver_init(bus, led_driver_address, config_data, light_config):
    # Configurate led driver mode
    bus.write_byte_data(led_driver_address, CONF_REG, config_data)

    # PWM cycle
    bus.write_byte_data(led_driver_address, PWM_REG, PWM_CYCLE)

    # Wite display data
    # bus.write_i2c_block_data(LED_DRIVER_ADDRESS, 0x0E, [0x00] * 10)

    # Set intensity config
    bus.write_byte_data(led_driver_address, LIGHT_EFFECT_REG, light_config)

    # Update display with data

    # bus.write_byte_data(LED_DRIVER_ADDRESS, 0x0C, 0x00)


def driver_print_chars(bus, led_driver_address, chars, dp=False):
    buffer = [0x00] * 8
    buffer2 = [0x00] * 8
    leter = Font5x7_full[ord(chars[0]) - 0x20]
    buffer[0] = 0b01000000  if dp == True else 0
    buffer[1] = leter[0]
    buffer[2] = leter[1]
    buffer[3] = leter[2]
    buffer[4] = leter[3]
    buffer[5] = leter[4]
    letter_2 = Font5x7_180[ord(chars[1]) - 0x20]

    buffer2[0] = letter_2[6] << 1
    buffer2[1] = letter_2[5] << 1
    buffer2[2] = letter_2[4] << 1
    buffer2[3] = letter_2[3] << 1
    buffer2[4] = letter_2[2] << 1
    buffer2[5] = letter_2[1] << 1
    buffer2[6] = letter_2[0] << 1

    bus.write_i2c_block_data(led_driver_address, 0x01, buffer2)
    bus.write_i2c_block_data(led_driver_address, 0x0E, buffer)
    bus.write_byte_data(led_driver_address, 0x0C, 0x00)


driver_init(bus, LED_DRIVER_ADDRESS, CONFIG_DATA, LIGHT_CONFIG)
driver_init(bus, LED_DRIVER2_ADDRESS, CONFIG_DATA, LIGHT_CONFIG)


str_time_now = ' ' * 4
decimal_point = True
while True:
    str_time_now = datetime.now().strftime('%H%M')
    decimal_point = ~ decimal_point
    driver_print_chars(bus, LED_DRIVER_ADDRESS, str_time_now[:2])
    driver_print_chars(bus, LED_DRIVER2_ADDRESS, str_time_now[2:4], decimal_point)

    time.sleep(0.125)

if __name__ == "__main__":
    pass
