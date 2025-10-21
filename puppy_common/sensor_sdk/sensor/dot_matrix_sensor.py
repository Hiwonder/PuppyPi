import time
import gpiod
TM1640_CMD1 = (64)  # 0x40 data command
TM1640_CMD2 = (192) # 0xC0 address command
TM1640_CMD3 = (128) # 0x80 display control command
TM1640_DSP_ON = (8) # 0x08 display on
TM1640_DELAY = (10) # 10us delay between clk/dio pulses

def sleep_us(t):
    time.sleep(t / 1000000)

class TM1640(object):
    # 字模数据(font data)
    digit_dict = {'0': 0x3f,
                  '1': 0x06,
                  '2': 0x5b,
                  '3': 0x4f,
                  '4': 0x66,
                  '5': 0x6d,
                  '6': 0x7d,
                  '7': 0x07,
                  '8': 0x7f,
                  '9': 0x6f,
                  '.': 0x80,
                  '-': 0x40}
    """Library for LED matrix display modules based on the TM1640 LED driver."""
    def __init__(self, clk, dio, brightness=7):
        self.display_buf = [0] * 16

        if not 0 <= brightness <= 7:
            raise ValueError("Brightness out of range")
        self._brightness = brightness

        chip = gpiod.chip("gpiochip0")
        self.clk = chip.get_line(clk)
        self.dio = chip.get_line(dio)
    	
        config = gpiod.line_request()
        config.consumer = "clk"
        config.request_type = gpiod.line_request.DIRECTION_OUTPUT
        self.clk.request(config)
        
        config = gpiod.line_request()
        config.consumer = "dio"
        config.request_type = gpiod.line_request.DIRECTION_OUTPUT
        self.dio.request(config)

        self.clk.set_value(0)
        self.dio.set_value(0)
        sleep_us(TM1640_DELAY)

        self._write_data_cmd()
        self._write_dsp_ctrl()

    def _start(self):
        self.dio.set_value(0)
        sleep_us(TM1640_DELAY)
        self.clk.set_value(0)
        sleep_us(TM1640_DELAY)

    def _stop(self):
        self.dio.set_value(0)
        sleep_us(TM1640_DELAY)
        self.clk.set_value(1)
        sleep_us(TM1640_DELAY)
        self.dio.set_value(1)

    def _write_data_cmd(self):
        # automatic address increment, normal mode
        self._start()
        self._write_byte(TM1640_CMD1)
        self._stop()

    def _write_dsp_ctrl(self):
        # display on, set brightness
        self._start()
        self._write_byte(TM1640_CMD3 | TM1640_DSP_ON | self._brightness)
        self._stop()

    def _write_byte(self, b):
        for i in range(8):
            self.dio.set_value((b >> i) & 1)
            sleep_us(TM1640_DELAY)
            self.clk.set_value(1)
            sleep_us(TM1640_DELAY)
            self.clk.set_value(0)
            sleep_us(TM1640_DELAY)

    def brightness(self, val=None):
        """Set the display brightness 0-7."""
        # brightness 0 = 1/16th pulse width
        # brightness 7 = 14/16th pulse width
        if val is None:
            return self._brightness
        if not 0 <= val <= 7:
            raise ValueError("Brightness out of range")

        self._brightness = val
        self._write_data_cmd()
        self._write_dsp_ctrl()

    def write(self, rows, pos=0):
        if not 0 <= pos <= 7:
            raise ValueError("Position out of range")

        self._write_data_cmd()
        self._start()

        self._write_byte(TM1640_CMD2 | pos)
        for row in rows:
            self._write_byte(row)

        self._stop()
        self._write_dsp_ctrl()

    def write_int(self, int, pos=0, len=8):
        self.write(int.to_bytes(len, 'big'), pos)

    def write_hmsb(self, buf, pos=0):
        self._write_data_cmd()
        self._start()

        self._write_byte(TM1640_CMD2 | pos)
        for i in range(7-pos, -1, -1):
            self._write_byte(buf[i])

        self._stop()
        self._write_dsp_ctrl()

    def clear(self):
        """熄灭点阵所有led(turn off all LEDs in the matrix)"""
        self.display_buf = [0] * 16
        self.update_display()

    def set_bit(self, x, y, s):
        """设置(x, y)处的led状态，0或者1，0表示熄灭(set the LED status at (x, y) to 0 or 1, where 0 represents turning off the LED)"""
        self.display_buf[x] = (self.display_buf[x] & (~(0x01 << y))) | (s << y)

    def set_number(self, number):
        """数码管显示数字(display a number on the digital tube)"""
        self.display_buf = [self.digit_dict['0']] * 4
        num = list(str(number))
        num.reverse()
        # num = list(str(number)).reverse()
        for i in range(len(num)):
            self.display_buf[-i-1] = self.digit_dict[num[i]]
            if num[i] == '.':
                self.display_buf[-i-2] = self.digit_dict[num[i - 1]] + self.digit_dict['.']
    
    def set_buf_horizontal(self, buf):
        """设置显示的二进制列表，横向(set the display list horizontally for binary representation)"""
        l = zip(*buf[::-1])
        self.display_buf = [int(''.join(i), 2) for i in l]
    
    def set_buf_vertical(self, buf):
        """设置显示的二进制列表，竖向(set the display list vertically for binary representation)"""
        self.display_buf = [int(i, 2) for i in buf]

    def update_display(self):
        """刷新显示(refresh display)"""
        self.write(self.display_buf)

if __name__ == "__main__":
    display = TM1640(dio=7, clk=8)
    while True:
        display.set_bit(0, 0, 1)
        display.update_display()
        time.sleep(0.5)
        display.set_bit(0, 0, 0)
        display.update_display()
        time.sleep(0.5)
