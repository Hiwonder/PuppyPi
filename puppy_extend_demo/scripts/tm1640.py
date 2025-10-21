import RPi.GPIO as GPIO
import time
TM1640_CMD1 = (64)  # 0x40 data command
TM1640_CMD2 = (192) # 0xC0 address command
TM1640_CMD3 = (128) # 0x80 display control command
TM1640_DSP_ON = (8) # 0x08 display on
TM1640_DELAY = (10) # 10us delay between clk/dio pulses

def sleep_us(t):
    time.sleep(t / 1000000)

class TM1640(object):
    """Library for LED matrix display modules based on the TM1640 LED driver."""
    def __init__(self, clk, dio, brightness=7):
        self.clk_p = clk
        self.dio_p = dio

        if not 0 <= brightness <= 7:
            raise ValueError("Brightness out of range")
        self._brightness = brightness
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(clk, GPIO.OUT)
        GPIO.output(clk, 0)
        GPIO.setup(dio, GPIO.OUT)
        GPIO.output(dio, 0)
        sleep_us(TM1640_DELAY)

        self._write_data_cmd()
        self._write_dsp_ctrl()

    def dio(self, n_s):
        GPIO.output(self.dio_p, n_s)

    def clk(self, n_s):
        GPIO.output(self.clk_p, n_s)

    def _start(self):
        self.dio(0)
        sleep_us(TM1640_DELAY)
        self.clk(0)
        sleep_us(TM1640_DELAY)

    def _stop(self):
        self.dio(0)
        sleep_us(TM1640_DELAY)
        self.clk(1)
        sleep_us(TM1640_DELAY)
        self.dio(1)

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
            self.dio((b >> i) & 1)
            sleep_us(TM1640_DELAY)
            self.clk(1)
            sleep_us(TM1640_DELAY)
            self.clk(0)
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


display = TM1640(dio=26, clk=24)

display_buf = [0] * 16

def set_bit(x, y, s):
    display_buf[x] = (display_buf[x] & (~(0x01 << y))) | (s << y)

def update_display():
    display.write(display_buf)

set_bit(1, 1, 0)
update_display()

