# Copyright (c) 2024 - 2025 Kevin G. Schlosser

# this driver uses a special i2c bus implimentation I have written.
# This implimentation takes into consideration the ESP32 and it having
# threading available. It also has some convience methods built into it
# that figure out what is wanting to be done automatically.
# read more about it's use in the stub files.

from micropython import const  # NOQA
import pointer_framework
import machine  # NOQA
import time


_CMD_REG = const(0x8040)
_CMD_CHECK_REG = const(0x8046)
_CMD_READ_DATA = const(0x01)

_ESD_CHECK_REG = const(0x8041)

_STATUS_REG = const(0x814E)
_POINTS_REG = const(0x814F)
_POINT_RECORD_SIZE = const(8)
_MAX_TOUCH_POINTS = const(5)

_PRODUCT_ID_REG = const(0x8140)
_FIRMWARE_VERSION_REG = const(0x8144)
_VENDOR_ID_REG = const(0x814A)

_X_CORD_RES_REG = const(0x8146)
_Y_CORD_RES_REG = const(0x8148)

I2C_ADDR = 0x5D
BITS = 16

_ADDR2 = const(0x14)


class GT911(pointer_framework.PointerDriver):

    def _read_reg(self, reg, num_bytes=None, buf=None):
        self._tx_buf[0] = reg >> 8
        self._tx_buf[1] = reg & 0xFF
        if num_bytes is not None:
            self._device.write_readinto(self._tx_mv[:2], self._rx_mv[:num_bytes])
        else:
            self._device.write_readinto(self._tx_mv[:2], buf)

    def _write_reg(self, reg, value=None, buf=None):
        if value is not None:
            self._tx_buf[0] = value
            self._device.write_mem(reg, self._tx_mv[:1])
        elif buf is not None:
            self._device.write_mem(reg, buf)

    def __init__(
        self,
        device,
        reset_pin=None,
        interrupt_pin=None,
        touch_cal=None,
        startup_rotation=pointer_framework.lv.DISPLAY_ROTATION._0,  # NOQA
        debug=False
    ):
        self._tx_buf = bytearray(3)
        self._tx_mv = memoryview(self._tx_buf)
        self._rx_buf = bytearray(_POINT_RECORD_SIZE * _MAX_TOUCH_POINTS)
        self._rx_mv = memoryview(self._rx_buf)

        self._device = device

        self.__x = 0
        self.__y = 0
        self.__touch_points = ()
        self.__last_state = self.RELEASED

        if isinstance(reset_pin, int):
            reset_pin = machine.Pin(reset_pin, machine.Pin.OUT)

        if isinstance(interrupt_pin, int):
            interrupt_pin = machine.Pin(interrupt_pin, machine.Pin.OUT)

        self._reset_pin = reset_pin
        self._interrupt_pin = interrupt_pin

        self.hw_reset()
        super().__init__(
            touch_cal=touch_cal, startup_rotation=startup_rotation, debug=debug
        )

    def hw_reset(self):
        if self._interrupt_pin and self._reset_pin:
            self._interrupt_pin.init(self._interrupt_pin.OUT)
            self._interrupt_pin(0)
            self._reset_pin(0)
            time.sleep_ms(10)  # NOQA
            self._interrupt_pin(0)
            time.sleep_ms(1)  # NOQA
            self._reset_pin(1)
            time.sleep_ms(5)  # NOQA
            self._interrupt_pin(0)
            time.sleep_ms(50)  # NOQA
            self._interrupt_pin.init(self._interrupt_pin.IN)
            time.sleep_ms(50)  # NOQA

        self._write_reg(_ESD_CHECK_REG, 0x00)
        self._write_reg(_CMD_CHECK_REG, _CMD_READ_DATA)
        self._write_reg(_CMD_REG, _CMD_READ_DATA)

        self._read_reg(_PRODUCT_ID_REG, 4)

        product_id = ''
        for item in self._rx_buf[:4]:
            try:
                product_id += chr(item)
            except:  # NOQA
                break

        print('Touch Product id:', product_id)

        self._read_reg(_FIRMWARE_VERSION_REG, 2)
        print(
            'Touch Firmware version:',
            hex(self._rx_buf[0] + (self._rx_buf[1] << 8))
            )

        self._read_reg(_VENDOR_ID_REG, 1)
        print(f'Touch Vendor id: 0x{hex(self._rx_buf[0])[2:].upper()}')
        x, y = self.hw_size
        print(f'Touch resolution: width={x}, height={y}')

    @property
    def hw_size(self):
        self._read_reg(_X_CORD_RES_REG, 2)
        x = self._rx_buf[0] + (self._rx_buf[1] << 8)

        self._read_reg(_Y_CORD_RES_REG, 2)
        y = self._rx_buf[0] + (self._rx_buf[1] << 8)

        return x, y

    @property
    def firmware_config(self):
        try:
            import gt911_extension
        except ImportError:
            raise ImportError(
                'you need to upload the gt911_extension.py file to the MCU'
            )
        return gt911_extension.GT911Extension(self, self._device)

    def get_touch_points(self):
        """Return the latest raw contacts as (track_id, x, y, strength)."""
        return self.__touch_points

    def _get_coords(self):
        self._read_reg(_STATUS_REG, 1)
        touch_cnt = self._rx_buf[0] & 0x0F
        status = self._rx_buf[0] & 0x80

        if status:
            if 0 < touch_cnt <= _MAX_TOUCH_POINTS:
                self._read_reg(
                    _POINTS_REG,
                    touch_cnt * _POINT_RECORD_SIZE
                )
                points = []
                for index in range(touch_cnt):
                    offset = index * _POINT_RECORD_SIZE
                    track_id = self._rx_buf[offset]
                    x = (
                        self._rx_buf[offset + 1] |
                        (self._rx_buf[offset + 2] << 8)
                    )
                    y = (
                        self._rx_buf[offset + 3] |
                        (self._rx_buf[offset + 4] << 8)
                    )
                    strength = (
                        self._rx_buf[offset + 5] |
                        (self._rx_buf[offset + 6] << 8)
                    )
                    points.append((track_id, x, y, strength))

                self.__touch_points = tuple(points)
                self.__x = points[0][1]
                self.__y = points[0][2]
                self.__last_state = self.PRESSED
            else:
                self.__touch_points = ()
                self.__last_state = self.RELEASED

            self._write_reg(_STATUS_REG, 0x00)

        return self.__last_state, self.__x, self.__y
