from __future__ import absolute_import, division, print_function, unicode_literals
from math import sin, cos, radians, atan2, degrees
import pi3d
import RPi.GPIO as gpio
import i2c
import time

class Dial(object):
    def __init__(self, angle_fr=-135, angle_to=135, step=5, outer=240, inner=200,
                min_t=15, max_t=35, shader=None, camera=None):

        self.angle_fr = angle_fr
        self.angle_to = angle_to
        self.step = step
        self.outer = outer
        self.inner = inner
        self.mid = (outer + inner) / 2
        self.min_t = min_t
        self.max_t = max_t

        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(26, gpio.IN, pull_up_down=gpio.PUD_DOWN)

        self.bus = i2c.I2C(2)

        try:
            self.bus.read(1, 0x5c)
            self.bus.write([0x6e, 0b00001110], 0x5c)
            self.bus.write([0x70, 0b00000000], 0x5c)
        except:
            print('Error: no touchscreen found')

        solid_verts = []
        tick_verts = []

        for x in range(self.angle_fr, self.angle_to, self.step):
            (s, c) = (sin(radians(x)), cos(radians(x))) # re-use for brevity below
            solid_verts.append((self.mid * s, self.mid * c, 2))
            tick_verts.append((self.inner * s, self.inner * c, 0.1))
            tick_verts.append((self.outer * s, self.outer * c, 0.1))

        if shader is None:
            shader = pi3d.Shader('mat_flat')
        if camera is None:
            camera = pi3d.Camera(is_3d=False)

        self.ticks = pi3d.Lines(vertices=tick_verts, camera=camera, line_width=5, strip=False)
        self.ticks.set_shader(shader)
        self.ticks.set_material((1, 1, 1))
        self.ticks.set_alpha(0.8)

        self.bline = pi3d.Lines(vertices=solid_verts, camera=camera, line_width=40, strip=True)
        self.bline.set_shader(shader)
        self.bline.set_material((0, 1, 0))
        self.bline.set_alpha(0.4)

        self.dial = pi3d.Lines(vertices=solid_verts, camera=camera, line_width=5, strip=True)
        self.dial.set_shader(shader)
        self.dial.set_material((1, 1, 1))
        self.dial.set_alpha(0.2)

        font = pi3d.Font('opensans.ttf', codepoints='0123456789.-°', grid_size=5)
        self.actval = pi3d.PointText(font, camera, max_chars=10, point_size=100) 
        self.temp_block = pi3d.TextBlock(0, 0, 0.1, 0.0, 6, justify=0.5, text_format="0°", size=0.79,
                    spacing="F", space=0.02, colour=(1.0, 1.0, 1.0, 1.0))
        self.actval.add_text_block(self.temp_block)

        self.dot2= pi3d.Disk(radius=20, sides=20, z=0.1, rx=90, camera=camera)
        self.dot2.set_shader(shader)
        self.dot2.set_material((1, 1, 1))
        self.dot2_alpha = 1.0

        self.x1, self.y1 = -5, 10
        self.set_flag = True
        self.value = 20.0

    def touch(self):
        try:
            time.sleep(0.001)
            data = self.bus.rdwr([0x40], 8, 0x5C)
            x1 = 400 - (data[0] | (data[4] << 8))
            y1 = (data[1] | (data[5] << 8)) - 240
            if y1 == 0:
                y1 = 1
            if ((-401 < x1 < 401) & (-241 < y1 < 241)):
                return x1, y1  # compensate position to match with PI3D
            else:
                time.sleep(0.01)
                return self.touch()
        except:
            time.sleep(0.05)
            return self.touch()

    def check_touch(self):
        if self.set_flag or gpio.input(26):
            if self.set_flag:
                x3, y3 = self.x1, self.y1
                self.x1 *= 0.9 # i.e. just inside but at the same angle
                self.y1 *= 0.9
            else:
                x3, y3 = self.touch()
            if ((self.x1 - 80) < x3 and x3  < (self.x1 + 80) and
                (self.y1 - 80) < y3 and y3  < (self.y1 + 80)):
                self.x1, self.y1 = x3, y3
                degree = int(degrees(atan2(self.x1, self.y1)))
                if degree < self.angle_fr:
                    degree = self.angle_fr
                if degree > self.angle_to:
                    degree = self.angle_to

                self.value = (self.min_t + (degree - self.angle_fr)
                             / (self.angle_to - self.angle_fr) * (self.max_t - self.min_t))
                rgbval = round((degree - self.angle_fr) / (self.angle_to - self.angle_fr), 2) # rgbval 0.0 - 1.0

                ## move vertices z value for shapes
                for (line_shape, z_first, z_second, rgb) in (
                        (self.ticks, 1.0, -1.0, (rgbval, 0.0, 1.0 - rgbval)),
                        (self.dial, -1.0, 0.1, (1.0, 1.0, 1.0)),
                        (self.bline, 2.0, -1.0, (0.0, 1.0, 0.0))):
                    b = line_shape.buf[0]
                    v = b.array_buffer
                    cut_n = int((degree - self.angle_fr) / (self.angle_to - self.angle_fr) * len(v))
                    if cut_n >= len(v):
                        cut_n = len(v) - 1
                    v[:cut_n:,2] = z_first  # show points by moving away
                    v[cut_n:, 2] = z_second # hide points by moving behind near plane!
                    b.re_init()
                    b.set_material(rgb)

                self.temp_block.set_text(text_format="{:4.1f}°".format(self.value))
                self.actval.regen()

                self.x1 = self.mid * sin(radians(degree))
                self.y1 = self.mid * cos(radians(degree))

                self.dot2.position(self.x1, self.y1, 0.5)
                self.dot2_alpha = 1.0
                self.bline.draw()
            self.set_flag = False

    def draw(self):
        if self.dot2_alpha > 0.0:
            self.dot2_alpha -= 0.003
            self.dot2.set_alpha(self.dot2_alpha)
            self.dot2.draw()
        self.ticks.draw()
        self.dial.draw()
        self.actval.draw()

    def get_value(self):
        return self.value

    def set_value(self, value):
        self.value = value
        degree = (self.angle_fr +  (self.angle_to - self.angle_fr) * (self.value - self.min_t)
                                                            / (self.max_t - self.min_t))
        self.x1 = self.mid * sin(radians(degree))
        self.y1 = self.mid * cos(radians(degree))
        self.set_flag = True

DISPLAY = pi3d.Display.create(layer=0, w=800, h=480, background=(0.0, 0.0, 0.0, 1.0),
                              frames_per_second=60, tk=False, samples=4)
#CAMERA = pi3d.Camera(is_3d=False) # will create its own cam if one not passed

dial = Dial()

i = 0
while DISPLAY.loop_running():
    dial.check_touch()
    dial.draw()

    i += 1
    if i % 1000 == 0:
        dial.set_value(25.0)
    if i % 250 == 0:
        print(dial.get_value())

DISPLAY.destroy()
