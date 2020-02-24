from __future__ import absolute_import, division, print_function, unicode_literals 
from math import sin, cos,radians,atan,degrees 
import pi3d 
import RPi.GPIO as gpio 
import i2c 
import time

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(26, gpio.IN, pull_up_down=gpio.PUD_DOWN)

bus = i2c.I2C(2)


try:
    bus.read(1, 0x5c)
    bus.write([0x6e, 0b00001110], 0x5c)
    bus.write([0x70, 0b00000000], 0x5c)
except:
    print('Error: no touchscreen found')
    

def touch():

            try:
                time.sleep(0.001)
                data = bus.rdwr([0x40], 8, 0x5C)
                x1 = 400 - (data[0] | (data[4] << 8))
                y1 = (data[1] | (data[5] << 8)) - 240
                if y1 == 0:
                    y1 = 1
                if ((-401 < x1 < 401) & (-241 < y1 < 241)):
                        #print(x1,y1)
                        return x1, y1  # compensate position to match with PI3D
                else:
                        time.sleep(0.01)
                        return touch()

            except:
                time.sleep(0.05)
                return touch()



DISPLAY = pi3d.Display.create(layer=0, w=800, h=480, background=(
    0.0, 0.0, 0.0, 1.0), frames_per_second=60, tk=False   , samples=4)
CAMERA = pi3d.Camera(is_3d=False)

MATSH = pi3d.Shader("mat_flat")

shape = []
shape2 = []


for x in range(-135,135):
   if x % 3 == 0:
    shape2.append((200* sin(radians(x)), 200* cos(radians(x)), 2))
   if x % 5 == 0:
    shape.append((180* sin(radians(x)) , 180 * cos(radians(x)), 2))
    shape.append((220* sin(radians(x)) , 220 * cos(radians(x)), 2))


line = pi3d.Lines(vertices=shape,camera=CAMERA, line_width=5, strip=False)
line.set_shader(MATSH)
line.set_material((1, 1, 1))
line.set_alpha(0.8)

line2 = pi3d.Lines(vertices=shape2,camera=CAMERA, line_width=25, strip=True)
line2.set_shader(MATSH)
line2.set_material((1, 1, 1))
line2.set_alpha(0.2)

x2 = 400
y2 = -100
SHADER = pi3d.Shader("uv_flat")
actval= pi3d.FixedString('opensans.ttf', '0', font_size=30,shadow_radius=0, 
                        background_color=(0,0,0,0), color= (255,255,255,255),
                        camera=CAMERA, shader=SHADER, f_type='SMOOTH')

while DISPLAY.loop_running():





 if (gpio.input(26)):
  x2, y2 = touch()


 #line3 = pi3d.Lines(vertices=[(0,0,2),(x2,y2,2)], line_width=2, strip=True)
 degree = int(degrees(atan(x2/y2)))

 if y2 < 0:
  degree += 180
  if x2 < 0:
   degree -= 360

 
 if degree > 135:  degree = 135
 if degree < -135: degree = -135

 actval = (degree + 135) / 2.7
 rgbval = round((degree + 135) / 270,2)
 
 
 line = pi3d.Lines(vertices=shape[0:((degree+138)*2)//5],camera=CAMERA, line_width=5, strip=False)
 line.set_shader(MATSH)
 line.set_material((rgbval, 0, 1-rgbval))
 line.set_alpha(0.8)

 line2 = pi3d.Lines(vertices=shape2[(degree+134)//3:],camera=CAMERA, line_width=5, strip=True)
 line2.set_shader(MATSH)
 line2.set_material((1, 1, 1))
 line2.set_alpha(0.2)

 actval= pi3d.FixedString('opensans.ttf', str(int(actval))+'%', font_size=65,shadow_radius=0, 
                        background_color=(0,0,0,0), color= (255,255,255,255),
                        camera=CAMERA, shader=SHADER, f_type='SMOOTH')
 actval.sprite.position(0, 0, 1)


 x1 = 200* sin(radians(degree))
 y1 = 200* cos(radians(degree))

 dot2= pi3d.Disk(radius=8, sides=20, z=0.1, rx=90,x=x1, y=y1, camera=CAMERA)
 dot2.set_shader(MATSH)
 dot2.set_material((rgbval, 0, 1-rgbval))
 dot2.set_alpha(1)


 
 line.draw() 
 dot2.draw()
 line2.draw()
 #line3.draw()
 actval.draw()
DISPLAY.destroy()
