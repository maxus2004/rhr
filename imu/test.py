import spidev
spi = spidev.SpiDev()
spi.open(0,0) # Open SPI bus 0, device (CS) 0, here based on the CS pin you choose, the value of device changes between 0 and 1
spi.mode = 0b11
spi.max_speed_hz = 5000

def read_register(spi, register):
  response = spi.xfer2([register | 0x80, 0x00])
  return response[1]

def write_register(spi, register, value):
  spi.xfer2([register, value])

while(True):
    who_am_i = read_register(spi, 0x0F)
    print("WHO_AM_I:", who_am_i)