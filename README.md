# ST VD55G0 Linux driver

## Installation

### Driver

Compile the driver using the Makefile

```
make
```

Place it in the kernel modules folder.

```
sudo cp vd55g0.ko /lib/modules/$(uname -r)
```

Resolve modules dependencies.

```
sudo depmod -a
```

### Device tree

Compile the device tree overlay matching your platform and plugin board from the `dts` folder.

```
sudo dtc <device-tree>.dts -o /boot/firmware/overlays/<device-tree>.dtbo
```

Set the device tree overlay in your platform. This may differ from platform to platform. Please refer to your platform documentation.

This is how to set the device tree overlay for Raspberry Pi.

```
echo "dtoverlay=<device-tree>" | sudo tee -a /boot/firmware/config.txt
```

Finally, run `sudo reboot` to test your changes.


## Usage

Run any video capture application to stream from the sensor.
QV4L2 testbench is a well featured test application.

```
sudo apt install qv4l2
qv4l2
```


## Features

Framesizes :

```
$ v4l2-ctl --list-framesizes=GREY
ioctl: VIDIOC_ENUM_FRAMESIZES
	Size: Discrete 644x604
	Size: Discrete 640x600
	Size: Discrete 640x480
	Size: Discrete 320x240
```

Controls :

```
$ v4l2-ctl -L

User Controls

                       exposure 0x00980911 (int)    : min=0 max=1796 step=1 default=200 value=200
                horizontal_flip 0x00980914 (bool)   : default=0 value=0
                  vertical_flip 0x00980915 (bool)   : default=0 value=0
         temperature_in_celsius 0x00981920 (int)    : min=-1024 max=1023 step=1 default=0 value=26 flags=read-only, volatile
      dark_calibration_pedestal 0x00981921 (int)    : min=0 max=255 step=1 default=64 value=64
                  vt_slave_mode 0x00981922 (bool)   : default=1 value=0 flags=grabbed

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=1 default=0 value=0 (Auto Mode)
				0: Auto Mode
				1: Manual Mode
             auto_exposure_bias 0x009a0913 (intmenu): min=0 max=12 default=6 value=6 (0 0x0)
				0: -3000 (0xfffffffffffff448)
				1: -2500 (0xfffffffffffff63c)
				2: -2000 (0xfffffffffffff830)
				3: -1500 (0xfffffffffffffa24)
				4: -1000 (0xfffffffffffffc18)
				5: -500 (0xfffffffffffffe0c)
				6: 0 (0x0)
				7: 500 (0x1f4)
				8: 1000 (0x3e8)
				9: 1500 (0x5dc)
				10: 2000 (0x7d0)
				11: 2500 (0x9c4)
				12: 3000 (0xbb8)
                        3a_lock 0x009a091b (bitmask): max=0x00000001 default=0x00000000 value=0

Flash Controls

                       led_mode 0x009c0901 (menu)   : min=0 max=1 default=0 value=0 (Off)
				0: Off
				1: Flash

Image Source Controls

              vertical_blanking 0x009e0901 (int)    : min=81 max=64935 step=1 default=1260 value=1260
            horizontal_blanking 0x009e0902 (int)    : min=560 max=560 step=1 default=560 value=560 flags=read-only
                  analogue_gain 0x009e0903 (int)    : min=0 max=24 step=1 default=0 value=0

Image Processing Controls

                 link_frequency 0x009f0901 (intmenu): min=0 max=0 default=0 value=0 (402000000 0x17f60880) flags=read-only
				0: 402000000 (0x17f60880)
                     pixel_rate 0x009f0902 (int64)  : min=1 max=2147483647 step=1 default=100500000 value=100500000 flags=read-only
                   test_pattern 0x009f0903 (menu)   : min=0 max=7 default=0 value=0 (Disabled)
				0: Disabled
				1: Solid
				2: Colorbar
				3: Gradbar
				4: Hgrey
				5: Vgrey
				6: Dgrey
				7: PN28
                   digital_gain 0x009f0905 (int)    : min=256 max=2048 step=1 default=256 value=256
