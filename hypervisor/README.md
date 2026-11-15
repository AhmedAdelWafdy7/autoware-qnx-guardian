# QNX Hypervisor on the Raspberry Pi 5

The single-board setup: QNX Hypervisor 8.0 on one Pi 5, with a QNX guest (the Checker) and,
next, a Linux guest (Autoware).

Status:
- **Built:** the host image with the QNX guest inside it.
- **Not yet done:** booting it on the Pi 5, the Linux guest, and guest networking.

## What is here

| File | What it does |
|---|---|
| `build.sh` | Builds the host image on the x86_64 build host |
| `host/hyp-additions.build` | Added to the Pi 5 BSP buildfile: `qvm`, `qvm-check`, vdevs, guests, start-up hook |
| `guests/qnx-guardian/qnx-guardian.qvmconf` | QNX guest: 512 MB, 1 vCPU on core 3, PL011 + virtio console |

The QNX BSPs are not stored here. `build.sh` unpacks them from the SDP at build time.

`build.sh` makes two changes to the BSP's `rpi5.build`:
- It adds `-Q enable` to the `startup-bcm2712-rpi5` line. On the Cortex-A76 this runs the host at EL2 with VHE.
- It appends `host/hyp-additions.build`.

The BSP start-up already runs `/scripts/board_startup.sh` if it exists. Our version of that script runs `qvm-check` and starts the guest.

## Build

Run this on an x86_64 Linux host. You need QNX SDP 8.0 and these Software Center packages:
- `com.qnx.qnx800.target.hypervisor.group` (3.6.0)
- `com.qnx.qnx800.bsp.hw.raspberrypi_bcm2712_rpi5` (0.4.0)
- `com.qnx.qnx800.bsp.hypervisor_guest_arm` (0.2.0)

```bash
cd hypervisor
./build.sh                  # QNX_ROOT defaults to ~/qnx800
# -> out/ifs-rpi5-hyp.bin (~78 MB) + .sha256
```

## Boot on the Pi 5 (TODO(verify): not tried on hardware yet)

This follows the QNX BSP 8.0 guide for the Pi 5
([copy binaries](https://www.qnx.com/developers/docs/BSP8.0/com.qnx.doc.bsp_raspberrypi.bcm2712.rpi5_8.0/topic/bsp_transfer.html),
[hardware setup](https://www.qnx.com/developers/docs/BSP8.0/com.qnx.doc.bsp_raspberrypi.bcm2712.rpi5_8.0/topic/bsp_hw.html),
[boot](https://www.qnx.com/developers/docs/BSP8.0/com.qnx.doc.bsp_raspberrypi.bcm2712.rpi5_8.0/topic/bsp_boot_board.html)).

1. Write Raspberry Pi OS (64-bit) to the microSD card with Raspberry Pi Imager. This gives the
   FAT boot partition with the firmware files QNX needs: `config.txt`, `bcm2712-rpi-5-b.dtb`,
   `bcm2712d0-rpi-5-b.dtb` and `overlays/`.
2. Copy the stock BSP image (`ifs-rpi5.bin`) and `out/ifs-rpi5-hyp.bin` to the boot partition.
3. Comment out `auto_initramfs=1` in `config.txt`, and add this at the end:
   ```
   [rpi5]
   enable_uart=1
   force_turbo=1
   kernel_address=0x8000000
   kernel=ifs-rpi5.bin
   ```
   `kernel_address=0x8000000` is required. The QNX Pi 5 image is linked to run at 128 MB;
   without this line the firmware loads it at its default low address and QNX crashes at once.
   On the board this shows up as a green/red LED reset loop with no console output.
4. Serial console: a Raspberry Pi Debug Probe on the board's 3-pin UART connector (J16),
   115200 8N1, no flow control. QNX's console is `/dev/ser10`, which is this UART. On macOS the
   probe is `/dev/cu.usbmodem*`. An adapter on GPIO14/15 (header pins 8/10) stays silent under
   QNX even though Linux uses those pins for its console.
   Use a proper USB-C PD power supply: a phone charger gives a blinking red power LED.
5. Power on. The Pi bootloader's own log (`RPi: BOOTSYS ...`) appears first, then
   `Welcome to QNX 8.0.0 on RaspberryPi5 !` and a `#` prompt. If even the bootloader lines are
   missing, the problem is the serial connection, not QNX.
6. When the stock image boots, change the last line to `kernel=ifs-rpi5-hyp.bin`. `qvm-check`
   should then say the system is set up for a hypervisor (exit 0), and the guest's start-up and
   console output appear on the same console. For a separate host shell, use SSH (the BSP image
   starts `sshd`).

If our image doesn't boot but the BSP's does, QNX's prebuilt Pi 5 Quick Start Target Image
(Software Center package `com.qnx.qnx800.quickstart.rpi5`) is a second known-good reference.

Things to check on the first boot:
- **The firmware enters the kernel at EL2.** If it doesn't, startup crashes with "Hypervisor support
  requested but CPU has no EL2 support". In that case, check the Pi firmware or armstub settings.
- **`qvm-check` exits 0.**
- **The guest shell prompt appears**, and `pidin` on the host shows `qvm`.

## Next

- **Guest-host networking:** `vdev virtio-net` on the guest, with `vdevpeer-net` in the host's
  io-sock.
- **Linux guest:** the meta-autoware image. It needs a kernel Image, an initrd or virtio-blk
  rootfs, and a `cmdline`. Then measure Autoware's RAM, CPU and cycle rates inside the guest.
- **Guardian and ROS 2 in the QNX guest image.**
- **vCPU priorities**, then isolation measurements with the Linux guest idle, stressed and
  panicking.
