
cd C:\VSARM\picoprobe\openocd-branch-rp2040-f8w14ec-windows\tcl

C:\VSARM\picoprobe\openocd-branch-rp2040-f8w14ec-windows\src\openocd.exe -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program \"C:/VSARM/sdk/pico/pico-sdk/lib/tinyusb/examples/host/msc_with_fatfs/build/msc_with_fatfs.elf\" verify reset exit"

cd C:\VSARM\sdk\pico\pico-sdk\lib\tinyusb\examples\host\msc_with_fatfs\build