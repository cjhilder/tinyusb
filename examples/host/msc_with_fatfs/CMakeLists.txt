
cmake_minimum_required(VERSION 3.12) 

include(pico_sdk_import.cmake)

project(msc_with_fatfs C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

# The GCC 10 release series differs from previous GCC releases in a number of ways.
# A common mistake in C is omitting extern when declaring a global variable in a header file. If the header is 
# included by several files it results in multiple definitions of the same variable. In previous GCC versions 
# this error is ignored. GCC 10 defaults to -fno-common, which means a linker error will now be reported. 
# To fix this, use extern in header files when declaring global variables, and ensure each global is defined 
# in exactly one C file. As a workaround, legacy C code can be compiled with -fcommon.
add_compile_options(-fcommon -fsanitize=leak)

add_executable(msc_with_fatfs 
			
                msc_with_fatfs.c
				hid_app.c
				msc_app.c
				cdc_app.c
				FATFs/diskio.c
				FATFs/ff.c
				FATFs/ffunicode.c
				)

target_include_directories(msc_with_fatfs PUBLIC
				${CMAKE_CURRENT_LIST_DIR})
		
target_link_libraries(
					msc_with_fatfs
					pico_stdlib 
					tinyusb_host
)

pico_set_binary_type(msc_with_fatfs copy_to_ram)

pico_enable_stdio_usb(msc_with_fatfs 0)
pico_enable_stdio_uart(msc_with_fatfs 1)
#create files beyond the executable (.uf2, .hex, .bin, .map, .dis) using an SDK provided function
pico_add_extra_outputs(msc_with_fatfs)