#This makefile is written for Raspberry cross-compiler downloadable at
#https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/GCC%209.1.0/Raspberry%20Pi%203A%2B%2C%203B%2B/

# For your specific compile-time configuration, take a copy of the revision-controlled config.h.template as config.h, and make
# your modifications. Then, just
# make
# and run:
# ./robotsoft


DEVIP = 192.168.10.43

ifdef noncross
CC = gcc
LD = gcc
else
CC = arm-linux-gnueabihf-gcc
LD = arm-linux-gnueabihf-gcc
endif

EXTRA_HEADERS = config.h slam_config.h api_board_to_soft.h api_soft_to_board.h api_tcp.h misc.h datatypes.h statevect.h
ROBOTSOFT_OBJ = main.o tcp_comm.o tcp_parser.o spi.o b2s_prints.o slam_matchers.o slam_cloud.o slam_icp.o slam_top.o voxmap.o voxmap_memdisk.o small_cloud.o routing.o
BOARDMON_OBJ = boardmon.o spi.o b2s_prints.o
SPIPROG_OBJ = spiprog.o
CALIBRATOR_OBJ = ../robotsoft-calibrator/calibrator.o spi.o b2s_prints.o

CFLAGS = -O3 -I. -Wall -Winline -std=c99 -DROBOTSOFT -D_XOPEN_SOURCE=700 -Wunused-function -Wunused-variable

ifndef noncross
CFLAGS += -I../raspilibs/usr/include -I/home/hrst/pulu/raspilibs/usr/include/arm-linux-gnueabihf
CFLAGS += -march=armv8-a -mfloat-abi=hard -mfpu=neon-fp-armv8
endif

ifdef noncross
CFLAGS += -DDISABLE_SPI
endif

#CFLAGS += -g

# Application-specific defines
# VACUUM_APP adds ignore regions on the pointcloud for the nozzle, for correct mapping result
CFLAGS += -DVACUUM_APP

ifndef noncross
LDFLAGS = -L/home/hrst/cross-pi-gcc-9.2.0-2/arm-linux-gnueabihf/libc/lib -L../raspilibs/lib/arm-linux-gnueabihf -L../raspilibs/usr/lib/arm-linux-gnueabihf -L/home/hrst/pulu/raspilibs/usr/lib/arm-linux-gnueabihf
endif

all: robotsoft

$(ROBOTSOFT_OBJ): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BOARDMON_OBJ): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(SPIPROG_OBJ): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(CALIBRATOR_OBJ): %.o: %.c
	$(CC) -c -DCALIBRATOR $(CFLAGS) $< -o $@

robotsoft: $(ROBOTSOFT_OBJ)
	$(LD) $(LDFLAGS) -o robotsoft $^ -ldl -lm -lz -pthread
ifndef noncross
	scp robotsoft pulu@$(DEVIP):~/robotsoft
endif

boardmon: $(BOARDMON_OBJ)
	$(LD) $(LDFLAGS) -o boardmon $^ -ldl -lm -pthread
ifndef noncross
	scp boardmon pulu@$(DEVIP):~/robotsoft
endif

spiprog: $(SPIPROG_OBJ)
	$(LD) $(LDFLAGS) -o spiprog $^
ifndef noncross
	scp spiprog pulu@$(DEVIP):~/robotsoft
endif

calibrator: $(CALIBRATOR_OBJ)
	$(LD) $(LDFLAGS) -o calibrator $^ -ldl -lm -pthread
ifndef noncross
	scp calibrator pulu@$(DEVIP):~/robotsoft
endif

e:
	gedit --new-window makefile $(EXTRA_HEADERS) `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_boardmon:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_calibrator:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h ../robotsoft-calibrator/calibproc.c `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.h"/g` &

clean:
	-rm *.o
	-rm ../robotsoft-calibrator/*.o
