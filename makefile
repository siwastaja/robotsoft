#This makefile is written for Raspberry cross-compiler downloadable at
#https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/GCC%209.1.0/Raspberry%20Pi%203A%2B%2C%203B%2B/

# For your specific compile-time configuration, take a copy of the revision-controlled config.h.template as config.h, and make
# your modifications. Then, just
# make
# and run:
# ./robotsoft

#DEVIP = 10.3.0.6
#DEVIP = 192.168.43.59
DEVIP = 10.42.0.231
#DEVIP = 192.168.1.6

CC = arm-linux-gnueabihf-gcc
LD = arm-linux-gnueabihf-gcc

EXTRA_HEADERS = config.h api_board_to_soft.h api_soft_to_board.h api_tcp.h misc.h datatypes.h
ROBOTSOFT_OBJ = main.o tcp_comm.o tcp_parser.o spi.o b2s_prints.o slam_matchers.o slam_cloud.o slam_top.o voxmap.o voxmap_memdisk.o small_cloud.o
BOARDMON_OBJ = boardmon.o spi.o b2s_prints.o
CALIBRATOR_OBJ = ../robotsoft-calibrator/calibrator.o spi.o b2s_prints.o

CFLAGS = -I. -I../raspilibs/usr/include -Wall -Winline -std=c99 -DROBOTSOFT -D_XOPEN_SOURCE=700

CFLAGS += -g
CFLAGS += -O3 -march=armv8-a -mfloat-abi=hard -mfpu=neon-fp-armv8

LDFLAGS = -L/home/hrst/cross-pi-gcc-9.1.0-2/arm-linux-gnueabihf/libc/lib -L../raspilibs/lib/arm-linux-gnueabihf -L../raspilibs/usr/lib/arm-linux-gnueabihf

all: robotsoft

%.o: %.c
	$(CC) -c $(CFLAGS) $*.c -o $*.o
	$(CC) -MM $(CFLAGS) $*.c > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

robotsoft: $(ROBOTSOFT_OBJ)
	$(LD) $(LDFLAGS) -o robotsoft $^ -lm -lz -pthread
	scp robotsoft pulu@$(DEVIP):~/robotsoft

boardmon: $(BOARDMON_OBJ)
	$(LD) $(LDFLAGS) -o boardmon $^ -lm -pthread
	scp boardmon pulu@$(DEVIP):~/robotsoft

calibrator: $(CALIBRATOR_OBJ)
	CFLAGS += -DCALIBRATOR
	$(LD) $(LDFLAGS) -o calibrator $^ -lm -pthread
	scp calibrator pulu@$(DEVIP):~/robotsoft

e:
	gedit --new-window makefile $(EXTRA_HEADERS) `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_boardmon:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_calibrator:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h ../robotsoft-calibrator/calibproc.c `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.h"/g` &

clean:
	rm *.o
	rm ../robotsoft-calibrator/*.o
