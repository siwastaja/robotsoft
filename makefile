# For your specific compile-time configuration, take a copy of the revision-controlled config.h.template as config.h, and make
# your modifications. Then, just
# make
# and run:
# ./robotsoft

#DEVIP = 10.3.0.6
#DEVIP = 192.168.43.59
#DEVIP = 10.42.0.242
DEVIP = 192.168.1.6

CC = gcc
LD = gcc

EXTRA_HEADERS = config.h api_board_to_soft.h api_soft_to_board.h api_tcp.h misc.h datatypes.h
ROBOTSOFT_OBJ = main.o tcp_comm.o tcp_parser.o spi.o b2s_prints.o slam_matchers.o slam_cloud.o slam_top.o voxmap.o voxmap_memdisk.o
BOARDMON_OBJ = boardmon.o spi.o b2s_prints.o
CALIBRATOR_OBJ = ../robotsoft-calibrator/calibrator.o spi.o b2s_prints.o
CALIBPROC_OBJ = ../robotsoft-calibrator/calibproc.o

CFLAGS = -I. -Wall -Winline -std=c99 -DROBOTSOFT -D_XOPEN_SOURCE=700

CFLAGS += -DCALIBRATOR

CFLAGS += -g
CFLAGS += -O3

LDFLAGS =

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

boardmon: $(BOARDMON_OBJ)
	$(LD) $(LDFLAGS) -o boardmon $^ -lm -pthread

calibrator: $(CALIBRATOR_OBJ)
	$(LD) $(LDFLAGS) -o calibrator $^ -lm -pthread

calibproc: $(CALIBPROC_OBJ)
	$(LD) $(LDFLAGS) -o calibproc $^ -lm -lpng

cp:
	scp *.c *.h makefile $(DEVIP):~/robotsoft
	scp ../robotsoft-calibrator/*.c ../robotsoft-calibrator/*.h $(DEVIP):~/robotsoft-calibrator

e:
	gedit --new-window makefile $(EXTRA_HEADERS) `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_boardmon:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_calibrator:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h ../robotsoft-calibrator/calibproc.c `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.h"/g` &

clean:
	rm *.o
	rm ../robotsoft-calibrator/*.o
