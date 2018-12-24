# For your specific compile-time configuration, take a copy of the revision-controlled config.h.template as config.h, and make
# your modifications. Then, just
# make
# and run:
# ./robotsoft

DEVIP = 192.168.1.5

CC = gcc
LD = gcc

ROBOTSOFT_OBJ = main.o tcp_comm.o tcp_parser.o spi.o b2s_prints.o map_memdisk.o mapping.o routing.o hwdata.o
BOARDMON_OBJ = boardmon.o spi.o b2s_prints.o
CALIBRATOR_OBJ = ../robotsoft-calibrator/calibrator.o spi.o b2s_prints.o
CALIBPROC_OBJ = ../robotsoft-calibrator/calibproc.o

CFLAGS = -I. -Wall -Winline -std=c99 -DROBOTSOFT

CFLAGS += -DCALIBRATOR

CFLAGS += -g

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
	$(LD) $(LDFLAGS) -o robotsoft $^ -lm -pthread

boardmon: $(BOARDMON_OBJ)
	$(LD) $(LDFLAGS) -o boardmon $^ -lm -pthread

calibrator: $(CALIBRATOR_OBJ)
	$(LD) $(LDFLAGS) -o calibrator $^ -lm -pthread

calibproc: $(CALIBPROC_OBJ)
	$(LD) $(LDFLAGS) -o calibproc $^ -lm

cp:
	scp *.c *.h makefile $(DEVIP):~/robotsoft_dev
	scp ../robotsoft-calibrator/*.c ../robotsoft-calibrator/*.h $(DEVIP):~/robotsoft-calibrator

e:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h datatypes.h `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_boardmon:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(BOARDMON_OBJ)" | sed s/"\.o"/"\.h"/g` &

e_calibrator:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h misc.h ../robotsoft-calibrator/calibproc.c `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(CALIBRATOR_OBJ)" | sed s/"\.o"/"\.h"/g` &

clean:
	rm *.o
	rm ../robotsoft-calibrator/*.o
