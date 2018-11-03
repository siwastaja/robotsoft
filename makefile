# For your specific compile-time configuration, take a copy of the revision-controlled config.h.template as config.h, and make
# your modifications. Then, just
# make
# and run:
# ./robotsoft

DEVIP = 192.168.43.102

CC = gcc
LD = gcc

ROBOTSOFT_OBJ = main.o tcp_comm.o tpc_parser.o spi.o
BOARDMON_OBJ = boardmon.o spi.o

CFLAGS = -Wall -Winline -std=c99 -DROBOTSOFT

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

cp:
	scp *.c *.h makefile $(DEVIP):~/robotsoft_dev

e:
	gedit --new-window makefile config.h api_board_to_soft.h api_soft_to_board.h boardmon.c misc.h `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(ROBOTSOFT_OBJ)" | sed s/"\.o"/"\.h"/g` &

