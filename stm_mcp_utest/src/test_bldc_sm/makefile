#
# makefile for individual unit test module
#

APP_INCS = ../inc
CFLAGS = -I ./inc  -I $(APP_INCS)
CFLAGS += -DUNIT_TEST
LDFLAGS =
CC = gcc
OBJS = obj/main.o obj/test_bldc_sm.o obj/BLDC_sm.o obj/mdata.o obj/stubs.o obj/putf.o

obj/putf.o: src/putf.c
	$(CC) $(CFLAGS) -c src/putf.c -o obj/putf.o


obj/main.o: src/test_bldc_sm/main.c
	$(CC) $(CFLAGS) -c src/test_bldc_sm/main.c -o obj/main.o


obj/stubs.o: src/stubs.c
	$(CC) $(CFLAGS) -c src/stubs.c -o obj/stubs.o

obj/test_bldc_sm.o: src/test_bldc_sm/test_bldc_sm.c
	$(CC) $(CFLAGS) -c src/test_bldc_sm/test_bldc_sm.c -o obj/test_bldc_sm.o


obj/BLDC_sm.o: ../src/BLDC_sm.c
	$(CC) $(CFLAGS) -c ../src/BLDC_sm.c -o obj/BLDC_sm.o

obj/mdata.o: ../src/mdata.c
	$(CC) $(CFLAGS) -c ../src/mdata.c -o obj/mdata.o

unit_test: $(OBJS)
	$(CC) $(LDFLAGS) obj/main.o obj/test_bldc_sm.o obj/BLDC_sm.o obj/mdata.o obj/stubs.o obj/putf.o -o unit_test
	
all: unit_test	

test: all
	./unit_test.exe | tee  test.out
	
clean:
	rm $(OBJS) unit_test test.out
	