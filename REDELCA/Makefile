CONTIKI  = ../..
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI_SOURCEFILES += definitions.c

%.o: %.c definitions.h
	gcc -Wall -c -o $@ $<

all: firmware.o definitions.o
	gcc firmware.c

CONTIKI_PROJECT = ../..

include $(CONTIKI)/Makefile.include
