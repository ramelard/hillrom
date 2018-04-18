SHELL = /bin/sh
CC    = gcc
CFLAGS       = -fPIC
LDFLAGS = -shared
NAME = extract_vitals_tk1
TARGET  = $(NAME).so

SOURCES = $(shell echo *.c)
HEADERS = $(shell echo *.h)
OBJECTS=$(SOURCES:.c=.o)

all: $(TARGET) 

%.o: $.c $(HEADERS)
	$(CC) -c -fPIC -o $@ $< 

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)

#$(NAME).o: $(NAME).c
#	gcc -c -fPIC -o $(NAME).o $(NAME).c

#$(TARGET): $(NAME).o
#	gcc -shared -o $(TARGET) $(NAME).o

#$(TARGET): $(OBJECTS)
#	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS)

