TARGET		= libdxl.so
OBJS		= dxl_hal.o dynamixel.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../include
LIBDIRS		+= 
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2

CC			= gcc
AR			= ar

$(TARGET): $(OBJS)
	$(CC) -shared -o $(TARGET) dxl_hal.o dynamixel.o
	cp $(TARGET) ../lib
	
.c.o:
	$(CC) -c -fPIC $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

