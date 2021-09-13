CC = gcc  
# CC = arm-none-linux-gnueabi-gcc  
	      
objs = serial.o gps.o main.o  
srcs = serial.c gps.c main.c  

test:$(objs)  
	$(CC)  -o test $(objs) -lm -lpthread -lusb
	@make clean  
      
main.o:$(srcs) serial.h gps.h
	$(CC) -c  $(srcs)  

serial.o:serial.c serial.h
	$(CC) -c serial.c 

gps.o:gps.c gps.h
	$(CC) -c gps.c 
      
clean:  
	-rm -f *.o  
