CC = gcc  
# CC = arm-none-linux-gnueabi-gcc  
	      
objs = serial.o main.o  
srcs = serial.c main.c  

test:$(objs)  
	$(CC)  -o test $(objs) -lm -lpthread
	@make clean  
      
main.o:$(srcs) serial.h 
	$(CC) -c  $(srcs)  

serial.o:serial.c serial.h
	$(CC) -c serial.c 
      
clean:  
	-rm -f *.o  
