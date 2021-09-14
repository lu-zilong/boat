CC = gcc  
# CC = arm-none-linux-gnueabi-gcc  
	      
objs = serial.o rud.o dirrev.o controller.o joystick.o main.o  
srcs = serial.c rud.c dirrev.c controller.c joystick.c main.c  
      
test:$(objs)  
	$(CC)  -o test $(objs) -lm -lpthread 
	@make clean  
      
main.o:$(srcs) serial.h rud.h dirrev.h controller.h joystick.h
	$(CC) -c  $(srcs)  


serial.o:serial.c serial.h
	$(CC) -c serial.c 

rud.o:rud.c rud.h
	$(CC) -c rud.c 

dirrev.o:dirrev.c dirrev.h
	$(CC) -c dirrev.c 

controller.o:controller.c controller.h
	$(CC) -c controller.c 

joystick.o:joystick.c joystick.h
	$(CC) -c joystick.c 
      
clean:  
	-rm -f *.o  
