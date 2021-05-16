OBJS=AXIController.o 
INC=~/workspace/os/can4linux/can4linux 

all:	AXIController

AXIController:	$(OBJS)
	arm-linux-gnueabihf-g++ $(LDFLAGS)  -pthread  -o AXIController $(OBJS)

$(OBJS): %.o:	%.cpp %.cpp
	arm-linux-gnueabihf-g++ $(CFLAGS) -I ~/workspace/os/can4linux/can4linux -pthread -c -o $@ $<
