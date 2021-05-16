
OBJS=AXIController.o ZYNQServer/board.o
INC=~/can4linux/can4linux ZYNQServer

all:	AXIController

AXIController:	$(OBJS)
	arm-linux-gnueabihf-g++ $(LDFLAGS)  -pthread  -o AXIController $(OBJS)

$(OBJS): %.o:	%.cpp %.cpp
	arm-linux-gnueabihf-g++ $(CFLAGS) -I ~/can4linux/can4linux -pthread -c -o $@ $<
