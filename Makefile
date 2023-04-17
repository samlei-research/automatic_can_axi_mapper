OBJS=can_axi_mapper.o 
INC=~/workspace/os/can4linux/can4linux 

all:	can_axi_mapper

can_axi_mapper:	$(OBJS)
	arm-linux-gnueabihf-g++ $(LDFLAGS)  -pthread  -o can_axi_mapper $(OBJS)

$(OBJS): %.o:	%.cpp %.cpp
	arm-linux-gnueabihf-g++ $(CFLAGS) -I ~/can4linux/can4linux -pthread -c -o $@ $<
