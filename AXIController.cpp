#include <errno.h>
#include <getopt.h>
//#include <inttypes.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/timeb.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>

#include <can4linux.h>
#include <pthread.h>


#include <time.h>       /* time */


// Local includes
#include "ZYNQServer/board.h"
#include "Xilinx/xil_io.h"


#define MAP_WIDTH 4
#define PROGRAM_NAME "uioctl"

#define AXIRegisterSize 8

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 3

#define RXMessageCount 32

#define LOWERBITS_MASK 0x00000000FFFFFFFF
#define HIGHERBITS_MASK 0xFFFFFFFF00000000

// Multithreading
#define RX_THREAD 0
#define RX_THREAD1 3
#define RX_THREAD2 4
#define TX_THREAD 1
#define AXI_THREAD 2
#define TCP_THREAD 5
#define MAX_THREADS 6

#define CAN1Offset 0x100
#define CAN2Offset 0x200
#define CAN3Offset 0x300
#define CAN4Offset 0x400
#define CAN5Offset 0x500
#define CAN6Offset 0x600

/* local defined data types
---------------------------------------------------------------------------*/
typedef struct {
    int tno;

    int can_fd;

    u_int8_t WriteIndex;

    u_int8_t ReadIndex;

} thread_data_t;

/* list of external used functions, if not in headers
---------------------------------------------------------------------------*/

/* list of global defined functions
---------------------------------------------------------------------------*/

/* list of local defined functions
---------------------------------------------------------------------------*/
static void *can1_rx_thread( void *ptr );
static void *can1_rx1_thread( void *ptr );
static void *can1_rx2_thread( void *ptr );
static void *can1_tx_thread( void *ptr );

static void *axi_handle_thread( void *ptr );

static void *tcp_handle_thread( void *ptr );

pthread_mutex_t lockCAN1 = PTHREAD_MUTEX_INITIALIZER;


u_int32_t transmitAXIAdresses[] = {0x000001B4, 0x000001AC, 0x000001A4};
u_int8_t AXIAdressCounter = 0;


struct sched_param AXIsched, CAN1Rcvsched;

    // dummy Test Data
    VCUmessage nachricht;

/*----------------------------------------------*/

// Transmit Buffer for all CAN Busses
canmsg_t transmitCAN1Buffer[TX_BUFFER_SIZE] = {0};
canmsg_t transmitCAN2Buffer[TX_BUFFER_SIZE];
canmsg_t transmitCAN3Buffer[TX_BUFFER_SIZE];
canmsg_t transmitCAN4Buffer[TX_BUFFER_SIZE];
canmsg_t transmitCAN5Buffer[TX_BUFFER_SIZE];
canmsg_t transmitCAN6Buffer[TX_BUFFER_SIZE];
// Variable used by the AXI Threads
u_int8_t transmitWriteIndex;
// Variable used by the CAN Threads
u_int8_t transmitReadIndex;
// Variable used by the TCP Thread
u_int8_t TCPTransmitIndex;

/*----------------------------------------------*/

// Receive Buffer for all CAN Busses
// canmsg_t receiveCAN1Buffer[RX_BUFFER_SIZE];
canmsg_t receiveCAN1Buffer[RX_BUFFER_SIZE] = {0};
canmsg_t receiveCAN2Buffer[RX_BUFFER_SIZE];
canmsg_t receiveCAN3Buffer[RX_BUFFER_SIZE];
canmsg_t receiveCAN4Buffer[RX_BUFFER_SIZE];
canmsg_t receiveCAN5Buffer[RX_BUFFER_SIZE];
canmsg_t receiveCAN6Buffer[RX_BUFFER_SIZE];
// Variable used by the AXI Threads
u_int8_t receiveReadIndex;
// Variable used by the CAN Threads
u_int8_t recieveWriteIndex;
// Variable used by the TCP Thread
u_int8_t TCPReceiveIndex;

/*----------------------------------------------*/


/* local defined variables 
------------------------------------------------*/

// For Multithreading
static pthread_t threads[MAX_THREADS];
static thread_data_t td[MAX_THREADS];

int c;
int simulink;
int can1, can2;
char* fpath = "/dev/mwipcore";
void *MEMptrRead;
void *MEMptrWrite;
int stopFlag = 1;


/* TCP Handling (Communication with Starkstrom GUI)
-----------------------------------------------------------------------------------------------*/









// Function to set Bitrate, Parameters: char device & bitrate
int	set_bitrate(
	int can_fd,			/* device descriptor */
	int baud		/* bit rate */
	)
{
config_par_t  cfg;
volatile command_par_t cmd;


    cmd.cmd = CMD_STOP;
    ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

    cfg.target = CONF_TIMING; 
    cfg.val1   = baud;
    ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);

    cmd.cmd = CMD_START;
    ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);
    return 0;
}


void InitAXIControl(void){
	transmitWriteIndex = 0;
	receiveReadIndex = 0;

	// Set Baudrate of all CAN Busses
	// CAN 1
	set_bitrate(can1, 1000);
	// CAN 2
	set_bitrate(can2, 1000);
	// CAN 3
	// ...
}

// Example CAN ID 1 Channel 1: 100+(1*8) -> AXI Register: 108
u_int32_t calcAXIAddressHigh(int CANChannelOffset, u_int16_t CAN_ID){

	u_int32_t tmpAXIAddress;
	
	return tmpAXIAddress = CANChannelOffset + (CAN_ID * AXIRegisterSize);
}

// Example CAN ID 1 Channel 1: 100+(1*8) - 4 -> AXI Register: 104
u_int32_t calcAXIAddressLow(int CANChannelOffset, u_int16_t CAN_ID){

	u_int32_t tmpAXIAddress;

	tmpAXIAddress = calcAXIAddressHigh(CANChannelOffset, CAN_ID) - 4;

	if(tmpAXIAddress > 0x00000FF0){
		tmpAXIAddress = 0x00000FF0;
	}

	return tmpAXIAddress;
}

u_int32_t calcCANID(u_int32_t AXIAddress){

	u_int32_t tmpCAN_ID;
	int CANChannelOffset;
	
	return tmpCAN_ID = (AXIAddress & 0xFF) / 0x8;
}



/*
 * This function is used to read a CAN Message from receiveBuffer and send it to the specified AXI Slave register address
 */
void sendToAXI(){

	u_int32_t tmpAXIAddress;
	u_int8_t tmpCANChannel = 1;
	u_int32_t CANOffset;
	u_int16_t tmpCAN_ID;
	u_int64_t tmpCANDataHigh;
	u_int64_t tmpCANDataLow;

	canmsg_t tmpCANMessage;

	tmpCANMessage = receiveCAN1Buffer[receiveReadIndex];
	
	// Handle CAN1 1 received messages
	//tmpCANChannel = receiveCANBuffer[receiveReadIndex]->Channel;
	tmpCAN_ID = receiveCAN1Buffer[receiveReadIndex].id;
	// Packing the Data field in High and Lower 32 Bit Packages
	tmpCANDataLow = (tmpCANMessage.data[3] << 24) + (tmpCANMessage.data[2] << 16) + (tmpCANMessage.data[1] << 8) + (tmpCANMessage.data[0]);
	tmpCANDataHigh = (tmpCANMessage.data[7] << 24) + (tmpCANMessage.data[6] << 16) + (tmpCANMessage.data[5] << 8) + (tmpCANMessage.data[4]);
	receiveReadIndex++;




	if(receiveReadIndex == RX_BUFFER_SIZE){
		receiveReadIndex = 0;
	}

	CANOffset = 0x100 * tmpCANChannel;


	//send lower 32 Bit
	tmpAXIAddress = calcAXIAddressLow(CANOffset, tmpCAN_ID);

	// Map current Address region for tmpAXIAddress
	MEMptrWrite = mmap(NULL, MAP_WIDTH + tmpAXIAddress, PROT_READ|PROT_WRITE, MAP_SHARED, simulink, 0);
	//Write tmpCANDataLow to mapped Address
    *((unsigned *)(MEMptrWrite + tmpAXIAddress)) = tmpCANDataLow;
    //Unnmap Address region
    munmap(MEMptrWrite, MAP_WIDTH);

    //send upper 32 Bit
    tmpAXIAddress+=4;
	MEMptrWrite = mmap(NULL, MAP_WIDTH + tmpAXIAddress, PROT_READ|PROT_WRITE, MAP_SHARED, simulink, 0);
    *((unsigned *)(MEMptrWrite + tmpAXIAddress)) = tmpCANDataHigh;

       /* clean up */
    munmap(MEMptrWrite, MAP_WIDTH);

    //send to GUI
    nachricht.VCUId = tmpCAN_ID - 0x170;
    nachricht.VCUData = tmpCANDataLow;
} 

/*
 * This function is used to read a AXI Slave register and send the data to the corresponding CAN Message into the transmitBuffer
 */
void readFromAXI(){

	// @TODO
	// not sure how to filter which AXI Addreses yet

	u_int32_t tmpAXIAddress;
	u_int8_t tmpCANChannel;
	u_int32_t tmpCAN_ID;
	u_int32_t tmpCANDataLow;
	u_int32_t tmpCANDataHigh;

	tmpAXIAddress = transmitAXIAdresses[AXIAdressCounter];
	AXIAdressCounter++;
	if(AXIAdressCounter == 3){
		AXIAdressCounter = 0;
	}

	MEMptrRead = mmap(NULL, MAP_WIDTH + tmpAXIAddress, PROT_READ|PROT_WRITE, MAP_SHARED, simulink, 0);

	tmpCAN_ID = calcCANID(tmpAXIAddress);
	tmpCANDataLow = *((unsigned *) (MEMptrRead + tmpAXIAddress));

	tmpAXIAddress += 4;

	//MEMptrRead = mmap(NULL, MAP_WIDTH + tmpAXIAddress, PROT_READ|PROT_WRITE, MAP_SHARED, simulink, 0);

	tmpCANDataHigh = *((unsigned *) (MEMptrRead + tmpAXIAddress));

	transmitCAN1Buffer[transmitWriteIndex].id = tmpCAN_ID;
	transmitCAN1Buffer[transmitWriteIndex].length = 8;
	transmitCAN1Buffer[transmitWriteIndex].data[7] = (u_int8_t)(tmpCANDataLow & 0x000000FF);
	transmitCAN1Buffer[transmitWriteIndex].data[6] = (u_int8_t)((tmpCANDataLow & 0x0000FF00) >> 8);
	transmitCAN1Buffer[transmitWriteIndex].data[5] = (u_int8_t)((tmpCANDataLow & 0x00FF0000)>> 16);
	transmitCAN1Buffer[transmitWriteIndex].data[4] = (u_int8_t)((tmpCANDataLow & 0xFF000000) >> 24);
	transmitCAN1Buffer[transmitWriteIndex].data[3] = (u_int8_t)(tmpCANDataHigh & 0x000000FF);
	transmitCAN1Buffer[transmitWriteIndex].data[2] = (u_int8_t)((tmpCANDataHigh & 0x0000FF00) >> 8);
	transmitCAN1Buffer[transmitWriteIndex].data[1] = (u_int8_t)((tmpCANDataHigh & 0x00FF0000) >> 16);
	transmitCAN1Buffer[transmitWriteIndex].data[0] = (u_int8_t)((tmpCANDataHigh & 0xFF000000) >> 24);

	transmitWriteIndex++;

	if(transmitWriteIndex == TX_BUFFER_SIZE){
		transmitWriteIndex = 0;
	}

	/* clean up */
    munmap(MEMptrRead, MAP_WIDTH);

}

/*
*	This functions sends the next CAN Frame from Buffer to the physical CAN BUS
*/
void sendtoCAN(){

	canmsg_t tmpCAN1Message = transmitCAN1Buffer[transmitWriteIndex];
	transmitWriteIndex++;

	if(transmitWriteIndex == TX_BUFFER_SIZE){
		transmitWriteIndex = 0;
	}
	tmpCAN1Message.length = 8;
	// write next Message to Transmit 
	// CAN Network 1
	write(can1, &tmpCAN1Message, 1);
	// CAN Network 2
	// CAN Network 3
	// CAN Network 4
	// CAN Network 5
	// CAN Network 6

}

/*
*	This functions reads the next CAN Frame from CANBuffer and writes it to internal Buffer
*/
void readfromCAN(){

	canmsg_t tmpCANMessage;
	int ret;

	//Receive from CAN1 Network
	ret = (int)read(can1, &tmpCANMessage, 1);
		if(ret == -1) {
			perror("Read Error");
		}

	/*printf("Received CAN ID: 0x%08x\n", tmpCANMessage.id);
	printf("Received CAN Data: 0x%08x\n", tmpCANMessage.data[0]);
	printf("Received CAN Data: 0x%08x\n", tmpCANMessage.data[1]);
	printf("Received CAN Data: 0x%08x\n", tmpCANMessage.data[2]);*/
	
	// Write received Message to internal Buffer
	receiveCAN1Buffer[0] = tmpCANMessage;
	recieveWriteIndex++;

	if(recieveWriteIndex == RX_BUFFER_SIZE){
		recieveWriteIndex = 0;
	}


}

int main(int argc,char **argv) {

	int counter = 10;
	/*u_int32_t tmpAXIAddress = 0x000001B4;
	u_int32_t Data;*/

    simulink = open("/dev/mwipcore", O_RDWR|O_SYNC);
    can1 = open("/dev/can1", O_RDWR); // Here the devices are switched because of Hardwarelayout decisions
    //can2 = open("/dev/can0", O_RDWR);


	InitAXIControl();


	int max_rr_priority = sched_get_priority_max(SCHED_RR);
    int min_rr_priority = sched_get_priority_min(SCHED_RR);
    int max_ff_priority = sched_get_priority_max(SCHED_FIFO);
    int min_ff_priority = sched_get_priority_min(SCHED_FIFO);

    AXIsched.sched_priority = max_rr_priority;
    CAN1Rcvsched.sched_priority = max_rr_priority;

    //pthread_setschedparam(threads[AXI_THREAD], SCHED_RR, &AXIsched);
    //pthread_setschedparam(threads[RX_THREAD], SCHED_RR, &CAN1Rcvsched);
  

    td[RX_THREAD].tno = 1;
    td[RX_THREAD].can_fd = can1;
    //td[RX_THREAD].rx_canmessages = &receiveCAN1Buffer;
    td[RX_THREAD].WriteIndex = recieveWriteIndex;
    pthread_create(&threads[RX_THREAD], NULL,
		can1_rx_thread, (void *) &td[RX_THREAD]);

    /*td[RX_THREAD1].tno = 4;
    td[RX_THREAD1].can_fd = can1;
    td[RX_THREAD1].canmessages = &receiveCAN1Buffer;
    td[RX_THREAD1].WriteIndex = recieveWriteIndex;
    pthread_create(&threads[RX_THREAD1], NULL,
		can1_rx1_thread, (void *) &td[RX_THREAD1]);

    td[RX_THREAD1].tno = 5;
    td[RX_THREAD1].can_fd = can1;
    td[RX_THREAD1].canmessages = &receiveCAN1Buffer;
    td[RX_THREAD1].WriteIndex = recieveWriteIndex;
    pthread_create(&threads[RX_THREAD2], NULL,
		can1_rx2_thread, (void *) &td[RX_THREAD2]);*/

    td[TX_THREAD].tno = 2;
    td[TX_THREAD].can_fd = can1;
    //td[TX_THREAD].tx_canmessages = &transmitCAN1Buffer;
    td[TX_THREAD].WriteIndex = transmitWriteIndex;
    pthread_create(&threads[TX_THREAD], NULL,
		can1_tx_thread, (void *) &td[TX_THREAD]);

    td[AXI_THREAD].tno = 3;
    pthread_create(&threads[AXI_THREAD], NULL,
		axi_handle_thread, (void *) &td[AXI_THREAD]);

    /*td[TCP_THREAD].tno = 6;
    pthread_create(&threads[TCP_THREAD], NULL,
		tcp_handle_thread, (void *) &td[TCP_THREAD]);*/

    pthread_join(threads[RX_THREAD], NULL);
    //pthread_join(threads[RX_THREAD1], NULL);
    //pthread_join(threads[RX_THREAD2], NULL);
   	pthread_join(threads[TX_THREAD], NULL);
    pthread_join(threads[AXI_THREAD], NULL);
    //pthread_join(threads[TCP_THREAD], NULL);

    return 0;

    /* clean up */
    //munmap(MEMptr, MAP_WIDTH);
    close(simulink);
    close(can1);
    //close(can2);


    return EXIT_SUCCESS;
}

/*
***************************************************************************/
/**
*
* TCP thread that reads/writes to a TCP socket on IP Adress & Port
* IP: 192.168.178.50 Port: 69
*
* \retval 0
*
*/
static void *tcp_handle_thread (void *ptr)
{
	int i;
	int ret;
	int tno;



	pthread_t tid;

    tid = pthread_self();
    tno = ((thread_data_t *) ptr)->tno;

    VCUProtocoll();
    VCUProtocoll ZYNQServer;


    
    VCUmessage* receivenachricht;
    receivenachricht = (VCUmessage*)malloc(sizeof(VCUmessage));
    VCUmessage* message2;
    message2 = (VCUmessage*)malloc(sizeof(VCUmessage));
    message2->VCUChannel = 0x03;
    message2->VCUId = 0x1;
    message2->VCUData = 0xFF;
    sockaddr_in hint;
    int counter = 0;


    //___________________creating Server socked and enalbe connection to client _____________//
    int listening = socket(AF_INET, SOCK_STREAM, 0);
    if (listening == -1)
    {
        //cerr << "Can't create a socket! Quitting" << endl;
        //return -1;
    }

    // Bind the ip address and port to a socket
    hint.sin_family = AF_INET;
    hint.sin_port = htons(69);
    inet_pton(AF_INET, "192.168.178.50", &hint.sin_addr); 

    bind(listening, (sockaddr*)&hint, sizeof(hint));

    // Tell Winsock the socket is for listening
    listen(listening, SOMAXCONN);

    // Wait for a connection
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);

    printf("Socket created!\n");

    // Hier bleiben wir aktuell haengen!
    int GUISocket = accept(listening, (sockaddr*)&client, &clientSize);

    char host[NI_MAXHOST];      // Client's remote name
    char service[NI_MAXSERV];   // Service (i.e. port) the client is connect on

    memset(host, 0, NI_MAXHOST); // same as memset(host, 0, NI_MAXHOST);
    memset(service, 0, NI_MAXSERV);

    if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
    {
        //cout << host << " connected on port " << service << endl;
    }
    else
    {
        inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
        //cout << host << " connected on port " << ntohs(client.sin_port) << endl;
    }

    // Close listening socket
    close(listening);

    int c1, c2, c3;


    do {
    	/*printf(" ZYNQ Server is running...");
    	ZYNQServer.VCU_message_receive(receivenachricht,GUISocket);
		
		c1 = rand() % 20000 + 1;
		c2 = rand() % 4000 + 1;
		c3 = rand() % 1000 + 1;
    	         
    	message2->VCUData = c1;
    	message2->VCUChannel = 0x01;
    	message2->VCUId = 0x1;
		ZYNQServer.writeBuffer(message2,GUISocket);
		c1 = rand() % 20000 + 1;
		message2->VCUData = c1;
		message2->VCUId = 0x2;
		ZYNQServer.writeBuffer(message2,GUISocket);


		message2->VCUData = c2;
		message2->VCUChannel = 0x02;
		ZYNQServer.writeBuffer(message2,GUISocket);
		message2->VCUData = c3;
		message2->VCUChannel = 0x03;		
		ZYNQServer.writeBuffer(message2,GUISocket);*/
        

    } while (1);

    fprintf(stderr, "TCP Thread exit\n", tno);
    close(GUISocket);
    pthread_exit(NULL);
}

/***************************************************************************/
/**
*
* CAN thread that reads from the CAN controller
*
*
* \retval 0
*
*/
static void *can1_rx2_thread (void *ptr){
	int i;
	int ret;
	int tno;
	int can_fd;
	int received = 0;
	char type;

	canmsg_t *canmessages;
	u_int8_t WriteIndex;

	pthread_t tid;
	canmsg_t tmpCANMessage;
	tmpCANMessage.length = 8;

    tid = pthread_self();
    tno = ((thread_data_t *) ptr)->tno;

    can_fd     = ((thread_data_t *) ptr)->can_fd;
    WriteIndex = ((thread_data_t *) ptr)->WriteIndex;

    do {

		//ret = read(can_fd,  &tmpCANMessage, 1);
		//ret = read(can_fd,  &receiveCAN1Buffer, 1);	
		//usleep(10);
		/*receiveCAN1Buffer[WriteIndex] = tmpCANMessage;
				WriteIndex++;

				if(WriteIndex == RX_BUFFER_SIZE){
				WriteIndex = 0;
				}*/

		
    } while (1);

    fprintf(stderr, "Thread(%d) RX exit\n", tno);
    pthread_exit(NULL);
}

/***************************************************************************/
/**
*
* CAN thread that reads from the CAN controller
*
*
* \retval 0
*
*/
static void *can1_rx1_thread (void *ptr)
{
	int i;
	int ret;
	int tno;
	int can_fd;
	int received = 0;
	char type;

	canmsg_t *canmessages;
	u_int8_t WriteIndex;

	pthread_t tid;
	canmsg_t tmpCANMessage;
	tmpCANMessage.length = 8;

    tid = pthread_self();

    tno = ((thread_data_t *) ptr)->tno;

    can_fd     = ((thread_data_t *) ptr)->can_fd;
    WriteIndex = ((thread_data_t *) ptr)->WriteIndex;

    do {

		//ret = read(can_fd,  &tmpCANMessage, 1);
		//ret = read(can_fd,  &receiveCAN1Buffer, 1);	
		//usleep(100);
		/*receiveCAN1Buffer[WriteIndex] = tmpCANMessage;
				WriteIndex++;

				if(WriteIndex == RX_BUFFER_SIZE){
				WriteIndex = 0;
				}*/

		
    } while (1);

    fprintf(stderr, "Thread(%d) RX exit\n", tno);
    pthread_exit(NULL);
}

/***************************************************************************/
/**
*
* CAN thread that reads from the CAN controller
*
*
* \retval 0
*
*/
static void *can1_rx_thread(void *ptr){

	int i;
	int ret;
	int tno;
	int can_fd;
	int received = 0;
	char type;
	int pError = 0;

	canmsg_t *canmessages;
	u_int8_t WriteIndex;

	pthread_t tid;
	canmsg_t tmpCANMessage;
	tmpCANMessage.length = 8;

    tid = pthread_self();
    tno = ((thread_data_t *) ptr)->tno;
    
    pthread_setschedparam(tid, SCHED_RR, &CAN1Rcvsched);

    can_fd     = ((thread_data_t *) ptr)->can_fd;
    WriteIndex = ((thread_data_t *) ptr)->WriteIndex;

    do {

		// Read the last message from Kerneldriver, RX_BUFFER_SIZE -> number of bytes
		ret = read(can_fd,  &receiveCAN1Buffer, RX_BUFFER_SIZE);	

    } while (10);

    fprintf(stderr, "Thread(%d) RX exit\n", tno);
    pthread_exit(NULL);
}

/***************************************************************************/
/**
*
* CAN thread that writes to the CAN controller
*
* \retval 0
*
*/
static void *can1_tx_thread (void *ptr){
	int i;
	int ret;
	int tno;
	int can_fd;
	int pError = 0;
	canmsg_t *canmessages;
	u_int8_t WriteIndex;

	pthread_t tid;
	canmsg_t tx;

    tid = pthread_self();
    fprintf(stderr, "Thread(%d) TX startet: Id = %x\n",  tno, (int)tid);

    can_fd = ((thread_data_t *) ptr)->can_fd;
    WriteIndex = ((thread_data_t *) ptr)->WriteIndex;

    do {
		

    	

			canmsg_t tmpCAN1Message = transmitCAN1Buffer[WriteIndex];
			WriteIndex++;

			if(WriteIndex == TX_BUFFER_SIZE){
				WriteIndex = 0;
			}

			tmpCAN1Message.length = 8;
	
			// write next Message to Transmit 
			// CAN Network 1

			write(can_fd, &tmpCAN1Message, 1);

    } while (10);

    fprintf(stderr, "Thread(%d) TX exit\n", tno);

    pthread_exit(NULL);
}


/***************************************************************************/
/**
*
* AXI thread that reads and writes to/from the AXI Bus of the Simulink model
*
*
* \retval 0
*
*/
static void *axi_handle_thread (void *ptr){

	pthread_t tid;

    tid = pthread_self();
    pthread_setschedparam(tid, SCHED_RR, &AXIsched);

	do{

    for(int i = 0; i < RXMessageCount; i++){
			sendToAXI();
		}
	readFromAXI();

	}while(10);


	pthread_exit(NULL);
}
