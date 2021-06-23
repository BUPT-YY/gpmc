#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#include <unistd.h>
#include <csignal>
#include <cctype>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>


#pragma pack(1)

#define	SDU_LEN_MAX     2048
#define	$XY		5855268

std::vector<std::string> targetIps = {
    "127.0.0.1", "127.0.0.1", "127.0.0.1", "127.0.0.1"
};

//#define SduTarget2_IP	 "127.0.0.1"
#define SduTarget2_Port	  12345

#define SduApp1_self_IP	"127.0.0.1"	//本机IP地址，即接收端地址
#define SduApp1_self_Port	12345			//本机端口号，即接收端端口号

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
  __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

typedef struct {//含Flag位的sdu发送帧格式
    uint32_t Flag;//识别标识位
    uint8_t T;//占一位，为3时是ack为1时是sdu
    uint8_t priority;//sdu的优先级
    uint8_t seg_sn;			//分段后的相对位置
    uint16_t sdu_length;//长度指示
    uint32_t sdu_sn;                    //序列号
    uint32_t service_type;            //业务类型
    uint32_t sender_id;                //发送者id
    //uint32_t t_stamp;			//时间戳
    uint8_t data[SDU_LEN_MAX];        //实际发送/接收数据
}SduFlagPack;

int main() {
    int fd;
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    printf("/dev/mem opened.\n");
    fflush(stdout);

    /* Map one page */
    void *map_base, *map_base2;

 
    off_t target, target2;


    target = strtoul("0x02000000", 0, 0);
	target2 = strtoul("0x03000000", 0, 0);
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	map_base2= mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target2 & ~MAP_MASK);
    
	if(map_base == (void *) -1) FATAL;
	if(map_base2 == (void *) -1) FATAL;
	unsigned short *virt_addr  = reinterpret_cast<unsigned short *>(map_base);
	unsigned short *virt_addr2 = reinterpret_cast<unsigned short *>(map_base2);

    //virt_addr = (unsigned short *)(map_base);
	//virt_addr2 = (unsigned short *)(map_base2);
	
    // >>> init socket
    int send_socket_1 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
    sockaddr_in send_addr_1;memset(&send_addr_1, 0, sizeof(send_addr_1));  //每个字节都用0填充
    send_addr_1.sin_family = AF_INET;  //使用IPv4地址
    send_addr_1.sin_addr.s_addr = inet_addr("127.0.0.1");  //具体的IP地址
    send_addr_1.sin_port = htons(SduTarget2_Port);  //端口

    SduFlagPack SduFlagPack_0;
	SduFlagPack_0.Flag=$XY;
	SduFlagPack_0.T=1;
	SduFlagPack_0.sdu_sn=0;
	SduFlagPack_0.seg_sn=0;
	SduFlagPack_0.sender_id=111;
	SduFlagPack_0.service_type=2;

    int recv_socket_1 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);//创建服务器端socket ，使用面向消息的UDP
    if(recv_socket_1==-1) {
	    printf("接收端socket创建失败\n");
	    return -1;
	}
	printf("接收端socket创建成功\n");
    //设置本地ip地址和端口号
    sockaddr_in recv_addr_1;//接收端ip地址 端口地址
    memset(&recv_addr_1, 0, sizeof(recv_addr_1));  //每个字节都用0填充
    recv_addr_1.sin_family = AF_INET;  //使用IPv4地址
    recv_addr_1.sin_addr.s_addr = inet_addr(SduApp1_self_IP);  //具体的IP地址，本地虚拟机ip地址10.211.55.3
    recv_addr_1.sin_port = htons(SduApp1_self_Port);  //端口
	printf("本机IP:%s\n本机端口号:%d\n",SduApp1_self_IP,SduApp1_self_Port);

    //设置监听套接字，使得重启之后不会影响绑定
	int on =1;
	if(setsockopt(recv_socket_1,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on))<0) {
	    printf("Failed to set address reuse\n");
	    return -1;
	}
	printf("Setting address reuse succeeded\n");
	

    int r = bind(recv_socket_1, (struct sockaddr*)&recv_addr_1, sizeof(recv_addr_1));//将端口地址，ip地址与socket进行绑定
 	if(r==-1) {
	    printf("绑定端口失败\n");
	    return -1;
	}
	printf("绑定端口成功\n");

    sockaddr_in send_addr_2;//用于返回发送方的地址
    socklen_t send_addr_2_size = sizeof(send_addr_2);
    SduFlagPack SduFlagPack_1; 
	SduFlagPack *p=&SduFlagPack_1;	
    // <<< socker init finished

	unsigned short* read_buffer = (unsigned short*)(SduFlagPack_0.data);
	unsigned short* write_buffer = (unsigned short*)(SduFlagPack_1.data);
    unsigned short read_data, read_code;	
	unsigned int yy_i_rd, yy_i_wr;
    unsigned int destination_id;
send:
    //一次发送
    yy_i_rd = 0;
    while(1) {
        read_code = *(virt_addr); read_data = *(virt_addr);
        if(read_code & 0x0004) {
            continue;
        }
        if(read_code & 0x0002) { 	//non_empty, sop
			yy_i_rd = 0;
			read_buffer[yy_i_rd] = read_data;
		} else if(read_code & 0x0001) {//non_empty, eop
			read_buffer[++yy_i_rd] = read_data;
			break;
		} else {                      //non_empty
			read_buffer[++yy_i_rd] = read_data;
		}
    }
    printf("Send Data Length = %d.\n",yy_i_rd);

    destination_id = read_buffer[0] & 0xff;
    send_addr_1.sin_addr.s_addr = inet_addr(targetIps[destination_id].c_str());  //具体的IP地址
    printf("dest_id: %d, destinatoin IP:%s\nPort:%d\n",destination_id, targetIps[destination_id].c_str(),SduTarget2_Port);

    SduFlagPack_0.sdu_length = 2*(yy_i_rd+1);
    ++SduFlagPack_0.sdu_sn;
    //int num1=sendto(send_socket_1,&SduFlagPack_0,sizeof(SduFlagPack),0,(struct sockaddr*)&send_addr_1,sizeof(send_addr_1));
	int num1=sendto(send_socket_1,&SduFlagPack_0,21+SduFlagPack_0.sdu_length,0,(struct sockaddr*)&send_addr_1,sizeof(send_addr_1));
    //printf("num:%d\n",num1);
	//printf("sdu_sn:%d\n",SduFlagPack_0.sdu_sn);
	//printf("sender_id:%d\n",SduFlagPack_0.sender_id);
	//printf("service_type:%d\n",SduFlagPack_0.service_type);
	//printf("sdu_length:%d\n",SduFlagPack_0.sdu_length);
    int randNumber = rand()%40+10;	
	printf("delay:%d ms\n",randNumber);fflush(stdout);
	usleep(1000*randNumber);

receive:	
    //一次接收
    int num2=recvfrom(recv_socket_1, p, sizeof(SduFlagPack),0,(struct sockaddr*)&send_addr_2,&send_addr_2_size);
    if (p->Flag==($XY)){	
		//printf("detected Data start bit\n");
		//printf("num:%d\n",num2);
		//printf("T:%d\n",p->T);
		//printf("sdu_sn:%d\n",p->sdu_sn);
		//printf("sender_id:%d\n",p->sender_id);
		//printf("service_type:%d\n",p->service_type);
		//printf("sdu_length:%d\n",p->sdu_length);
        //p->data 接收的数组.
        int N = (p->sdu_length);
        int L1 = (N>>1)-1;//Length
        printf("received data length = %d\n", L1);
	    yy_i_wr = 0;
	    *(virt_addr2+4) = write_buffer[yy_i_wr];//sop
	    for(yy_i_wr = 1;yy_i_wr < L1; ++yy_i_wr)	{
	    	*(virt_addr2) = write_buffer[yy_i_wr];
	    }	
	    *(virt_addr2+2) = write_buffer[L1];		//eop
	} else{
		printf("unable to detect the Data start bit\n");
	}
//goto send;

    if(munmap(map_base, MAP_SIZE) == -1) FATAL;
    if(munmap(map_base2, MAP_SIZE) == -1) FATAL;
    close(fd);
    return 0;
}
