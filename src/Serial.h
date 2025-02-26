#ifndef SERIAL_H_
#define SERIAL_H_

#include     <stdio.h>     
#include     <stdlib.h>     
#include     <unistd.h>     
#include     <sys/types.h> 
#include     <sys/stat.h>  
#include     <fcntl.h>      
#include     <termios.h>    
#include     <errno.h>      
#include   	 <string.h>       
#include 	 <iostream>
#include 	 <memory>


// #define SEND_DATA_NUM (16)
// #define READ_DATA_NUM (19)

#define SEND_DATA_NUM (49) 
#define READ_DATA_NUM (24)

namespace TRoMaC
{
	#pragma pack(1)

    typedef struct //这里预留写按钮兑矿或者啥的
    {   
        uint8_t     CheckID;        // 机器人ID
        uint8_t     Exposure;       // 视觉模式
        uint8_t     LossPackage;    // 丢包率   
        uint8_t     Switch;         // 预留调节曝光的一个字节
        uint8_t     EndFrame;       // 帧尾
    } VisionFrameRX_structTypedef;
    
    /* ==== 云台接收帧结构体&联合体 ==== */

    typedef struct 
    {   
        float       x;              //角点x坐标
        float       y;              //角点y坐标
        float       z;              //角点z坐标

        float       X_x;            //兑换槽水平方向向量x坐标
        float       X_y;                     
        float       X_z;
                 
        float       Y_x;            //兑换槽竖直方向向量x坐标
        float       Y_y;
        float       Y_z;

        float       Z_x;            //垂直兑换槽方向向量x坐标
        float       Z_y;
        float       Z_z;           

        uint8_t     pumpSwitch;     //气泵开关
        //uint8_t     a,b,c,d;

    } VisionFrameTX_structTypedef;  
    
    typedef union
 	{
    	VisionFrameTX_structTypedef     VisionFrameTX;
    	uint8_t                         u8arr[sizeof(VisionFrameTX_structTypedef)];
    } VisionFrameTX_unionTypeDef;
    
    typedef union
 	{
    	VisionFrameRX_structTypedef     VisionFrameRX;
    	uint8_t                         u8arr[sizeof(VisionFrameRX_structTypedef)];
    }  VisionFrameRX_unionTypeDef;
    
    //用于保存目标相关角度和距离信息及瞄准情况
	typedef struct
	{
        float       x;              //角点x坐标
        float       y;              //角点y坐标
        float       z;              //角点z坐标

        float       X_x;            //兑换槽水平方向向量x坐标
        float       X_y;                     
        float       X_z;
                 
        float       Y_x;            //兑换槽竖直方向向量x坐标
        float       Y_y;
        float       Y_z;

        float       Z_x;            //垂直兑换槽方向向量x坐标
        float       Z_y;
        float       Z_z;  

        uint8_t     pumpSwitch;     //气泵开关
    } VisionData;
	
    typedef struct
	{
        uint8_t     CheckID;        // 机器人ID
        uint8_t     Exposure;       // 视觉模式
        uint8_t     LossPackage;    // 丢包率   
        uint8_t     Switch;         // 预留调节曝光的一个字节
        uint8_t     EndFrame;       //帧尾								
    } ReceiveData;
    
    #pragma pack()

	class Uart 
	{
	public:
		Uart();
		~Uart();
        void Close();
        
        int open_port(int comport);

        int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

		bool Open(int comport, int _speed);

        bool checkSerial();

        bool ReadData();
        
		void send(const VisionData& data);
		
        int serial_id;
        int speed;
		const char* uart_path;
        unsigned char data[30];

        VisionFrameRX_structTypedef read_data;
        VisionFrameRX_structTypedef *lptmp = (VisionFrameRX_structTypedef *)data;
        VisionFrameTX_unionTypeDef TXunion;
        
	private:
	};
}

#endif
