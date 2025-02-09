#include "Serial.h"

namespace TRoMaC
{
	Uart::Uart() {}

	Uart::~Uart() 
    {
		int a = close(serial_id);
	}

	void Uart::Close() 
    {
		int a = close(serial_id);
	}
    
    int Uart::open_port(int comport) 
    { 
        int fd = 0;
        if (comport == 1)
        {
            fd = open( "/dev/ttyUSB_vision", O_RDWR | O_NOCTTY | O_NDELAY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
        } 
        else if(comport == 2)
        {     
            fd = open( "/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
        } 
        else if (comport == 3)
        { 
            fd = open( "/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
        } 
        if(fcntl(fd, F_SETFL, 0) < 0)
        {
            printf("fcntl failed!\n");
        }
        else 
        {
            printf("fcntl = %d\n", fcntl(fd, F_SETFL, 0));
        }
        if(isatty(STDIN_FILENO) == 0) 
        {
            printf("standard input is not a terminal device\n"); 
        }
        else
        {
            printf("isatty success!\n"); 
        }
        printf("fd-open = %d\n", fd); 
        return fd; 
    }

    int Uart::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) 
    { 
        struct termios newtio, oldtio; 
        if (tcgetattr(fd, &oldtio) != 0)
        {  
            perror("SetupSerial 1");
            printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
            return -1; 
        } 
        bzero(&newtio, sizeof(newtio));  
        newtio.c_cflag  |=  CLOCAL | CREAD;  
        newtio.c_cflag &= ~CSIZE;  
        switch(nBits) 
        { 
            case 7: 
                newtio.c_cflag |= CS7; 
                break; 
            case 8: 
                newtio.c_cflag |= CS8; 
                break; 
        } 
        switch(nEvent) 
        {
            case 'o':
            case 'O':
                newtio.c_cflag |= PARENB; 
                newtio.c_cflag |= PARODD; 
                newtio.c_iflag |= (INPCK | ISTRIP); 
                break; 
            case 'e':
            case 'E': 
                newtio.c_iflag |= (INPCK | ISTRIP); 
                newtio.c_cflag |= PARENB; 
                newtio.c_cflag &= ~PARODD; 
                break;
            case 'n':
            case 'N':
                newtio.c_cflag &= ~PARENB; 
                break;
            default:
                break;
        }
        switch(nSpeed) 
        { 
            case 2400: 
                cfsetispeed(&newtio, B2400); 
                cfsetospeed(&newtio, B2400); 
                break; 
            case 4800: 
                cfsetispeed(&newtio, B4800); 
                cfsetospeed(&newtio, B4800); 
                break; 
            case 9600: 
                cfsetispeed(&newtio, B9600); 
                cfsetospeed(&newtio, B9600); 
                break; 
            case 115200: 
                cfsetispeed(&newtio, B115200); 
                cfsetospeed(&newtio, B115200); 
                break; 
            case 460800: 
                cfsetispeed(&newtio, B460800); 
                cfsetospeed(&newtio, B460800); 
                break; 
            case 921600: 
                cfsetispeed(&newtio, B921600); 
                cfsetospeed(&newtio, B921600); 
                break; 
            case 1000000: 
                cfsetispeed(&newtio, B1000000); 
                cfsetospeed(&newtio, B1000000); 
                break; 
            default: 
                cfsetispeed(&newtio, B9600); 
                cfsetospeed(&newtio, B9600); 
                break; 
        }
        if(nStop == 1)
        {
            newtio.c_cflag &=  ~CSTOPB;
        }
        else if (nStop == 2)
        {
            newtio.c_cflag |=  CSTOPB; 
        }
        newtio.c_cc[VTIME] = 1; 
        newtio.c_cc[VMIN] = READ_DATA_NUM;
        tcflush(fd, TCIFLUSH); 
        
        
        if((tcsetattr(fd, TCSANOW, &newtio)) != 0) 
        { 
            perror("com set error"); 
            return -1; 
        } 
        printf("set done!\n"); 
        return 0; 
    } 

	bool Uart::Open(int comport, int _speed) 
    {
        int fd = open_port(comport);
        if(fd < 0)
        {
            perror("open_port error"); 
            return false; 
        }
        if((set_opt(fd, _speed, 8, 'N', 1)) < 0)
        {
            perror("set_opt error"); 
            return false; 
        } 
        serial_id = fd;
        return true; 
	}

    bool Uart::checkSerial() 
    {
        struct termios oldtio; 
        if(tcgetattr(serial_id, &oldtio) == 0)
        {  
            return true;
        } 
        serial_id = -1;
        return false;
    }
    
	bool Uart::ReadData() 
    {
		int readsize = read(serial_id, data, READ_DATA_NUM);
        tcflush(serial_id, TCIFLUSH);

        if (readsize != READ_DATA_NUM && readsize != 0) 
        {
			std::cout << "READ ERROR:" << readsize << std::endl;
			return false;
		}
		else if(readsize == 0)
        {
            std::cout << "get the data error" << std::endl;
            return false;
        }

//下位机给我发什么东西
        read_data.CheckID = lptmp -> CheckID;
        read_data.Exposure = lptmp -> Exposure;
        read_data.LossPackage = lptmp -> LossPackage;
        read_data.Switch = lptmp -> Switch;
        read_data.EndFrame = lptmp -> EndFrame;
        // read_data.Ene_OutPost_HP = lptmp -> Ene_OutPost_HP;
        // read_data.Remain_Time = lptmp -> Remain_Time;
        // read_data.Digital_Recog_Switch = lptmp -> Digital_Recog_Switch;
        // read_data.Beat_Hero_or_Engineer = lptmp -> Beat_Hero_or_Engineer;
        // read_data.Resurrection_Count_Down = lptmp -> Resurrection_Count_Down;
        // read_data.My_Position_x = lptmp -> My_Position_x;
        // read_data.My_Position_y = lptmp -> My_Position_y;

        return true;
	}
//我给下位机发什么东西
	void Uart::send(const VisionData& data)
	{
        TXunion.VisionFrameTX.x = 3.14;//data.x;
        TXunion.VisionFrameTX.y = 2.54;//data.y;
        TXunion.VisionFrameTX.z = 1.23;//data.z;
        TXunion.VisionFrameTX.x_axis_angle = 0.5;//data.x_axis_angle;
        TXunion.VisionFrameTX.y_axis_angle = 0.5;//data.y_axis_angle;
        TXunion.VisionFrameTX.z_axis_angle = 1.23;//data.z_axis_angle;
        TXunion.VisionFrameTX.a = 0x00;
        TXunion.VisionFrameTX.b = 0x00;
        TXunion.VisionFrameTX.c = 0x80;
        TXunion.VisionFrameTX.d = 0x7f;
        // TXunion.VisionFrameTX.Robot_x = data.robot_x;
        // TXunion.VisionFrameTX.Robot_y = data.robot_y;
        // TXunion.VisionFrameTX.Robot_ID = data.robot_id;
        // TXunion.VisionFrameTX.IsHeroFlag = data.isheroflag;

		auto write_stauts = write(serial_id, &TXunion.u8arr[0], SEND_DATA_NUM);

        tcflush(serial_id, TCOFLUSH);
        
		if (write_stauts != SEND_DATA_NUM) 
        { 
			std::cout << "send error! the length of data is: " << write_stauts << std::endl;
		}
	}
}
