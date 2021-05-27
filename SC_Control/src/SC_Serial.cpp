#include "SC_Serial.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <bitset>

using namespace std;
serial::Serial ser;

int alive = 0;




bool valid()
{
    if(ser.isOpen())
        return true;
    else
        return false;
}

bool connect(std::string port_name, int baudrate)
{
    if(valid())
        return false;
    try
    {
        ser.setPort(port_name);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
         ROS_INFO("open port ");
        return true;

    }
    catch (serial::IOException& e)
    {
         ROS_INFO("CAN'T OPEN PORT ");
	ros::shutdown();
        return false;
    }
}

SC_Serial::SC_Serial(std::string port_name, int baudrate)
{
    connect(port_name, baudrate);

   // SerialProcessing(0, 0,1,0);
    Tx_Vel = 0;
    Tx_Steer = 0;
    Tx_break = 1;
    Tx_gear = 0;

    pub_vel = node_.advertise<std_msgs::Int16>("/MSG_CON/Rx_Vel",1);
    pub_steer = node_.advertise<std_msgs::Int16>("/MSG_CON/Rx_Steer",1);
    pub_enc = node_.advertise<std_msgs::Int32>("/MSG_CON/Rx_Enc",1);


    Rx_Vel.data = 0;
    Rx_Steer.data = 0;
     Rx_Enc.data = 0;
}

void SC_Serial::SendData(void)
{
    // STX
    input[0]=0x53;
    input[1]=0x54;
    input[2]=0x58;

    // Data
    input[3]=0x01;      // A or M
    input[4]=0x00;      // E-Stop

    input[5] = this->Tx_gear & 0xFF;           //0x0;      // Gear : 0~2


    input[6]=(this->Tx_Vel >> 8) & 0xFF;     // Speed0
    input[7]= this->Tx_Vel & 0xFF;            // Speed1

    input[8] =(this->Tx_Steer >> 8) & 0xFF;
    input[9] = this->Tx_Steer & 0xFF;

    input[10]= this->Tx_break & 0xFF;                     //0x01;     // Break : 1 - 33
    //std::cout << "Tx_break : " << Tx_break << "  input[10]: " << (short)input[10] << std::endl;

    input[11]=(BYTE)alive;     // Alive

    input[12]=0x0D;     // ETX0
    input[13]=0x0A;     // ETX1

    string sersend = "";

    //cout << "Send Data" << endl;
    for(int i=0; i<14; i++)
    {
        sersend += input[i];
        //cout << hex << "Tx[" << i << "] : " << (int)input[i] << endl;
    }
    ser.write(sersend);

    alive++;
    if(alive % 255 == 0)
        alive = 0;
}


void SC_Serial::ReceiveData(std::string str)
{
    if(str[0]=='S')
    {
        int steer_full =0;

        printf("vel:\n");
        printf("%d \n",(unsigned char)str[6]);
        printf("steer:\n");
        printf("%X \n",str[8]);
        printf("%X \n",str[9]);
        printf("break:\n");
        printf("%d \n",str[10]);

        printf("enc:\n");
        printf("%X \n",(unsigned char)str[11]);
        printf("%X \n",(unsigned char)str[12]);
        printf("%X \n",(unsigned char)str[13]);
        printf("%X \n",(unsigned char)str[14]);


        Rx_Vel.data =(unsigned char)str[6];//(str[6] & 0xff)+(str[7] <<8); //((str[6] & 0xf0U) >> 4)*16 + (str[6] & 0x0fU);
        Rx_Steer.data = ((~str[9] & 0xf0U) >> 4)*4096 + (~str[9] & 0x0fU)*256 + ((~str[8] & 0xf0U) >> 4)*16 + (~str[8]& 0x0fU)+1;

    int b,c,d,e;
    b=(str[11] & 0xffU);
    c=(str[12] & 0xffU);
    d=(str[13] & 0xffU);
    e=(str[14] & 0xffU);
    Rx_Enc.data =( e<<24) + (d<<16) + (c<<8) +b;
    printf("INPutvel : %d , Rx_vel :%d    break : %d\n",this->Tx_Vel,Rx_Vel.data,str[10]);
    printf("Rx_Steer :%d      input_steering : %d   \n",Rx_Steer.data, this->Tx_Steer);
    printf("Rx_Enc :%d\n",Rx_Enc.data);







        pub_vel.publish( Rx_Vel);

       // pub_steer.publish( Rx_Steer);
       // pub_enc.publish( Rx_Enc);
    }
}

void SC_Serial::SerialProcessing(short Tx_Vel, short Tx_Steer, short Tx_break, short Tx_gear)
{
    string result1;
//    ros::spinOnce();


    if(ser.available()){
        this->Tx_Vel = Tx_Vel;
        this->Tx_Steer = Tx_Steer;
        this->Tx_break = Tx_break;
        this->Tx_gear = Tx_gear;
        SendData();
      //  cout << "process this " << this->Tx_break << "input break" << Tx_break <<  endl;

        ser.readline(result1);
        ReceiveData(result1);
    }//else
       // ROS_INFO("NO SERIAL DATA ");



}
int SC_Serial::HexStringToDec(char A , char B )
{
    //16진수로 된 char 배열 값을 정수로 변경하여 반환한다
    cout <<"A:" << A << " B: " << B <<  endl;
    int sum = 0;
    int temp = 0;

    sum = sum*16 + (A & 0xf0U) >> 4;
    sum = sum*16 + (A & 0x0fU);

    sum = sum*16 + (B & 0xf0U) >> 4;
    sum = sum*16 + (B & 0x0fU);

    return sum;
}





