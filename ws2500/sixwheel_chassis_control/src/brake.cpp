//
union uchar_int16
{
	unsigned char data[2];
	int16_t val;
}; 
union uchar_int32
{
	unsigned char data[4];
	int32_t val;
};
void send_buf(uint8_t node_id,,uint8_t len_cmd,uint16_t index,uint8_t sub_idx,int32_t val)
{
	unsigned char buf[30]={};
	unsigned char counter=0;
	union uchar_int16 temp_idx;
	union uchar_int32 temp_val;
	temp_idx.val = index;
	temp_val.val = val;
	 
	buf[counter++]=0x08;
	buf[counter++]=0x00;
	buf[counter++]=0x00;
	buf[counter++]=0x06;	//rx
	buf[counter++]=node_id;	//Node ID
	buf[counter++]=len_cmd;	//byte length command of val 
//	buf[counter++]=(uint8_t)(index&0x00ff);	//index
//	buf[counter++]=(uint8_t)((index>>8)&0x00ff);
	buf[counter++]=temp_idx.data[0];	//index
	buf[counter++]=temp_idx.data[1];
	buf[counter++]=sub_idx;	//sub index
	buf[counter++]=temp_val.data[0]; 	//value
	buf[counter++]=temp_val.data[1]; 	//
	buf[counter++]=temp_val.data[2]; 	//
	buf[counter++]=temp_val.data[3]; 	//
	
//	buf[counter++]=(uint8_t)(val&0x000000ff); 	//value
//	buf[counter++]=(uint8_t)((val>>8)&0x000000ff); 	//
//	buf[counter++]=(uint8_t)((val>>16)&0x000000ff); 	//
//	buf[counter++]=(uint8_t)((val>>24)&0x000000ff); 	//
	//下发指令
	//send();
}

void brake_init()
{
	//设定控制模式，轮廓位置模式
	send_buf(1,0x2F,0x6060 ,0x00,1);
	//设定给定速度，-3000～3000rpm
	send_buf(1,0x23,0x607A,0x00,15);
	//设定加速度 rpm/ms
	send_buf(1,0x23,0x6083,0x00,15);	
	//设定减速度 rpm/ms
	send_buf(1,0x23,0x6084,0x00,15);
	//设定控制字
	//给使能	
	send_buf(1,0x2B，0x6040,0x00 ,0x06);
	//警报清除
	send_buf(1,0x2B，0x6040,0x00 ,0x07);
}
void analyse_brake(unsigned char *rec_data，unsigned char len)
{
	
}
void brake_control()
{	
	//状态诊断
	analyse_brake();	
	//新的控制指令下发
	send_buf();
	//	
}
