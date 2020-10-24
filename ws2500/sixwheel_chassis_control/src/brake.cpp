// 

void send_buf(uint8_t node_id,uint16_t index,uint8_t sub_idx,int32_t val,uint8_t len_cmd)
{
	unsigned char buf[30]={};
	unsigned char counter=0;
	buf[counter++]=0x08;
	buf[counter++]=0x00;
	buf[counter++]=0x00;
	buf[counter++]=0x06;	//rx
	buf[counter++]=node_id;	//Node ID
	buf[counter++]=len_cmd;	//byte length command of val 
	buf[counter++]=uint8_t(index&0x00ff);	//index
	buf[counter++]=uint8_t((index>>8)&0x00ff);
	buf[counter++]=sub_idx;	//sub index
	buf[counter++]=uint8_t(val&0x000000ff); 	//value
	buf[counter++]=uint8_t((val>>8)&0x000000ff); 	//
	buf[counter++]=uint8_t((val>>16)&0x000000ff); 	//
	buf[counter++]=uint8_t((val>>24)&0x000000ff); 	//
	//下发指令
	//send();
}

void brake_init()
{
	//设定控制模式
	send_buf();
	//设定给定速度
	send_buf();
	//设定加速度
	send_buf();
	//设定减速度
	send_buf();
	//设定控制字
	send_buf();
}
void brake_control()
{	
	//状态诊断
	
	//控制指令下发
	send_buf();
	
}
