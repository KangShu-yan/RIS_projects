# 使用大疆电机3508 P19 配C620电调 对广成科技的CANNET-II 进行了测试
大疆电机上传频率 1000hz
大疆电机CAN网络 比特率 1000kbps

# ros_rate		got_can_frame_counter   short_frame_counter
	
	2000hz            5189                      112
	
	1000hz			  5386						50
	
	500hz			  5603						37 
	
#按测试情况看，只要解析处理得当，tcp2can（CANNET-II）可以接受高速转换
