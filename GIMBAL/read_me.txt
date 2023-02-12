云台
烧录到新云台的C版中,还没有加入USB虚拟串口
		DR16.rc.ch0 -= 1024;
		把这句话放前面一点会好一些
		const TickType_t TimeIncrement = pdMS_TO_TICKS(3); //每3毫秒强制进入总控制  实测3毫秒就不会出问题  还是会出,但概率小很多

在之前的原理介绍中，改变 PWM 的频率就可以改变无源蜂鸣器的音调。故而改变定时器的
分频系数和重载值，改变 PWM 的频率，就能够控制无源蜂鸣器发出的响声频率。
在主程序中，声明了 psc 和 pwm 两个变量，分别控制定时器 4 的分频系数和重载值，每一
次循环中这两个变量进行一次自加。通过 宏 定 义 的 方 式 ， 设 置 pwm 的值在
MIN_BUZZER_PWM（10000）和 MAX_BUZZER_PWM（20000）之间变动，psc 的值在
0 和 MAX_PSC（1000）之间变动，程序主流程如图所示

报错:

云台:帧率异常101 pitch 102yaw 103both,堵转,温度异常,离线,

发射:超速,空仓,卡弹,拨盘,堵转  电机离线,温度异常  视觉开火指令异常

底盘:边界值异常(编码器跳动)  底盘电机温度异常

spinning top小陀螺   examine 检测
//低音
//1 - do - 262Hz
//2 - re - 294Hz
//3 - mi - 330Hz
//4 - fa - 349Hz
//5 - so - 392Hz
//6 - la - 440Hz
//7 - si - 494Hz
//中音
//1 - do - 523Hz
//2 - re - 587Hz
//3 - mi - 659Hz
//4 - fa - 698Hz
//5 - so - 784Hz
//6 - la - 880Hz
//7 - si - 988Hz
//高音
//1 - do - 1046Hz
//2 - re - 1175Hz
//3 - mi - 1318Hz
//4 - fa - 1397Hz
//5 - so - 1568Hz
//6 - la - 1760Hz
//7 - si - 1976Hz


unsigned          char   1个字节（1字节=8个位）    8个字节
float占几个字节      4 个字节

typedef   signed short     int int16_t;            两个字节

		int16_t ch0;//yaw
		int16_t ch1;//pitch
		int16_t ch2;//left_right
		int16_t ch3;//forward_back


0708  任务运行时间统计:
Debug_Task     	5814		<1%
led            	5804		<1%
IDLE           	885108		27%
IMU_Send_Task  	1764361		54%
cali           	157311		4%
test           	554		<1%
Task_Robot_Cont	350312		10%
Tmr Svc        	0		<1%
imuTask        	62044		1%
Can2_Rei_Task  	1		<1%
CAN1           	0		<1%

注释掉视觉串口发送后:
任务名\t\t\t运行时间\t运行所占百分比		

Debug_Task     	114		<1%
IDLE           	70208		78%
cali           	8311		9%
led            	46		<1%
IMU_Send_Task  	58		<1%
test           	4		<1%
Task_Robot_Cont	9573		10%
CAN1           	1		<1%
imuTask        	1570		1%
Tmr Svc        	0		<1%
Can2_Rei_Task  	0		<1%

使用DMA视觉串口发送后:
任务名\t\t\t运行时间\t运行所占百分比
Debug_Task     	571		<1%
IDLE           	222460		76%
cali           	29245		10%
led            	405		<1%
IMU_Send_Task  	334		<1%
test           	49		<1%
Task_Robot_Cont	31416		10%
Tmr Svc        	0		<1%
imuTask        	5524		1%
Can2_Rei_Task  	1		<1%
CAN1           	0		<1%



校准关节电机的角度;
左下右下  摇杆内8向下:将当前角度写入flash


左下右下  摇杆外8向上:从flash读取角度作为抬最高时的角度






参数的坟场:
MIT电机比较软:
	MIT_A.kp_temp=15;
	MIT_A.kv_temp=2.3;
	
	MIT_B.kp_temp=15;
	MIT_B.kv_temp=2.3;
	
	MIT_C.kp_temp=15;
	MIT_C.kv_temp=2.3;
	
	MIT_D.kp_temp=15;
	MIT_D.kv_temp=2.3;









