#include <msp430.h>
#define highSpeed 115//80
#define lowSpeed 27//20
#define low 50//这三者决定了占空比和PWM波频率
int delay_time = 500;//命令值设为全局变量
int command = 7;//前进后退左转右转
int old_command = 1;
int old_direction = 1;
int speed = 0;//控制了速度档位(占空比
const unsigned int high[4]={highSpeed,lowSpeed,0,16};//0-2分别对应高速低速和停止
unsigned int runSpeed;
unsigned int i=0;
char buffer[125];
char message[8];
int map[10][10];
//对手位置
int ene_x = 0;
int ene_y = 0;
//小车所处的位置
int barrier_num = 0;
int times = 0;
int times_sec = 0;
int time_flag = 0;
int main_way = 0;
int x = 0;
int y = 9;
int direction = 0;//direction 4 进制，0为x正半轴，3为y正半轴，左转+3%4,右转+5%4
//堡垒和驱逐舰
const int bar[5][5][2] = {{{0,0},{0,1},{1,0},{0,-1},{-1,0}},{{0,1},{1,1},{-1,1},{-1,0},{-1,-1}},{{-1,1},{0,1},{1,1},{1,0},{1,-1}},
{{-1,-1},{0,-1},{1,-1},{1,0},{1,1}},{{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
int right_way;//前方右转弯后的路况
int left_way;//前方左转弯后的路况
int x_turn = 0,y_turn = 9,turn_flag = 0;
int flag_left = 0, flag_right = 0;
void delay();
void run();
void initPWM();
void initUARTA0();
void initIrr();
void initClock();
char judgeState();
void send(int x, int y, int type);
void judge_turn_point();
int main(void){

	WDTCTL = WDTPW | WDTHOLD;// stop watchdog timer
	initPWM();//小车PWM的引脚设置
	initUARTA0();//通讯设置
	initIrr();//通讯中断设置
	initClock();
	//调试
	old_command = 1;
	command = 7;
	speed = 0;
	direction = 1;
	old_direction = 1;
	while(1){
		run();//小车的运行
	}
}
void initPWM(){

	P2SEL &= ~(BIT1+BIT2+BIT3+BIT4);//P2.1-2.4輸出，按顺序对应AIN1,AIN2,PWMA,PWMB
	P2SEL2 &= ~(BIT1+BIT2+BIT3+BIT4);
	P2OUT |= (BIT1+BIT2+BIT3+BIT4);
	P2DIR |= (BIT1+BIT2+BIT3+BIT4);//设置输出方向
}
void initUARTA0(){
	UCA0CTL1 |= UCSWRST;//置软件复位位swrst为1
	P1SEL |= BIT1+BIT2;//置P1.1、P1.2为串行接口收、发引脚功能
	P1SEL2 |= BIT1+BIT2;
	//数据格式选用上电复位设置：无校验，8位数据，1个停止位，异步串行通信
	UCA0CTL1|=UCSSEL0+UCRXEIE; //波特率时钟选择上电复位时的ACLK，32.768KHz，对错均收
	UCA0BR0 =3;//波特率9600
	UCA0BR1 = 0;
	UCA0MCTL = UCBRF_0+UCBRS_3;
	UCA0CTL1 &= ~UCSWRST;        //置软件复位位swrst为0，串口设置完毕
}
void initIrr(){
	//传感器中断P1.0
	//中断引脚的相关设置
	P1SEL &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1SEL2 &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1OUT |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1REN |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1DIR &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IES |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IFG &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IE &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IE |= BIT0;
	//通讯中断
	IE2 |= UCA0RXIE;//允许USCI_A0 的接收中断
	_EINT();//总中断允许
}

void initClock(){
    BCSCTL1 |= DIVA_0; //ACLK采用外部晶振，1分频
    TA0CCTL0=CCIE; //设定定时器CCR0中断使能
    TA0CTL=TASSEL_1+MC_1; //选择ACLK时钟源, UP模式
    TA0CCR0= 328; //设定计数值 32768*1/32768=1s计数周期
    _EINT(); //开中断
    times = 1300;//防止开始进入中断
    time_flag = 0;
 //   LPM0;
}
void delay(unsigned int i_temp){

	for(;i_temp>0;i_temp--);
}
void run(){
	//command=9;
	runSpeed = high[speed];//占空比(延迟函数传参)
	if(command == 1){//前进
	    if(((P1IN&BIT4) == 0) && ((P1IN&BIT6) != 0)){
	        P2OUT |= BIT1;
	        P2OUT &= ~BIT2;
	        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
	        if(speed == 0){
	            delay(50);//高电平持续时间
	            P2OUT &= ~BIT3;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	            delay(30);//低电平持续时间
	            P2OUT &= ~BIT4;
	            delay(65);//低电平持续时间
	        }
	        else if(speed == 1 || speed == 3){
	            delay(50);//高电平持续时间
	            P2OUT &= ~BIT3;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	            delay(50);//低电平持续时间
	            P2OUT &= ~BIT4;
	            delay(100);//低电平持续时间
	        }
	    }
	    else if(((P1IN&BIT4) != 0) && ((P1IN&BIT6) == 0)){
	        P2OUT |= BIT1;
	        P2OUT &= ~BIT2;
	        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
	        if(speed == 0){
	            delay(50);//高电平持续时间
	            P2OUT &= ~BIT4;//调整PWMA为低电平，做到PWMB的占空比大于PWMA
	            delay(30);//低电平持续时间
	            P2OUT &= ~BIT3;
	            delay(65);//低电平持续时间
	        }
	        else if(speed == 1 || speed == 3){
	            delay(50);//高电平持续时间
	            P2OUT &= ~BIT4;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	            delay(50);//低电平持续时间
	            P2OUT &= ~BIT3;
	            delay(150);//低电平持续时间
	        }
	    }
	    else{//两黑或两白，看最边上的
	        if(((P1IN&BIT3) != 0) && ((P1IN&BIT7) == 0)){
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
	            if(speed == 0){
	                delay(50);//高电平持续时间
	                P2OUT &= ~BIT4;//调整PWMA为低电平，做到PWMB的占空比大于PWMA
	                delay(30);//低电平持续时间
	                P2OUT &= ~BIT3;
	                delay(65);//低电平持续时间
	            }
	            else if(speed == 1 || speed == 3){
	                delay(50);//高电平持续时间
	                P2OUT &= ~BIT4;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	                delay(50);//低电平持续时间
	                P2OUT &= ~BIT3;
	                delay(150);//低电平持续时间
	            }
	        }
	        else if(((P1IN&BIT3) == 0) && ((P1IN&BIT7) != 0)){
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
	            if(speed == 0){
	                delay(50);//高电平持续时间
	                P2OUT &= ~BIT3;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	                delay(30);//低电平持续时间
	                P2OUT &= ~BIT4;
	                delay(65);//低电平持续时间
	            }
	            else if(speed == 1 || speed == 3){
	                delay(50);//高电平持续时间
	                P2OUT &= ~BIT3;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
	                delay(50);//低电平持续时间
	                P2OUT &= ~BIT4;
	                delay(100);//低电平持续时间
	            }
	        }
	        else{
	            P1IFG &= ~BIT5;
	            P1IE &= ~BIT5;//关中间中断
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);
	            delay(runSpeed);
	            P2OUT &= ~(BIT3+BIT4);
	            delay(low);
	        }
	    }
	}
	else if(command == 2){//后退
		P2OUT |= BIT2;
		P2OUT &= ~BIT1;
		P2OUT |= (BIT3+BIT4);
		delay(runSpeed);
		P2OUT &= ~(BIT3+BIT4);
		delay(low);
	}
	if(command == 5){//原地左转
		P2OUT |= BIT1;//右轮前转
		P2OUT &= ~BIT2;
		P2OUT |= BIT4;
		P2OUT &= ~BIT3;
		delay(lowSpeed*2);
		P2OUT &= ~BIT4;
		delay(low);
		//左轮后转
		P2OUT |= BIT2;
		P2OUT &= ~BIT1;
		P2OUT |= BIT3;
		P2OUT &= ~BIT4;
		delay(lowSpeed*2);
		P2OUT &= ~BIT3;
		delay(low);
		P2OUT |= BIT1;
//		            P2OUT &= ~BIT2;
//		            P2OUT |= (BIT3+BIT4);
//		            P2OUT &= ~BIT3;
//		            delay(20);
//		            P2OUT &= ~BIT4;
//		            delay(low);//低电平持续时间
	}
	if(command == 6){//原地右转
		P2OUT |= BIT1;//左轮前传
		P2OUT &= ~BIT2;
		P2OUT |= BIT3;
		P2OUT &= ~BIT4;
		delay(lowSpeed*2);
		P2OUT &= ~BIT3;
		delay(low);
		//右轮后转
		P2OUT |= BIT2;
		P2OUT &= ~BIT1;
		P2OUT |= BIT4;
		P2OUT &= ~BIT3;
		delay(lowSpeed*2);
		P2OUT &= ~BIT4;
		delay(low);
		P2OUT |= BIT1;
//		            P2OUT &= ~BIT2;
//		            P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
//		            P2OUT &= ~BIT4;
//		            delay(20);//占空比95%
//		            P2OUT &= ~BIT3;
//		            delay(low);//低电平持续时间
	}
	if(command == 7){
		P2OUT &= ~(BIT3+BIT4);//制动
		delay(low);
	}
	if(command == 8){//前进右转
		int cnt = 0;
		if(flag_right == 0)cnt = delay_time;//延迟时间
		else cnt = 1;
		while(cnt > 0){
			P2OUT |= BIT1;
			P2OUT &= ~BIT2;
			P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
			P2OUT &= ~BIT4;
			delay(18);//占空比95%//20
			P2OUT &= ~BIT3;
			delay(low);//低电平持续时间
			flag_right = 1;
			P1IFG &= ~BIT5;
			if(cnt == 1)P1IE |= BIT5;
			cnt--;
		}
	}
	if(command == 9){//前进左转
        int cnt = 0;
        if(flag_left == 0)cnt = delay_time;//延迟时间
        else cnt = 1;
		while(cnt > 0){
			P2OUT |= BIT1;
			P2OUT &= ~BIT2;
			P2OUT |= (BIT3+BIT4);
			P2OUT &= ~BIT3;
			delay(18);//20
			P2OUT &= ~BIT4;
			delay(low);//低电平持续时间
			flag_left = 1;
            P1IFG &= ~BIT5;
			if(cnt == 1)P1IE |= BIT5;
			cnt--;
		}
	}
}
char judgeState(){//用于解析buffer数组，判断上机位的指令
	if((buffer[0] >= 'A' && buffer[0] <= 'Z')||(buffer[0] >= 'a' && buffer[0] <= 'z'))return buffer[0];//字母直接返回
	//训练赛解析:
	if(buffer[0] >= '0' && buffer[0] <= '9' && buffer[3] != '&'){
		x_turn = buffer[0] - '0';
		y_turn = 9;
	}
	if(buffer[0] >= '0' && buffer[0] <= '9' && buffer[3] == '&'){
	    unsigned int g,h;
	    for(g=0;g<10;g++)
	        for(h=0;h<10;h++)map[g][h] = 0;//初始化
	    x = buffer[0] - '0';
	    y = buffer[2] - '0';
	    unsigned int m = 4;
	    while(buffer[4] != '&'){//资源点不为空
	        map[buffer[m] - '0'][buffer[m+2] - '0'] = 1;//资源
	        m += 4;
	        if(buffer[m-1] == '&')break;
	    }
	    if(buffer[4] == '&'){//资源点为空
	        ene_x = buffer[5];
	        ene_y = buffer[7];
	        unsigned int j = 9;
	        while(buffer[9] != '\n'){//障碍物不为空
	            int type_bar = buffer[j+4] - '0' - 1;//堡垒类型
	            int n = 0;
	            for(n = 0; n<5; n++){
                    int x_tmp = buffer[j] - '0' + bar[type_bar][n][0];
                    int y_tmp = buffer[j+2] - '0' + bar[type_bar][n][1];
                    if(x_tmp >=0 && x_tmp <= 9 && y_tmp >=0 && y_tmp <= 9) map[x_tmp][y_tmp] = 2 ;//2为障碍物
	            }
	            j += 6;
	            if(buffer[j-1] == '\n')break;
	        }
	    }
	    else if(buffer[4] != '&'){
            ene_x = buffer[m] -'0';
            ene_y = buffer[m+2] -'0';
            int u = ene_x - 1 , v = ene_y - 1;
            for(u = ene_x - 1;u <= ene_x + 1;u++)
                for(v = ene_y - 1; v <= ene_y + 1;v++)
                    if(u>=0 && v>=0 && u<=9 && v<=9) map[u][v] = 2;//对手3*3内不能通过
            unsigned int j = m+4;
            while(buffer[m+4] != '\n'){//障碍物不为空
                int type_bar = buffer[j+4] - '0' - 1;//堡垒类型
                int n = 0;
                for(n = 0; n<5; n++){
                    int x_tmp = buffer[j] - '0' + bar[type_bar][n][0];
                    int y_tmp = buffer[j+2] - '0' + bar[type_bar][n][1];
                    if(x_tmp >=0 && x_tmp <= 9 && y_tmp >=0 && y_tmp <= 9) map[x_tmp][y_tmp] = 2 ;//2为障碍物
                }
                j += 6;
                if(buffer[j-1] == '\n' || buffer[j-1] == '\r')
                    break;
            }
	    }
//        if(x == 0 && y == 9)direction = 1;
//        if(x == 9 && y == 0)direction = 2;
	    //特判
	    if((x == 8 && y == 3)||(x == 7 && y == 3))map[1][3] = 2;
	    judge_turn_point();
	    speed = 0;//修改speed
	    if(direction == 0 && x == x_turn - 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 1 && x == x_turn && y == y_turn + 1)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 2 && x == x_turn + 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 3 && x == x_turn && y == y_turn - 1)speed = 1/*,send(x,y,direction+4)*/;
	    else speed = 0;
	    if(main_way == 0){//堵死
	        if(direction == 0){
	            if(y<5){
	                old_command = 5;
	                direction = (direction + 3) % 4;
	            }
	            else {
	                old_command = 6;
	                direction = (direction + 5) % 4;
	            }
                judge_turn_point();
	        }
	        else if(direction == 1){
                if(x<5){
                    old_command = 5;
                    direction = (direction + 3) % 4;
                }
                else {
                    old_command = 6;
                    direction = (direction + 5) % 4;
                }
                judge_turn_point();
            }
	        else if(direction == 2){
                if(y>5){
                    old_command = 5;
                    direction = (direction + 3) % 4;
                }
                else {
                    old_command = 6;
                    direction = (direction + 5) % 4;
                }
                judge_turn_point();
            }
	        else if(direction == 3){
                if(x>5){
                    old_command = 5;
                    direction = (direction + 3) % 4;
                }
                else {
                    old_command = 6;
                    direction = (direction + 5) % 4;
                }
                judge_turn_point();
            }
	        P1IFG &= ~(BIT0 + BIT5);
	        P1IE &= ~BIT0;//关闭该中断(转弯)
	        P1IE |= BIT5;
	    }
	    barrier_num = 0;
	}
	return 'a';
}

void send(int x, int y, int type){
	message[0] = x + '0';
	message[1] = 32;
	message[2] = y + '0';
	message[3] = 32;
	message[4] = type + '0';
	message[5] = '\n';
	unsigned int k = 0;
	while(1)
	{
		if(message[k]!='\n') {     //输出提示信息
			while((IFG2&UCA0TXIFG)==0);     //检测发送缓冲是否空
			    UCA0TXBUF=message[k];       //取一个数据发送
		}
		else {
		    while((IFG2&UCA0TXIFG)==0);     //检测发送缓冲是否空
		        UCA0TXBUF=message[5];
		    break;
		}
		k++;
	}
}

void judge_turn_point()//更新转弯点
{
    turn_flag = 0;
    main_way = 0;//前方主路路况,其值表示策略的优先
    int p;
    //0表示有障碍物阻挡 1表示什么都没有 2表示先有一个资源点然后有一个障碍物阻挡或者只有一个资源点 3表示有两个或以上资源点
    if (direction == 0)
    {
        y_turn = y;
        int obstacle_point = 10;
        //int sourse_point;//main_way为2时表示资源点坐标；
        //int none_flag = 0;
        int x_judge;
        for (x_judge = x + 1; x_judge < 10; x_judge++)//判断主路路况
        {
            if (map[x_judge][y] == 1)//发现资源点
            {
                if (main_way == 2)//如果已经有了一个资源点
                {
                    main_way = 3;//赋予主路最高的优先级，并且跳出循环
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x_judge][y] == 2)//检测到障碍物
            {
                if (main_way != 2)
                {
                    main_way = 0;//赋予主路最低的优先级，将选择转弯的策略
                }
                obstacle_point = x_judge;//记录主路上第一个障碍物的位置
                break;
            }
            else if (x_judge == 9)//当遍历到主路最后一格时
            {
                if (main_way != 2) main_way = 1;//判断这条路上啥也没有，并将策略优先级赋为1
            }
        }
        x_turn = obstacle_point - 1;//设置保底转弯点，防止小车到出界或触碰障碍物
        if (main_way == 1)//如果主路啥也没有，先让小车尽早转弯，去寻找新的机会，如果检索到其他支路有资源的话后续会更改策略
        {
            main_way = 0;//将主路赋为0，以便让支路啥也没有的话优先级也高于主路，从而实现小车尽快转弯
            x_turn = x + 1;//下一个点就转
        }
        //if (y < 5) turn_flag = 1;//哪边宽阔往哪边转，以便寻找到更多的资源
        //else turn_flag = 2;
        
        if(main_way != 3)//如果主路上没有两个及以上的资源，要考虑支路
        {
            for (p = x + 1; p<obstacle_point; p++)
            {
                left_way=0;
                right_way=0;
                int break_judge = 0;//辅助跳出两层循环
                int y_judge;
                for (y_judge = (y + 1); y_judge < 10; y_judge++)//判断小车左侧支路路况
                {
                    if (map[p][y_judge] == 1)//检测到资源点
                    {
                        if (left_way == 2)//如果该支路上已知有一个资源点
                        {
                            left_way = 3;//该支路被赋予最高的优先级
                            break_judge = 1;//准备跳出所有循环
                            turn_flag = 1;//左转
                            x_turn = p;//最终转弯点确立（不会再改）
                            break;
                        }
                        else
                        {
                            left_way = 2;//该支路已知有一个资源点
                        }
                    }
                    else if (map[p][y_judge] == 2)//发现该支路有障碍物
                    {
                        if (left_way != 2)//有一个资源点的话标为只有一个资源点，没有资源点的话标该路不通
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if(y_judge == 9)//检索到该支路最后一个点
                    {
                        if (left_way != 2) left_way = 1;//如果该支路啥也没有，赋为1
                    }
                }
                for (y_judge = (y - 1); y_judge >= 0; y_judge--)//判断小车右侧路况
                {
                    if (map[p][y_judge] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//右转
                            x_turn = p;
                            break;
                        }
                        else
                        {
                            right_way = 2;
                        }
                    }
                    else if (map[p][y_judge] == 2)
                    {
                        if (right_way != 2)
                        {
                            right_way = 0;
                        }
                        break;
                    }
                    else if (y_judge == 0)
                    {
                        if (right_way != 2) right_way = 1;
                    }
                }
                if (left_way > main_way)//检测到左转有更好的策略
                {
                    x_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//暂时确定转弯的策略，并赋予该策略一定的优先值
                }
                if (right_way > main_way)//检测到右转有更好的策略
                {
                    x_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//跳出循环开关
                    break;
            }
        }
        int sources = 0;//转弯点右侧好东西更多记为正，左侧好东西更多记为负
        for (p = y - 1; p >= 0; p--)//判断右侧
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources++;
            else if (map[x_turn][p] == 1)
                sources = sources + 10;
        }
        for (p = y + 1; p < 10; p++)//判断左侧
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources--;
            else if (map[x_turn][p] == 1)
                sources = sources - 10;
        }
        if (sources > 0) turn_flag = 2;
        else turn_flag = 1;
    }
    else if (direction == 1)
    {
    x_turn = x;
    int obstacle_point = -1;
    //int sourse_point;//main_way为2时表示资源点坐标；
    //int none_flag = 0;
    int y_judge=0;
    for (y_judge = y - 1; y_judge >= 0; y_judge--)//判断主路路况
    {
        if (map[x][y_judge] == 1)//发现资源点
        {
            if (main_way == 2)//如果已经有了一个资源点
            {
                main_way = 3;//赋予主路最高的优先级，并且跳出循环
                break;
            }
            else
            {
                main_way = 2;
            }
        }
        else if (map[x][y_judge] == 2)//检测到障碍物
        {
            if (main_way != 2)
            {
                main_way = 0;//赋予主路最低的优先级，将选择转弯的策略
            }
            obstacle_point = y_judge;//记录主路上第一个障碍物的位置
            break;
        }
        else if (y_judge == 0)//当遍历到主路最后一格时
        {
            if (main_way != 2) main_way = 1;//判断这条路上啥也没有，并将策略优先级赋为1
        }
    }
    y_turn = obstacle_point + 1;//设置保底转弯点，防止小车到出界或触碰障碍物
    if (main_way == 1)//如果主路啥也没有，先让小车尽早转弯，去寻找新的机会，如果检索到其他支路有资源的话后续会更改策略
    {
        main_way = 0;//将主路赋为0，以便让支路啥也没有的话优先级也高于主路，从而实现小车尽快转弯
        y_turn = y - 1;//下一个点就转
    }
    //if (x < 5) turn_flag = 1;//哪边宽阔往哪边转，以便寻找到更多的资源
    //else turn_flag = 2;
    if (main_way != 3)//如果主路上没有两个及以上的资源，要考虑支路
    {
        for (p = y - 1; p > obstacle_point; p--)
        {
            left_way=0;
            right_way=0;
            int break_judge = 0;//辅助跳出两层循环
            int x_judge;
            for (x_judge = (x + 1); x_judge < 10; x_judge++)//判断小车左侧支路路况
            {
                if (map[x_judge][p] == 1)//检测到资源点
                {
                    if (left_way == 2)//如果该支路上已知有一个资源点
                    {
                        left_way = 3;//该支路被赋予最高的优先级
                        break_judge = 1;//准备跳出所有循环
                        turn_flag = 1;//左转
                        y_turn = p;//最终转弯点确立（不会再改）
                        break;
                    }
                    else
                    {
                        left_way = 2;//该支路已知有一个资源点
                    }
                }
                else if (map[x_judge][p] == 2)//发现该支路有障碍物
                {
                    if (left_way != 2)//有一个资源点的话标为只有一个资源点，没有资源点的话标该路不通
                    {
                        left_way = 0;
                    }
                    break;
                }
                else if (x_judge == 9)//检索到该支路最后一个点
                {
                    if (left_way != 2) left_way = 1;//如果该支路啥也没有，赋为1
                }
            }
            for (x_judge = (x - 1); x_judge >= 0; x_judge--)//判断小车右侧路况
            {
                if (map[x_judge][p] == 1)
                {
                    if (right_way == 2)
                    {
                        right_way = 3;
                        //main_way = 3;
                        break_judge = 1;
                        turn_flag = 2;//右转
                        y_turn = p;
                        break;
                    }
                    else
                    {
                        right_way = 2;
                    }
                }
                else if (map[x_judge][p] == 2)
                {
                    if (right_way != 2)
                    {
                        right_way = 0;
                    }
                    break;
                }
                else if (x_judge == 0)
                {
                    if (right_way != 2) right_way = 1;
                }
            }
            if (left_way > main_way)//检测到左转有更好的策略
            {
                y_turn = p;
                turn_flag = 1;
                main_way = left_way;//暂时确定转弯的策略，并赋予该策略一定的优先值
            }
            if (right_way > main_way)//检测到右转有更好的策略
            {
                y_turn = p;
                turn_flag = 2;
                main_way = right_way;
            }
            if (break_judge == 1)//跳出循环开关
                break;
        }
    }
    int sources = 0;//转弯点右侧好东西更多记为正，左侧好东西更多记为负
    for (p = x - 1; p >= 0; p--)//判断右侧
    {
        if (map[p][y_turn] == 2)
            break;
        else if (map[p][y_turn] == 0)
            sources++;
        else if (map[p][y_turn] == 1)
            sources = sources + 10;
    }
    for (p = x + 1; p < 10; p++)//判断左侧
    {
        if (map[p][y_turn] == 2)
            break;
        else if (map[p][y_turn] == 0)
            sources--;
        else if (map[p][y_turn] == 1)
            sources = sources - 10;
    }
    if (sources > 0) turn_flag = 2;
    else turn_flag = 1;
    }
    else if (direction == 2)
    {
        y_turn = y;
        int obstacle_point = -1;//main_way为2时表示障碍物的坐标
        //int sourse_point;//main_way为2时表示资源点坐标；
        //int none_flag = 0;
        int x_judge;
        for (x_judge = x - 1; x_judge >= 0; x_judge--)//判断主路路况
        {
            if (map[x_judge][y] == 1)//发现资源点
            {
                if (main_way == 2)//如果已经有了一个资源点
                {
                    main_way = 3;//赋予主路最高的优先级，并且跳出循环
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x_judge][y] == 2)//检测到障碍物
            {
                if (main_way != 2)
                {
                    main_way = 0;//赋予主路最低的优先级，将选择转弯的策略
                }
                obstacle_point = x_judge;//记录主路上第一个障碍物的位置
                break;
            }
            else if (x_judge == 0)//当遍历到主路最后一格时
            {
                if (main_way != 2) main_way = 1;//判断这条路上啥也没有，并将策略优先级赋为1
            }
        }
        x_turn = obstacle_point + 1;//设置保底转弯点，防止小车到出界或触碰障碍物
        if (main_way == 1)//如果主路啥也没有，先让小车尽早转弯，去寻找新的机会，如果检索到其他支路有资源的话后续会更改策略
        {
            main_way = 0;//将主路赋为0，以便让支路啥也没有的话优先级也高于主路，从而实现小车尽快转弯
            x_turn = x - 1;//下一个点就转
        }
        if (y < 5) turn_flag = 2;//哪边宽阔往哪边转，以便寻找到更多的资源
        else turn_flag = 1;
        if (main_way != 3)//如果主路上没有两个及以上的资源，要考虑支路
        {
            int p;
            for (p = x - 1; p > obstacle_point; p--)
            {
                right_way=0;
                left_way=0;
                int break_judge = 0;//辅助跳出两层循环
                int y_judge;
                for (y_judge = (y - 1); y_judge >= 0; y_judge--)//判断小车左侧支路路况
                {
                    if (map[p][y_judge] == 1)//检测到资源点
                    {
                        if (left_way == 2)//如果该支路上已知有一个资源点
                        {
                            left_way = 3;//该支路被赋予最高的优先级
                            break_judge = 1;//准备跳出所有循环
                            turn_flag = 1;//左转
                            x_turn = p;//最终转弯点确立（不会再改）
                            break;
                        }
                        else
                        {
                            left_way = 2;//该支路已知有一个资源点
                        }
                    }
                    else if (map[p][y_judge] == 2)//发现该支路有障碍物
                    {
                        if (left_way != 2)//有一个资源点的话标为只有一个资源点，没有资源点的话标该路不通
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if (y_judge == 0)//检索到该支路最后一个点
                    {
                        if (left_way != 2) left_way = 1;//如果该支路啥也没有，赋为1
                    }
                }
                for (y_judge = (y + 1); y_judge < 10; y_judge++)//判断小车右侧路况
                {
                    if (map[p][y_judge] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//右转
                            x_turn = p;
                            break;
                        }
                        else
                        {
                            right_way = 2;
                        }
                    }
                    else if (map[p][y_judge] == 2)
                    {
                        if (right_way != 2)
                        {
                            right_way = 0;
                        }
                        break;
                    }
                    else if (y_judge == 9)
                    {
                        if (right_way != 2) right_way = 1;
                    }
                }
                if (left_way > main_way)//检测到左转有更好的策略
                {
                    x_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//暂时确定转弯的策略，并赋予该策略一定的优先值
                }
                if (right_way > main_way)//检测到右转有更好的策略
                {
                    x_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//跳出循环开关
                    break;
            }
        }
        int sources = 0;//转弯点右侧好东西更多记为正，左侧好东西更多记为负
        for (p = y - 1; p >= 0; p--)//判断左侧
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources--;
            else if (map[x_turn][p] == 1)
                sources = sources - 10;
        }
        for (p = y + 1; p < 10; p++)//判断右侧
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources++;
            else if (map[x_turn][p] == 1)
                sources = sources + 10;
        }
        if (sources > 0) turn_flag = 2;
        else turn_flag = 1;
    }
    else if (direction == 3)
    {
        x_turn = x;
        int obstacle_point = 10;
        //int sourse_point;//main_way为2时表示资源点坐标；
        //int none_flag = 0;
        int y_judge;
        for (y_judge = y + 1; y_judge < 10; y_judge++)//判断主路路况
        {
            if (map[x][y_judge] == 1)//发现资源点
            {
                if (main_way == 2)//如果已经有了一个资源点
                {
                    main_way = 3;//赋予主路最高的优先级，并且跳出循环
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x][y_judge] == 2)//检测到障碍物
            {
                if (main_way != 2)
                {
                    main_way = 0;//赋予主路最低的优先级，将选择转弯的策略
                }
                obstacle_point = y_judge;//记录主路上第一个障碍物的位置
                break;
            }
            else if (y_judge == 9)//当遍历到主路最后一格时
            {
                if (main_way != 2) main_way = 1;//判断这条路上啥也没有，并将策略优先级赋为1
            }
        }
        y_turn = obstacle_point - 1;//设置保底转弯点，防止小车到出界或触碰障碍物
        if (main_way == 1)//如果主路啥也没有，先让小车尽早转弯，去寻找新的机会，如果检索到其他支路有资源的话后续会更改策略
        {
            main_way = 0;//将主路赋为0，以便让支路啥也没有的话优先级也高于主路，从而实现小车尽快转弯
            y_turn = y + 1;//下一个点就转
        }
        if (x < 5) turn_flag = 2;//哪边宽阔往哪边转，以便寻找到更多的资源
        else turn_flag = 1;
        if (main_way != 3)//如果主路上没有两个及以上的资源，要考虑支路
        {
            for (p = y + 1; p < obstacle_point; p++)
            {
                right_way=0;
                left_way=0;
                int break_judge = 0;//辅助跳出两层循环
                int x_judge;
                for (x_judge = (x - 1); x_judge >= 0; x_judge--)//判断小车左侧支路路况
                {
                    if (map[x_judge][p] == 1)//检测到资源点
                    {
                        if (left_way == 2)//如果该支路上已知有一个资源点
                        {
                            left_way = 3;//该支路被赋予最高的优先级
                            break_judge = 1;//准备跳出所有循环
                            turn_flag = 1;//左转
                            y_turn = p;//最终转弯点确立（不会再改）
                            break;
                        }
                        else
                        {
                            left_way = 2;//该支路已知有一个资源点
                        }
                    }
                    else if (map[x_judge][p] == 2)//发现该支路有障碍物
                    {
                        if (left_way != 2)//有一个资源点的话标为只有一个资源点，没有资源点的话标该路不通
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if (x_judge == 0)//检索到该支路最后一个点
                    {
                        if (left_way != 2) left_way = 1;//如果该支路啥也没有，赋为1
                    }
                }
                for (x_judge = (x + 1); x_judge < 10; x_judge++)//判断小车右侧路况
                {
                    if (map[x_judge][p] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//右转
                            y_turn = p;
                            break;
                        }
                        else
                        {
                            right_way = 2;
                        }
                    }
                    else if (map[x_judge][p] == 2)
                    {
                        if (right_way != 2)
                        {
                            right_way = 0;
                        }
                        break;
                    }
                    else if (x_judge == 9)
                    {
                        if (right_way != 2) right_way = 1;
                    }
                }
                if (left_way > main_way)//检测到左转有更好的策略
                {
                    y_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//暂时确定转弯的策略，并赋予该策略一定的优先值
                }
                if (right_way > main_way)//检测到右转有更好的策略
                {
                    y_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//跳出循环开关
                    break;
            }
        }
        int sources = 0;//转弯点右侧好东西更多记为正，左侧好东西更多记为负
        for (p = x - 1; p >= 0; p--)//判断左侧
        {
            if (map[p][y_turn] == 2)
                break;
            else if (map[p][y_turn] == 0)
                sources--;
            else if (map[p][y_turn] == 1)
                sources = sources - 10;
        }
        for (p = x + 1; p < 10; p++)//判断右侧
        {
            if (map[p][y_turn] == 2)
                break;
            else if (map[p][y_turn] == 0)
                sources++;
            else if (map[p][y_turn] == 1)
                sources = sources + 10;
        }
        if (sources > 0) turn_flag = 2;
        else turn_flag = 1;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UCA0RX_ISR( ){

	while(1){//未接收到换行符
		while((IFG2&UCA0RXIFG)==0);//检测接收缓冲器是否满
		buffer[i]= UCA0RXBUF;//接收一个数据并保存
		if(buffer[i]==10){//结束

			char result = judgeState();
//			if(result == 'E'/*||result == 'S'*/){
//				speed = 2;
//				command = 7;//停车
//				initIrr();
//			}
			if(result == 'S'){
			    speed = 2;
			    command = 7;//停车
			    direction = old_direction;
			    initIrr();
			}
			else if(result == 'O'){}//成功部署堡垒
			else if(result == 'F'){}//未处理成功
			else if(result == 'G'){
			    times = 0;//开始计时
			    time_flag = 0;
				//speed = 0;
				command = old_command;//全速出发
				old_command = 1;
			}
			//以下四个指令调试使用
//			else if(result == 'L')command = 4;
//			else if(result == 'R')command = 3;
//			else if(result == 'l')command = 9;
//			else if(result == 'r')command = 8;
			//for(i;i>0;i--)buffer[i] = 0;
			i = 0;//归零
			break;
		}
		i++;
	}
	//IE2 &= ~UCA0RXIE; //关闭串口0接收中断允许
}

#pragma vector=PORT1_VECTOR //置P1中断向量
__interrupt void port_int(void)//中断子程
{
	//白色寄存器对应位IN为1，黑色为0
	//过一个黑条
	if((P1IFG&BIT0) != 0 && (P1IN&BIT0) == 0){
		if(direction == 0) x++;
		else if(direction == 1) y--;
		else if(direction == 2) x--;
		else if(direction == 3) y++;
		//send(x,y,0);
        if(time_flag == 1){
            int cnt = 100;
            while(cnt > 0){
                P2OUT |= BIT1;
                P2OUT &= ~BIT2;
                P2OUT |= (BIT3+BIT4);
                delay(30);
                P2OUT &= ~(BIT3+BIT4);
                delay(low);
                cnt--;
            }
            //old_command = command;
            command = 7;//停车
            initIrr();
            turn_flag = 0;
            time_flag == 0;
//            speed = 1;
//            times_sec = 0;
//            time_flag = 2;
        }
		if((x == ene_x || y == ene_y ) && barrier_num < 3){
		    send(x,y,barrier_num + 1);
		    barrier_num ++;
		}
//		if(map[x][y] == 0)
//		    judge_turn_point();
		if(map[x][y] == 1){
		    map[x][y] = 0;//吃掉
		    if(x != x_turn || y != y_turn)judge_turn_point();//判断下一步
		}

		//
		if(x == x_turn && y == y_turn){//到达转弯点
		    if(turn_flag == 1){
		        command = 9;//左转
		        turn_flag = 0;
		        direction = (direction + 3) % 4;
		        judge_turn_point();
		    }
		    else if(turn_flag == 2){
			    command = 8;//右转
			    turn_flag = 0;
			    direction = (direction + 5) % 4;
			    judge_turn_point();
			}
			P1IE &= ~BIT0;//关闭该中断(转弯)
		}
		if((x != x_turn )||(y != y_turn))speed = 0;
		//降速
		if(direction == 0 && x == x_turn - 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 1 && x == x_turn && y == y_turn + 1)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 2 && x == x_turn + 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 3 && x == x_turn && y == y_turn - 1)speed = 1/*,send(x,y,direction+4)*/;
		P1IFG &= ~(BIT0+BIT5);
	}
	//转弯回正判断
	else if((P1IFG & BIT5) != 0 && (command == 5 || command == 6) && (P1IN & BIT5)==0){
		flag_left = 0,flag_right = 0;
		command = 1;//直行
		speed = 3;//怠速
        initIrr();
	}
    else if((P1IFG & BIT5) != 0 && (command == 8 || command == 9) && (P1IN & BIT5)==0){
        flag_left = 0,flag_right = 0;
        command = 1;//直行
        speed = 1;//怠速
        initIrr();
    }

}
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    times ++;
    if(times == 1020){
        time_flag = 1;
        speed = 1;
       // times=0;
    }
//    times_sec ++;
//    if(time_flag == 2 && times_sec == 6){//0.1s
//        times_sec = 0;//清零
//        old_command = command;
//        command = 7;//停车
//        turn_flag = 0;
//    }
}
