#include <msp430.h> 
#define highSpeed 10000
#define lowSpeed 500
#define low 50//这三者决定了占空比和PWM波频率
#define delay_time 40//命令值设为全局变量
int command = 0;//前进后退左转右转
int speed = 0;//控制了速度档位(占空比
const unsigned int high[3]={highSpeed,lowSpeed,0};//0-2分别对应高速低速和停止
unsigned int runSpeed;
unsigned int i=0;
char buffer[30];
char message[8];
int map[11][11];
//对手位置
int ene_x = 0;
int ene_y = 0;
//小车所处的位置
int x = 0;
int y = 9;
int direction = 0;//direction 4 进制，0为x正半轴，3为y正半轴，左转+3%4,右转+5%4
//堡垒和驱逐舰
const int bar[5][5][2] = {{{0,0},{0,1},{1,0},{0,-1},{-1,0}},{{0,1},{1,1},{-1,1},{-1,0},{-1,-1}},{{-1,1},{0,1},{1,1},{1,0},{1,-1}},
                          {{-1,-1},{0,-1},{1,-1},{1,0},{1,1}},{{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
//训练赛使用(正赛不用这个
int x_turn = 1,y_turn = 9;
void delay();
void run();
void initPWM();
void initUARTA0();
void initIrr();
char judgeState();
void send(int x, int y, int type);
int main(void){
	WDTCTL = WDTPW | WDTHOLD;// stop watchdog timer
	initPWM();//小车PWM的引脚设置
	initUARTA0();//通讯设置
	initIrr();//通讯中断设置
    //调试
    command = 1;
    speed = 0;

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
    //传感器中断P1.3-P1.7
    //中断引脚的相关设置
    P1SEL &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1SEL2 &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1OUT |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1REN |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1DIR &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IES |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IE |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    //通讯中断
    IE2 |= UCA0RXIE;//允许USCI_A0 的接收中断
    _EINT();//总中断允许
}
void delay(unsigned int i){

    for(;i>0;i--);
}
void run(){
    runSpeed = high[speed];//占空比(延迟函数传参)
    if(command == 1){//前进
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
        delay(runSpeed);//高电平持续时间
        P2OUT &= ~(BIT3+BIT4);
        delay(low);//低电平持续时间
    }
    if(command == 2){//后退
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
        delay(runSpeed);//高电平持续时间
        P2OUT &= ~(BIT3+BIT4);
        delay(low);//低电平持续时间
    }
    if(command == 3){//前进右转(微调用
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
        delay(runSpeed/20);//高电平持续时间
        P2OUT &= ~BIT3;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
        delay(runSpeed/20);//低电平持续时间
        P2OUT &= ~BIT4;
        delay(low);//低电平持续时间
//        P2OUT |= (BIT4);//调整四个输出引脚状态
//        delay(runSpeed/20);//高电平持续时间
//        P2OUT &= ~BIT4;//调整PWMB为低电平，做到PWMA的占空比大于PWMB
//        delay(low);//低电平持续时间

    }
    if(command == 4){//前进左转(微调用
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
        delay(runSpeed/20);//高电平持续时间
        P2OUT &= ~BIT4;//调整PWMA为低电平，做到PWMB的占空比大于PWMA
        delay(runSpeed/20);//低电平持续时间
        P2OUT &= ~BIT3;
        delay(low);//低电平持续时间
//        P2OUT |= (BIT3);//调整四个输出引脚状态
//        delay(runSpeed/20);//高电平持续时间
//        P2OUT &= ~BIT3;//调整PWMA为低电平，做到PWMB的占空比大于PWMA
//        delay(low);//低电平持续时间
    }
    if(command == 5){//原地左转
        P2OUT |= BIT1;//右轮前转
        P2OUT &= ~BIT2;
        P2OUT |= BIT4;
        P2OUT &= ~BIT3;
        delay(lowSpeed);
        P2OUT &= ~BIT4;
        delay(low);
        //左轮后转
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= BIT3;
        P2OUT &= ~BIT4;
        delay(lowSpeed);
        P2OUT &= ~BIT3;
        delay(low);
    }
    if(command == 6){//原地右转
        P2OUT |= BIT1;//左轮前传
        P2OUT &= ~BIT2;
        P2OUT |= BIT3;
        P2OUT &= ~BIT4;
        delay(lowSpeed);
        P2OUT &= ~BIT3;
        delay(low);
        //右轮后转
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= BIT4;
        P2OUT &= ~BIT3;
        delay(lowSpeed);
        P2OUT &= ~BIT4;
        delay(low);
    }
    if(command == 7){
        P2OUT &= ~(BIT3+BIT4);//制动
        delay(low);
    }
    if(command == 8){//前进右转
        int cnt = delay_time;//延迟时间
        while(cnt > 0){
            P2OUT |= BIT1;
            P2OUT &= ~BIT2;
            P2OUT |= (BIT3+BIT4);//调整四个输出引脚状态
            P2OUT &= ~BIT4;
            delay(runSpeed/10);
//            P2OUT &= ~BIT4;
//            delay(runSpeed);
            P2OUT &= ~BIT3;
            delay(low);//低电平持续时间
            cnt--;
        }
        direction = (direction + 5) % 4;
       // command = 1;
        initIrr();
    }
    if(command == 9){//前进左转
        int cnt = delay_time;//控制转弯时长
        while(cnt > 0){
            P2OUT |= BIT1;
            P2OUT &= ~BIT2;
            P2OUT |= (BIT3+BIT4);
            P2OUT &= BIT3;
            delay(runSpeed/10);
//            P2OUT &= ~BIT3;
//            delay(runSpeed);//低电平持续时间
            P2OUT &= ~BIT4;
            delay(low);//低电平持续时间
            cnt--;
        }
        direction = (direction + 3) % 4;
        //command = 1;
        initIrr();
    }
}
char judgeState(){//用于解析buffer数组，判断上机位的指令
    if((buffer[0] >= 'A' && buffer[0] <= 'Z')||(buffer[0] >= 'a' && buffer[0] <= 'z'))return buffer[0];//字母直接返回
    //训练赛解析:
    else if(buffer[0] >= '1' && buffer[0] <= '9' && buffer[3] != '&'){
        x_turn = buffer[0] - '0';
        y_turn = 10;
    }
    else if(buffer[0] >= '1' && buffer[0] <= '9' && buffer[3] == '&')
    {

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
        while(message[k]!='\n')      //输出提示信息
        {
            while((IFG2&UCA0TXIFG)==0);     //检测发送缓冲是否空
            UCA0TXBUF=message[k];       //取一个数据发送
        }
        k++;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UCA0RX_ISR( ){

    while(1){//未接收到换行符
        while((IFG2&UCA0RXIFG)==0);//检测接收缓冲器是否满
        buffer[i]= UCA0RXBUF;//接收一个数据并保存
        if(buffer[i]==10){//结束
            i = 0;//归零
            char result = judgeState();
            if(result == 'E'||result == 'S'){
                speed = 0;
                command = 7;//停车
            }
            else if(result == 'O'){}//成功部署堡垒
            else if(result == 'F'){}//未处理成功
            else if(result == 'G'){
                speed = 1;
                command = 1;//全速出发
            }
            //以下四个指令调试使用
            else if(result == 'L')command = 4;
            else if(result == 'R')command = 3;
            else if(result == 'l')command = 5;
            else if(result == 'r')command = 6;
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
    if((P1IFG&BIT3)!=0 && (P1IFG&BIT7)!=0 && (P1IN&BIT3)==0 && (P1IN&BIT4)==0 && (P1IN&BIT5)==0 && (P1IN&BIT6)==0 && (P1IN&BIT7)==0 && (command == 1 || command == 3 || command == 4)){
        if(direction == 0) x++;
        else if(direction == 1) y--;
        else if(direction == 2) x--;
        else if(direction == 3) y++;

        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
        //调试
        if(x == x_turn && y == y_turn){//到达转弯点
            command = 8;//右转
            send(x,y,1);//type
            //command = 7;
            P1IE &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);//关闭该中断(转弯)
        }
    }
    //如果走偏
    if((P1IN&BIT4) == 0 && command == 1 && (P1IN&BIT3)!=0 && (P1IN&BIT6)!=0 && (P1IN&BIT7)!=0){
        command = 3;
        P1IES ^= BIT4;//BIT4触发沿取反
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
        //2号处于黑色边条上方，1.4.5号处于白色区域上方说明右偏；command == 1说明是直行状态
    }
    if((P1IN&BIT6)==0 && command == 1 && (P1IN&BIT7)!=0 && (P1IN&BIT3)!=0 && (P1IN&BIT4)!=0){
        //4号处于黑色边条上方，1.2.5号处于白色区域上方说明左偏；command == 1说明是直行状态
        P1IES ^= BIT6;//BIT6触发沿取反
        command = 4;
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    }
    if((P1IN&BIT4)!=0 && (command == 4 || command == 3) &&(P1IN&BIT6)!=0){
        //command == 3 or 4说明是微调，2号和4号已经脱离黑色边条上方，改为直行
        if(command == 4)P1IES ^= BIT6;//BIT6触发沿取反
        if(command == 3)P1IES ^= BIT4;//BIT4触发沿取反
        command = 1;//直行
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    }
    //转弯回正判断
    if((command == 8 || command == 9) && (P1IFG & BIT5) != 0){
        command = 1;//直行
        P1IFG &= ~(BIT5);
        //1.5号处于白色区域上方,3处于黑色上方说明已经转弯完成
    }
}
