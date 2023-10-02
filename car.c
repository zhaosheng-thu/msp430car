#include <msp430.h> 
#define highSpeed 10000
#define lowSpeed 500
#define low 50//�����߾�����ռ�ձȺ�PWM��Ƶ��
#define delay_time 40//����ֵ��Ϊȫ�ֱ���
int command = 0;//ǰ��������ת��ת
int speed = 0;//�������ٶȵ�λ(ռ�ձ�
const unsigned int high[3]={highSpeed,lowSpeed,0};//0-2�ֱ��Ӧ���ٵ��ٺ�ֹͣ
unsigned int runSpeed;
unsigned int i=0;
char buffer[30];
char message[8];
int map[11][11];
//����λ��
int ene_x = 0;
int ene_y = 0;
//С��������λ��
int x = 0;
int y = 9;
int direction = 0;//direction 4 ���ƣ�0Ϊx�����ᣬ3Ϊy�����ᣬ��ת+3%4,��ת+5%4
//���ݺ�����
const int bar[5][5][2] = {{{0,0},{0,1},{1,0},{0,-1},{-1,0}},{{0,1},{1,1},{-1,1},{-1,0},{-1,-1}},{{-1,1},{0,1},{1,1},{1,0},{1,-1}},
                          {{-1,-1},{0,-1},{1,-1},{1,0},{1,1}},{{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
//ѵ����ʹ��(�����������
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
	initPWM();//С��PWM����������
	initUARTA0();//ͨѶ����
	initIrr();//ͨѶ�ж�����
    //����
    command = 1;
    speed = 0;

    while(1){
        run();//С��������

    }
}
void initPWM(){

    P2SEL &= ~(BIT1+BIT2+BIT3+BIT4);//P2.1-2.4ݔ������˳���ӦAIN1,AIN2,PWMA,PWMB
    P2SEL2 &= ~(BIT1+BIT2+BIT3+BIT4);
    P2OUT |= (BIT1+BIT2+BIT3+BIT4);
    P2DIR |= (BIT1+BIT2+BIT3+BIT4);//�����������
}
void initUARTA0(){
    UCA0CTL1 |= UCSWRST;//�������λλswrstΪ1
    P1SEL |= BIT1+BIT2;//��P1.1��P1.2Ϊ���нӿ��ա������Ź���
    P1SEL2 |= BIT1+BIT2;
    //���ݸ�ʽѡ���ϵ縴λ���ã���У�飬8λ���ݣ�1��ֹͣλ���첽����ͨ��
    UCA0CTL1|=UCSSEL0+UCRXEIE; //������ʱ��ѡ���ϵ縴λʱ��ACLK��32.768KHz���Դ����
    UCA0BR0 =3;//������9600
    UCA0BR1 = 0;
    UCA0MCTL = UCBRF_0+UCBRS_3;
    UCA0CTL1 &= ~UCSWRST;        //�������λλswrstΪ0�������������
}
void initIrr(){
    //�������ж�P1.3-P1.7
    //�ж����ŵ��������
    P1SEL &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1SEL2 &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1OUT |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1REN |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1DIR &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IES |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    P1IE |= (BIT3+BIT4+BIT5+BIT6+BIT7);
    //ͨѶ�ж�
    IE2 |= UCA0RXIE;//����USCI_A0 �Ľ����ж�
    _EINT();//���ж�����
}
void delay(unsigned int i){

    for(;i>0;i--);
}
void run(){
    runSpeed = high[speed];//ռ�ձ�(�ӳٺ�������)
    if(command == 1){//ǰ��
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
        delay(runSpeed);//�ߵ�ƽ����ʱ��
        P2OUT &= ~(BIT3+BIT4);
        delay(low);//�͵�ƽ����ʱ��
    }
    if(command == 2){//����
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
        delay(runSpeed);//�ߵ�ƽ����ʱ��
        P2OUT &= ~(BIT3+BIT4);
        delay(low);//�͵�ƽ����ʱ��
    }
    if(command == 3){//ǰ����ת(΢����
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
        delay(runSpeed/20);//�ߵ�ƽ����ʱ��
        P2OUT &= ~BIT3;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
        delay(runSpeed/20);//�͵�ƽ����ʱ��
        P2OUT &= ~BIT4;
        delay(low);//�͵�ƽ����ʱ��
//        P2OUT |= (BIT4);//�����ĸ��������״̬
//        delay(runSpeed/20);//�ߵ�ƽ����ʱ��
//        P2OUT &= ~BIT4;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
//        delay(low);//�͵�ƽ����ʱ��

    }
    if(command == 4){//ǰ����ת(΢����
        P2OUT |= BIT1;
        P2OUT &= ~BIT2;
        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
        delay(runSpeed/20);//�ߵ�ƽ����ʱ��
        P2OUT &= ~BIT4;//����PWMAΪ�͵�ƽ������PWMB��ռ�ձȴ���PWMA
        delay(runSpeed/20);//�͵�ƽ����ʱ��
        P2OUT &= ~BIT3;
        delay(low);//�͵�ƽ����ʱ��
//        P2OUT |= (BIT3);//�����ĸ��������״̬
//        delay(runSpeed/20);//�ߵ�ƽ����ʱ��
//        P2OUT &= ~BIT3;//����PWMAΪ�͵�ƽ������PWMB��ռ�ձȴ���PWMA
//        delay(low);//�͵�ƽ����ʱ��
    }
    if(command == 5){//ԭ����ת
        P2OUT |= BIT1;//����ǰת
        P2OUT &= ~BIT2;
        P2OUT |= BIT4;
        P2OUT &= ~BIT3;
        delay(lowSpeed);
        P2OUT &= ~BIT4;
        delay(low);
        //���ֺ�ת
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= BIT3;
        P2OUT &= ~BIT4;
        delay(lowSpeed);
        P2OUT &= ~BIT3;
        delay(low);
    }
    if(command == 6){//ԭ����ת
        P2OUT |= BIT1;//����ǰ��
        P2OUT &= ~BIT2;
        P2OUT |= BIT3;
        P2OUT &= ~BIT4;
        delay(lowSpeed);
        P2OUT &= ~BIT3;
        delay(low);
        //���ֺ�ת
        P2OUT |= BIT2;
        P2OUT &= ~BIT1;
        P2OUT |= BIT4;
        P2OUT &= ~BIT3;
        delay(lowSpeed);
        P2OUT &= ~BIT4;
        delay(low);
    }
    if(command == 7){
        P2OUT &= ~(BIT3+BIT4);//�ƶ�
        delay(low);
    }
    if(command == 8){//ǰ����ת
        int cnt = delay_time;//�ӳ�ʱ��
        while(cnt > 0){
            P2OUT |= BIT1;
            P2OUT &= ~BIT2;
            P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
            P2OUT &= ~BIT4;
            delay(runSpeed/10);
//            P2OUT &= ~BIT4;
//            delay(runSpeed);
            P2OUT &= ~BIT3;
            delay(low);//�͵�ƽ����ʱ��
            cnt--;
        }
        direction = (direction + 5) % 4;
       // command = 1;
        initIrr();
    }
    if(command == 9){//ǰ����ת
        int cnt = delay_time;//����ת��ʱ��
        while(cnt > 0){
            P2OUT |= BIT1;
            P2OUT &= ~BIT2;
            P2OUT |= (BIT3+BIT4);
            P2OUT &= BIT3;
            delay(runSpeed/10);
//            P2OUT &= ~BIT3;
//            delay(runSpeed);//�͵�ƽ����ʱ��
            P2OUT &= ~BIT4;
            delay(low);//�͵�ƽ����ʱ��
            cnt--;
        }
        direction = (direction + 3) % 4;
        //command = 1;
        initIrr();
    }
}
char judgeState(){//���ڽ���buffer���飬�ж��ϻ�λ��ָ��
    if((buffer[0] >= 'A' && buffer[0] <= 'Z')||(buffer[0] >= 'a' && buffer[0] <= 'z'))return buffer[0];//��ĸֱ�ӷ���
    //ѵ��������:
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
        while(message[k]!='\n')      //�����ʾ��Ϣ
        {
            while((IFG2&UCA0TXIFG)==0);     //��ⷢ�ͻ����Ƿ��
            UCA0TXBUF=message[k];       //ȡһ�����ݷ���
        }
        k++;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UCA0RX_ISR( ){

    while(1){//δ���յ����з�
        while((IFG2&UCA0RXIFG)==0);//�����ջ������Ƿ���
        buffer[i]= UCA0RXBUF;//����һ�����ݲ�����
        if(buffer[i]==10){//����
            i = 0;//����
            char result = judgeState();
            if(result == 'E'||result == 'S'){
                speed = 0;
                command = 7;//ͣ��
            }
            else if(result == 'O'){}//�ɹ�������
            else if(result == 'F'){}//δ����ɹ�
            else if(result == 'G'){
                speed = 1;
                command = 1;//ȫ�ٳ���
            }
            //�����ĸ�ָ�����ʹ��
            else if(result == 'L')command = 4;
            else if(result == 'R')command = 3;
            else if(result == 'l')command = 5;
            else if(result == 'r')command = 6;
            break;
        }
        i++;
    }
    //IE2 &= ~UCA0RXIE; //�رմ���0�����ж�����
}

#pragma vector=PORT1_VECTOR //��P1�ж�����
__interrupt void port_int(void)//�ж��ӳ�
{
    //��ɫ�Ĵ�����ӦλINΪ1����ɫΪ0
    //��һ������
    if((P1IFG&BIT3)!=0 && (P1IFG&BIT7)!=0 && (P1IN&BIT3)==0 && (P1IN&BIT4)==0 && (P1IN&BIT5)==0 && (P1IN&BIT6)==0 && (P1IN&BIT7)==0 && (command == 1 || command == 3 || command == 4)){
        if(direction == 0) x++;
        else if(direction == 1) y--;
        else if(direction == 2) x--;
        else if(direction == 3) y++;

        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
        //����
        if(x == x_turn && y == y_turn){//����ת���
            command = 8;//��ת
            send(x,y,1);//type
            //command = 7;
            P1IE &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);//�رո��ж�(ת��)
        }
    }
    //�����ƫ
    if((P1IN&BIT4) == 0 && command == 1 && (P1IN&BIT3)!=0 && (P1IN&BIT6)!=0 && (P1IN&BIT7)!=0){
        command = 3;
        P1IES ^= BIT4;//BIT4������ȡ��
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
        //2�Ŵ��ں�ɫ�����Ϸ���1.4.5�Ŵ��ڰ�ɫ�����Ϸ�˵����ƫ��command == 1˵����ֱ��״̬
    }
    if((P1IN&BIT6)==0 && command == 1 && (P1IN&BIT7)!=0 && (P1IN&BIT3)!=0 && (P1IN&BIT4)!=0){
        //4�Ŵ��ں�ɫ�����Ϸ���1.2.5�Ŵ��ڰ�ɫ�����Ϸ�˵����ƫ��command == 1˵����ֱ��״̬
        P1IES ^= BIT6;//BIT6������ȡ��
        command = 4;
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    }
    if((P1IN&BIT4)!=0 && (command == 4 || command == 3) &&(P1IN&BIT6)!=0){
        //command == 3 or 4˵����΢����2�ź�4���Ѿ������ɫ�����Ϸ�����Ϊֱ��
        if(command == 4)P1IES ^= BIT6;//BIT6������ȡ��
        if(command == 3)P1IES ^= BIT4;//BIT4������ȡ��
        command = 1;//ֱ��
        P1IFG &= ~(BIT3+BIT4+BIT5+BIT6+BIT7);
    }
    //ת������ж�
    if((command == 8 || command == 9) && (P1IFG & BIT5) != 0){
        command = 1;//ֱ��
        P1IFG &= ~(BIT5);
        //1.5�Ŵ��ڰ�ɫ�����Ϸ�,3���ں�ɫ�Ϸ�˵���Ѿ�ת�����
    }
}
