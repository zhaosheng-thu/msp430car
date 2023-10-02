#include <msp430.h>
#define highSpeed 115//80
#define lowSpeed 27//20
#define low 50//�����߾�����ռ�ձȺ�PWM��Ƶ��
int delay_time = 500;//����ֵ��Ϊȫ�ֱ���
int command = 7;//ǰ��������ת��ת
int old_command = 1;
int old_direction = 1;
int speed = 0;//�������ٶȵ�λ(ռ�ձ�
const unsigned int high[4]={highSpeed,lowSpeed,0,16};//0-2�ֱ��Ӧ���ٵ��ٺ�ֹͣ
unsigned int runSpeed;
unsigned int i=0;
char buffer[125];
char message[8];
int map[10][10];
//����λ��
int ene_x = 0;
int ene_y = 0;
//С��������λ��
int barrier_num = 0;
int times = 0;
int times_sec = 0;
int time_flag = 0;
int main_way = 0;
int x = 0;
int y = 9;
int direction = 0;//direction 4 ���ƣ�0Ϊx�����ᣬ3Ϊy�����ᣬ��ת+3%4,��ת+5%4
//���ݺ�����
const int bar[5][5][2] = {{{0,0},{0,1},{1,0},{0,-1},{-1,0}},{{0,1},{1,1},{-1,1},{-1,0},{-1,-1}},{{-1,1},{0,1},{1,1},{1,0},{1,-1}},
{{-1,-1},{0,-1},{1,-1},{1,0},{1,1}},{{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
int right_way;//ǰ����ת����·��
int left_way;//ǰ����ת����·��
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
	initPWM();//С��PWM����������
	initUARTA0();//ͨѶ����
	initIrr();//ͨѶ�ж�����
	initClock();
	//����
	old_command = 1;
	command = 7;
	speed = 0;
	direction = 1;
	old_direction = 1;
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
	//�������ж�P1.0
	//�ж����ŵ��������
	P1SEL &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1SEL2 &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1OUT |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1REN |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1DIR &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IES |= (BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IFG &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IE &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);
	P1IE |= BIT0;
	//ͨѶ�ж�
	IE2 |= UCA0RXIE;//����USCI_A0 �Ľ����ж�
	_EINT();//���ж�����
}

void initClock(){
    BCSCTL1 |= DIVA_0; //ACLK�����ⲿ����1��Ƶ
    TA0CCTL0=CCIE; //�趨��ʱ��CCR0�ж�ʹ��
    TA0CTL=TASSEL_1+MC_1; //ѡ��ACLKʱ��Դ, UPģʽ
    TA0CCR0= 328; //�趨����ֵ 32768*1/32768=1s��������
    _EINT(); //���ж�
    times = 1300;//��ֹ��ʼ�����ж�
    time_flag = 0;
 //   LPM0;
}
void delay(unsigned int i_temp){

	for(;i_temp>0;i_temp--);
}
void run(){
	//command=9;
	runSpeed = high[speed];//ռ�ձ�(�ӳٺ�������)
	if(command == 1){//ǰ��
	    if(((P1IN&BIT4) == 0) && ((P1IN&BIT6) != 0)){
	        P2OUT |= BIT1;
	        P2OUT &= ~BIT2;
	        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
	        if(speed == 0){
	            delay(50);//�ߵ�ƽ����ʱ��
	            P2OUT &= ~BIT3;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	            delay(30);//�͵�ƽ����ʱ��
	            P2OUT &= ~BIT4;
	            delay(65);//�͵�ƽ����ʱ��
	        }
	        else if(speed == 1 || speed == 3){
	            delay(50);//�ߵ�ƽ����ʱ��
	            P2OUT &= ~BIT3;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	            delay(50);//�͵�ƽ����ʱ��
	            P2OUT &= ~BIT4;
	            delay(100);//�͵�ƽ����ʱ��
	        }
	    }
	    else if(((P1IN&BIT4) != 0) && ((P1IN&BIT6) == 0)){
	        P2OUT |= BIT1;
	        P2OUT &= ~BIT2;
	        P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
	        if(speed == 0){
	            delay(50);//�ߵ�ƽ����ʱ��
	            P2OUT &= ~BIT4;//����PWMAΪ�͵�ƽ������PWMB��ռ�ձȴ���PWMA
	            delay(30);//�͵�ƽ����ʱ��
	            P2OUT &= ~BIT3;
	            delay(65);//�͵�ƽ����ʱ��
	        }
	        else if(speed == 1 || speed == 3){
	            delay(50);//�ߵ�ƽ����ʱ��
	            P2OUT &= ~BIT4;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	            delay(50);//�͵�ƽ����ʱ��
	            P2OUT &= ~BIT3;
	            delay(150);//�͵�ƽ����ʱ��
	        }
	    }
	    else{//���ڻ����ף�������ϵ�
	        if(((P1IN&BIT3) != 0) && ((P1IN&BIT7) == 0)){
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
	            if(speed == 0){
	                delay(50);//�ߵ�ƽ����ʱ��
	                P2OUT &= ~BIT4;//����PWMAΪ�͵�ƽ������PWMB��ռ�ձȴ���PWMA
	                delay(30);//�͵�ƽ����ʱ��
	                P2OUT &= ~BIT3;
	                delay(65);//�͵�ƽ����ʱ��
	            }
	            else if(speed == 1 || speed == 3){
	                delay(50);//�ߵ�ƽ����ʱ��
	                P2OUT &= ~BIT4;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	                delay(50);//�͵�ƽ����ʱ��
	                P2OUT &= ~BIT3;
	                delay(150);//�͵�ƽ����ʱ��
	            }
	        }
	        else if(((P1IN&BIT3) == 0) && ((P1IN&BIT7) != 0)){
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
	            if(speed == 0){
	                delay(50);//�ߵ�ƽ����ʱ��
	                P2OUT &= ~BIT3;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	                delay(30);//�͵�ƽ����ʱ��
	                P2OUT &= ~BIT4;
	                delay(65);//�͵�ƽ����ʱ��
	            }
	            else if(speed == 1 || speed == 3){
	                delay(50);//�ߵ�ƽ����ʱ��
	                P2OUT &= ~BIT3;//����PWMBΪ�͵�ƽ������PWMA��ռ�ձȴ���PWMB
	                delay(50);//�͵�ƽ����ʱ��
	                P2OUT &= ~BIT4;
	                delay(100);//�͵�ƽ����ʱ��
	            }
	        }
	        else{
	            P1IFG &= ~BIT5;
	            P1IE &= ~BIT5;//���м��ж�
	            P2OUT |= BIT1;
	            P2OUT &= ~BIT2;
	            P2OUT |= (BIT3+BIT4);
	            delay(runSpeed);
	            P2OUT &= ~(BIT3+BIT4);
	            delay(low);
	        }
	    }
	}
	else if(command == 2){//����
		P2OUT |= BIT2;
		P2OUT &= ~BIT1;
		P2OUT |= (BIT3+BIT4);
		delay(runSpeed);
		P2OUT &= ~(BIT3+BIT4);
		delay(low);
	}
	if(command == 5){//ԭ����ת
		P2OUT |= BIT1;//����ǰת
		P2OUT &= ~BIT2;
		P2OUT |= BIT4;
		P2OUT &= ~BIT3;
		delay(lowSpeed*2);
		P2OUT &= ~BIT4;
		delay(low);
		//���ֺ�ת
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
//		            delay(low);//�͵�ƽ����ʱ��
	}
	if(command == 6){//ԭ����ת
		P2OUT |= BIT1;//����ǰ��
		P2OUT &= ~BIT2;
		P2OUT |= BIT3;
		P2OUT &= ~BIT4;
		delay(lowSpeed*2);
		P2OUT &= ~BIT3;
		delay(low);
		//���ֺ�ת
		P2OUT |= BIT2;
		P2OUT &= ~BIT1;
		P2OUT |= BIT4;
		P2OUT &= ~BIT3;
		delay(lowSpeed*2);
		P2OUT &= ~BIT4;
		delay(low);
		P2OUT |= BIT1;
//		            P2OUT &= ~BIT2;
//		            P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
//		            P2OUT &= ~BIT4;
//		            delay(20);//ռ�ձ�95%
//		            P2OUT &= ~BIT3;
//		            delay(low);//�͵�ƽ����ʱ��
	}
	if(command == 7){
		P2OUT &= ~(BIT3+BIT4);//�ƶ�
		delay(low);
	}
	if(command == 8){//ǰ����ת
		int cnt = 0;
		if(flag_right == 0)cnt = delay_time;//�ӳ�ʱ��
		else cnt = 1;
		while(cnt > 0){
			P2OUT |= BIT1;
			P2OUT &= ~BIT2;
			P2OUT |= (BIT3+BIT4);//�����ĸ��������״̬
			P2OUT &= ~BIT4;
			delay(18);//ռ�ձ�95%//20
			P2OUT &= ~BIT3;
			delay(low);//�͵�ƽ����ʱ��
			flag_right = 1;
			P1IFG &= ~BIT5;
			if(cnt == 1)P1IE |= BIT5;
			cnt--;
		}
	}
	if(command == 9){//ǰ����ת
        int cnt = 0;
        if(flag_left == 0)cnt = delay_time;//�ӳ�ʱ��
        else cnt = 1;
		while(cnt > 0){
			P2OUT |= BIT1;
			P2OUT &= ~BIT2;
			P2OUT |= (BIT3+BIT4);
			P2OUT &= ~BIT3;
			delay(18);//20
			P2OUT &= ~BIT4;
			delay(low);//�͵�ƽ����ʱ��
			flag_left = 1;
            P1IFG &= ~BIT5;
			if(cnt == 1)P1IE |= BIT5;
			cnt--;
		}
	}
}
char judgeState(){//���ڽ���buffer���飬�ж��ϻ�λ��ָ��
	if((buffer[0] >= 'A' && buffer[0] <= 'Z')||(buffer[0] >= 'a' && buffer[0] <= 'z'))return buffer[0];//��ĸֱ�ӷ���
	//ѵ��������:
	if(buffer[0] >= '0' && buffer[0] <= '9' && buffer[3] != '&'){
		x_turn = buffer[0] - '0';
		y_turn = 9;
	}
	if(buffer[0] >= '0' && buffer[0] <= '9' && buffer[3] == '&'){
	    unsigned int g,h;
	    for(g=0;g<10;g++)
	        for(h=0;h<10;h++)map[g][h] = 0;//��ʼ��
	    x = buffer[0] - '0';
	    y = buffer[2] - '0';
	    unsigned int m = 4;
	    while(buffer[4] != '&'){//��Դ�㲻Ϊ��
	        map[buffer[m] - '0'][buffer[m+2] - '0'] = 1;//��Դ
	        m += 4;
	        if(buffer[m-1] == '&')break;
	    }
	    if(buffer[4] == '&'){//��Դ��Ϊ��
	        ene_x = buffer[5];
	        ene_y = buffer[7];
	        unsigned int j = 9;
	        while(buffer[9] != '\n'){//�ϰ��ﲻΪ��
	            int type_bar = buffer[j+4] - '0' - 1;//��������
	            int n = 0;
	            for(n = 0; n<5; n++){
                    int x_tmp = buffer[j] - '0' + bar[type_bar][n][0];
                    int y_tmp = buffer[j+2] - '0' + bar[type_bar][n][1];
                    if(x_tmp >=0 && x_tmp <= 9 && y_tmp >=0 && y_tmp <= 9) map[x_tmp][y_tmp] = 2 ;//2Ϊ�ϰ���
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
                    if(u>=0 && v>=0 && u<=9 && v<=9) map[u][v] = 2;//����3*3�ڲ���ͨ��
            unsigned int j = m+4;
            while(buffer[m+4] != '\n'){//�ϰ��ﲻΪ��
                int type_bar = buffer[j+4] - '0' - 1;//��������
                int n = 0;
                for(n = 0; n<5; n++){
                    int x_tmp = buffer[j] - '0' + bar[type_bar][n][0];
                    int y_tmp = buffer[j+2] - '0' + bar[type_bar][n][1];
                    if(x_tmp >=0 && x_tmp <= 9 && y_tmp >=0 && y_tmp <= 9) map[x_tmp][y_tmp] = 2 ;//2Ϊ�ϰ���
                }
                j += 6;
                if(buffer[j-1] == '\n' || buffer[j-1] == '\r')
                    break;
            }
	    }
//        if(x == 0 && y == 9)direction = 1;
//        if(x == 9 && y == 0)direction = 2;
	    //����
	    if((x == 8 && y == 3)||(x == 7 && y == 3))map[1][3] = 2;
	    judge_turn_point();
	    speed = 0;//�޸�speed
	    if(direction == 0 && x == x_turn - 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 1 && x == x_turn && y == y_turn + 1)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 2 && x == x_turn + 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
	    else if(direction == 3 && x == x_turn && y == y_turn - 1)speed = 1/*,send(x,y,direction+4)*/;
	    else speed = 0;
	    if(main_way == 0){//����
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
	        P1IE &= ~BIT0;//�رո��ж�(ת��)
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
		if(message[k]!='\n') {     //�����ʾ��Ϣ
			while((IFG2&UCA0TXIFG)==0);     //��ⷢ�ͻ����Ƿ��
			    UCA0TXBUF=message[k];       //ȡһ�����ݷ���
		}
		else {
		    while((IFG2&UCA0TXIFG)==0);     //��ⷢ�ͻ����Ƿ��
		        UCA0TXBUF=message[5];
		    break;
		}
		k++;
	}
}

void judge_turn_point()//����ת���
{
    turn_flag = 0;
    main_way = 0;//ǰ����··��,��ֵ��ʾ���Ե�����
    int p;
    //0��ʾ���ϰ����赲 1��ʾʲô��û�� 2��ʾ����һ����Դ��Ȼ����һ���ϰ����赲����ֻ��һ����Դ�� 3��ʾ��������������Դ��
    if (direction == 0)
    {
        y_turn = y;
        int obstacle_point = 10;
        //int sourse_point;//main_wayΪ2ʱ��ʾ��Դ�����ꣻ
        //int none_flag = 0;
        int x_judge;
        for (x_judge = x + 1; x_judge < 10; x_judge++)//�ж���··��
        {
            if (map[x_judge][y] == 1)//������Դ��
            {
                if (main_way == 2)//����Ѿ�����һ����Դ��
                {
                    main_way = 3;//������·��ߵ����ȼ�����������ѭ��
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x_judge][y] == 2)//��⵽�ϰ���
            {
                if (main_way != 2)
                {
                    main_way = 0;//������·��͵����ȼ�����ѡ��ת��Ĳ���
                }
                obstacle_point = x_judge;//��¼��·�ϵ�һ���ϰ����λ��
                break;
            }
            else if (x_judge == 9)//����������·���һ��ʱ
            {
                if (main_way != 2) main_way = 1;//�ж�����·��ɶҲû�У������������ȼ���Ϊ1
            }
        }
        x_turn = obstacle_point - 1;//���ñ���ת��㣬��ֹС������������ϰ���
        if (main_way == 1)//�����·ɶҲû�У�����С������ת�䣬ȥѰ���µĻ��ᣬ�������������֧·����Դ�Ļ���������Ĳ���
        {
            main_way = 0;//����·��Ϊ0���Ա���֧·ɶҲû�еĻ����ȼ�Ҳ������·���Ӷ�ʵ��С������ת��
            x_turn = x + 1;//��һ�����ת
        }
        //if (y < 5) turn_flag = 1;//�ı߿������ı�ת���Ա�Ѱ�ҵ��������Դ
        //else turn_flag = 2;
        
        if(main_way != 3)//�����·��û�����������ϵ���Դ��Ҫ����֧·
        {
            for (p = x + 1; p<obstacle_point; p++)
            {
                left_way=0;
                right_way=0;
                int break_judge = 0;//������������ѭ��
                int y_judge;
                for (y_judge = (y + 1); y_judge < 10; y_judge++)//�ж�С�����֧··��
                {
                    if (map[p][y_judge] == 1)//��⵽��Դ��
                    {
                        if (left_way == 2)//�����֧·����֪��һ����Դ��
                        {
                            left_way = 3;//��֧·��������ߵ����ȼ�
                            break_judge = 1;//׼����������ѭ��
                            turn_flag = 1;//��ת
                            x_turn = p;//����ת���ȷ���������ٸģ�
                            break;
                        }
                        else
                        {
                            left_way = 2;//��֧·��֪��һ����Դ��
                        }
                    }
                    else if (map[p][y_judge] == 2)//���ָ�֧·���ϰ���
                    {
                        if (left_way != 2)//��һ����Դ��Ļ���Ϊֻ��һ����Դ�㣬û����Դ��Ļ����·��ͨ
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if(y_judge == 9)//��������֧·���һ����
                    {
                        if (left_way != 2) left_way = 1;//�����֧·ɶҲû�У���Ϊ1
                    }
                }
                for (y_judge = (y - 1); y_judge >= 0; y_judge--)//�ж�С���Ҳ�·��
                {
                    if (map[p][y_judge] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//��ת
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
                if (left_way > main_way)//��⵽��ת�и��õĲ���
                {
                    x_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//��ʱȷ��ת��Ĳ��ԣ�������ò���һ��������ֵ
                }
                if (right_way > main_way)//��⵽��ת�и��õĲ���
                {
                    x_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//����ѭ������
                    break;
            }
        }
        int sources = 0;//ת����Ҳ�ö��������Ϊ�������ö��������Ϊ��
        for (p = y - 1; p >= 0; p--)//�ж��Ҳ�
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources++;
            else if (map[x_turn][p] == 1)
                sources = sources + 10;
        }
        for (p = y + 1; p < 10; p++)//�ж����
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
    //int sourse_point;//main_wayΪ2ʱ��ʾ��Դ�����ꣻ
    //int none_flag = 0;
    int y_judge=0;
    for (y_judge = y - 1; y_judge >= 0; y_judge--)//�ж���··��
    {
        if (map[x][y_judge] == 1)//������Դ��
        {
            if (main_way == 2)//����Ѿ�����һ����Դ��
            {
                main_way = 3;//������·��ߵ����ȼ�����������ѭ��
                break;
            }
            else
            {
                main_way = 2;
            }
        }
        else if (map[x][y_judge] == 2)//��⵽�ϰ���
        {
            if (main_way != 2)
            {
                main_way = 0;//������·��͵����ȼ�����ѡ��ת��Ĳ���
            }
            obstacle_point = y_judge;//��¼��·�ϵ�һ���ϰ����λ��
            break;
        }
        else if (y_judge == 0)//����������·���һ��ʱ
        {
            if (main_way != 2) main_way = 1;//�ж�����·��ɶҲû�У������������ȼ���Ϊ1
        }
    }
    y_turn = obstacle_point + 1;//���ñ���ת��㣬��ֹС������������ϰ���
    if (main_way == 1)//�����·ɶҲû�У�����С������ת�䣬ȥѰ���µĻ��ᣬ�������������֧·����Դ�Ļ���������Ĳ���
    {
        main_way = 0;//����·��Ϊ0���Ա���֧·ɶҲû�еĻ����ȼ�Ҳ������·���Ӷ�ʵ��С������ת��
        y_turn = y - 1;//��һ�����ת
    }
    //if (x < 5) turn_flag = 1;//�ı߿������ı�ת���Ա�Ѱ�ҵ��������Դ
    //else turn_flag = 2;
    if (main_way != 3)//�����·��û�����������ϵ���Դ��Ҫ����֧·
    {
        for (p = y - 1; p > obstacle_point; p--)
        {
            left_way=0;
            right_way=0;
            int break_judge = 0;//������������ѭ��
            int x_judge;
            for (x_judge = (x + 1); x_judge < 10; x_judge++)//�ж�С�����֧··��
            {
                if (map[x_judge][p] == 1)//��⵽��Դ��
                {
                    if (left_way == 2)//�����֧·����֪��һ����Դ��
                    {
                        left_way = 3;//��֧·��������ߵ����ȼ�
                        break_judge = 1;//׼����������ѭ��
                        turn_flag = 1;//��ת
                        y_turn = p;//����ת���ȷ���������ٸģ�
                        break;
                    }
                    else
                    {
                        left_way = 2;//��֧·��֪��һ����Դ��
                    }
                }
                else if (map[x_judge][p] == 2)//���ָ�֧·���ϰ���
                {
                    if (left_way != 2)//��һ����Դ��Ļ���Ϊֻ��һ����Դ�㣬û����Դ��Ļ����·��ͨ
                    {
                        left_way = 0;
                    }
                    break;
                }
                else if (x_judge == 9)//��������֧·���һ����
                {
                    if (left_way != 2) left_way = 1;//�����֧·ɶҲû�У���Ϊ1
                }
            }
            for (x_judge = (x - 1); x_judge >= 0; x_judge--)//�ж�С���Ҳ�·��
            {
                if (map[x_judge][p] == 1)
                {
                    if (right_way == 2)
                    {
                        right_way = 3;
                        //main_way = 3;
                        break_judge = 1;
                        turn_flag = 2;//��ת
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
            if (left_way > main_way)//��⵽��ת�и��õĲ���
            {
                y_turn = p;
                turn_flag = 1;
                main_way = left_way;//��ʱȷ��ת��Ĳ��ԣ�������ò���һ��������ֵ
            }
            if (right_way > main_way)//��⵽��ת�и��õĲ���
            {
                y_turn = p;
                turn_flag = 2;
                main_way = right_way;
            }
            if (break_judge == 1)//����ѭ������
                break;
        }
    }
    int sources = 0;//ת����Ҳ�ö��������Ϊ�������ö��������Ϊ��
    for (p = x - 1; p >= 0; p--)//�ж��Ҳ�
    {
        if (map[p][y_turn] == 2)
            break;
        else if (map[p][y_turn] == 0)
            sources++;
        else if (map[p][y_turn] == 1)
            sources = sources + 10;
    }
    for (p = x + 1; p < 10; p++)//�ж����
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
        int obstacle_point = -1;//main_wayΪ2ʱ��ʾ�ϰ��������
        //int sourse_point;//main_wayΪ2ʱ��ʾ��Դ�����ꣻ
        //int none_flag = 0;
        int x_judge;
        for (x_judge = x - 1; x_judge >= 0; x_judge--)//�ж���··��
        {
            if (map[x_judge][y] == 1)//������Դ��
            {
                if (main_way == 2)//����Ѿ�����һ����Դ��
                {
                    main_way = 3;//������·��ߵ����ȼ�����������ѭ��
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x_judge][y] == 2)//��⵽�ϰ���
            {
                if (main_way != 2)
                {
                    main_way = 0;//������·��͵����ȼ�����ѡ��ת��Ĳ���
                }
                obstacle_point = x_judge;//��¼��·�ϵ�һ���ϰ����λ��
                break;
            }
            else if (x_judge == 0)//����������·���һ��ʱ
            {
                if (main_way != 2) main_way = 1;//�ж�����·��ɶҲû�У������������ȼ���Ϊ1
            }
        }
        x_turn = obstacle_point + 1;//���ñ���ת��㣬��ֹС������������ϰ���
        if (main_way == 1)//�����·ɶҲû�У�����С������ת�䣬ȥѰ���µĻ��ᣬ�������������֧·����Դ�Ļ���������Ĳ���
        {
            main_way = 0;//����·��Ϊ0���Ա���֧·ɶҲû�еĻ����ȼ�Ҳ������·���Ӷ�ʵ��С������ת��
            x_turn = x - 1;//��һ�����ת
        }
        if (y < 5) turn_flag = 2;//�ı߿������ı�ת���Ա�Ѱ�ҵ��������Դ
        else turn_flag = 1;
        if (main_way != 3)//�����·��û�����������ϵ���Դ��Ҫ����֧·
        {
            int p;
            for (p = x - 1; p > obstacle_point; p--)
            {
                right_way=0;
                left_way=0;
                int break_judge = 0;//������������ѭ��
                int y_judge;
                for (y_judge = (y - 1); y_judge >= 0; y_judge--)//�ж�С�����֧··��
                {
                    if (map[p][y_judge] == 1)//��⵽��Դ��
                    {
                        if (left_way == 2)//�����֧·����֪��һ����Դ��
                        {
                            left_way = 3;//��֧·��������ߵ����ȼ�
                            break_judge = 1;//׼����������ѭ��
                            turn_flag = 1;//��ת
                            x_turn = p;//����ת���ȷ���������ٸģ�
                            break;
                        }
                        else
                        {
                            left_way = 2;//��֧·��֪��һ����Դ��
                        }
                    }
                    else if (map[p][y_judge] == 2)//���ָ�֧·���ϰ���
                    {
                        if (left_way != 2)//��һ����Դ��Ļ���Ϊֻ��һ����Դ�㣬û����Դ��Ļ����·��ͨ
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if (y_judge == 0)//��������֧·���һ����
                    {
                        if (left_way != 2) left_way = 1;//�����֧·ɶҲû�У���Ϊ1
                    }
                }
                for (y_judge = (y + 1); y_judge < 10; y_judge++)//�ж�С���Ҳ�·��
                {
                    if (map[p][y_judge] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//��ת
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
                if (left_way > main_way)//��⵽��ת�и��õĲ���
                {
                    x_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//��ʱȷ��ת��Ĳ��ԣ�������ò���һ��������ֵ
                }
                if (right_way > main_way)//��⵽��ת�и��õĲ���
                {
                    x_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//����ѭ������
                    break;
            }
        }
        int sources = 0;//ת����Ҳ�ö��������Ϊ�������ö��������Ϊ��
        for (p = y - 1; p >= 0; p--)//�ж����
        {
            if (map[x_turn][p] == 2)
                break;
            else if (map[x_turn][p] == 0)
                sources--;
            else if (map[x_turn][p] == 1)
                sources = sources - 10;
        }
        for (p = y + 1; p < 10; p++)//�ж��Ҳ�
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
        //int sourse_point;//main_wayΪ2ʱ��ʾ��Դ�����ꣻ
        //int none_flag = 0;
        int y_judge;
        for (y_judge = y + 1; y_judge < 10; y_judge++)//�ж���··��
        {
            if (map[x][y_judge] == 1)//������Դ��
            {
                if (main_way == 2)//����Ѿ�����һ����Դ��
                {
                    main_way = 3;//������·��ߵ����ȼ�����������ѭ��
                    break;
                }
                else
                {
                    main_way = 2;
                }
            }
            else if (map[x][y_judge] == 2)//��⵽�ϰ���
            {
                if (main_way != 2)
                {
                    main_way = 0;//������·��͵����ȼ�����ѡ��ת��Ĳ���
                }
                obstacle_point = y_judge;//��¼��·�ϵ�һ���ϰ����λ��
                break;
            }
            else if (y_judge == 9)//����������·���һ��ʱ
            {
                if (main_way != 2) main_way = 1;//�ж�����·��ɶҲû�У������������ȼ���Ϊ1
            }
        }
        y_turn = obstacle_point - 1;//���ñ���ת��㣬��ֹС������������ϰ���
        if (main_way == 1)//�����·ɶҲû�У�����С������ת�䣬ȥѰ���µĻ��ᣬ�������������֧·����Դ�Ļ���������Ĳ���
        {
            main_way = 0;//����·��Ϊ0���Ա���֧·ɶҲû�еĻ����ȼ�Ҳ������·���Ӷ�ʵ��С������ת��
            y_turn = y + 1;//��һ�����ת
        }
        if (x < 5) turn_flag = 2;//�ı߿������ı�ת���Ա�Ѱ�ҵ��������Դ
        else turn_flag = 1;
        if (main_way != 3)//�����·��û�����������ϵ���Դ��Ҫ����֧·
        {
            for (p = y + 1; p < obstacle_point; p++)
            {
                right_way=0;
                left_way=0;
                int break_judge = 0;//������������ѭ��
                int x_judge;
                for (x_judge = (x - 1); x_judge >= 0; x_judge--)//�ж�С�����֧··��
                {
                    if (map[x_judge][p] == 1)//��⵽��Դ��
                    {
                        if (left_way == 2)//�����֧·����֪��һ����Դ��
                        {
                            left_way = 3;//��֧·��������ߵ����ȼ�
                            break_judge = 1;//׼����������ѭ��
                            turn_flag = 1;//��ת
                            y_turn = p;//����ת���ȷ���������ٸģ�
                            break;
                        }
                        else
                        {
                            left_way = 2;//��֧·��֪��һ����Դ��
                        }
                    }
                    else if (map[x_judge][p] == 2)//���ָ�֧·���ϰ���
                    {
                        if (left_way != 2)//��һ����Դ��Ļ���Ϊֻ��һ����Դ�㣬û����Դ��Ļ����·��ͨ
                        {
                            left_way = 0;
                        }
                        break;
                    }
                    else if (x_judge == 0)//��������֧·���һ����
                    {
                        if (left_way != 2) left_way = 1;//�����֧·ɶҲû�У���Ϊ1
                    }
                }
                for (x_judge = (x + 1); x_judge < 10; x_judge++)//�ж�С���Ҳ�·��
                {
                    if (map[x_judge][p] == 1)
                    {
                        if (right_way == 2)
                        {
                            right_way = 3;
                            //main_way = 3;
                            break_judge = 1;
                            turn_flag = 2;//��ת
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
                if (left_way > main_way)//��⵽��ת�и��õĲ���
                {
                    y_turn = p;
                    turn_flag = 1;
                    main_way = left_way;//��ʱȷ��ת��Ĳ��ԣ�������ò���һ��������ֵ
                }
                if (right_way > main_way)//��⵽��ת�и��õĲ���
                {
                    y_turn = p;
                    turn_flag = 2;
                    main_way = right_way;
                }
                if (break_judge == 1)//����ѭ������
                    break;
            }
        }
        int sources = 0;//ת����Ҳ�ö��������Ϊ�������ö��������Ϊ��
        for (p = x - 1; p >= 0; p--)//�ж����
        {
            if (map[p][y_turn] == 2)
                break;
            else if (map[p][y_turn] == 0)
                sources--;
            else if (map[p][y_turn] == 1)
                sources = sources - 10;
        }
        for (p = x + 1; p < 10; p++)//�ж��Ҳ�
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

	while(1){//δ���յ����з�
		while((IFG2&UCA0RXIFG)==0);//�����ջ������Ƿ���
		buffer[i]= UCA0RXBUF;//����һ�����ݲ�����
		if(buffer[i]==10){//����

			char result = judgeState();
//			if(result == 'E'/*||result == 'S'*/){
//				speed = 2;
//				command = 7;//ͣ��
//				initIrr();
//			}
			if(result == 'S'){
			    speed = 2;
			    command = 7;//ͣ��
			    direction = old_direction;
			    initIrr();
			}
			else if(result == 'O'){}//�ɹ�������
			else if(result == 'F'){}//δ����ɹ�
			else if(result == 'G'){
			    times = 0;//��ʼ��ʱ
			    time_flag = 0;
				//speed = 0;
				command = old_command;//ȫ�ٳ���
				old_command = 1;
			}
			//�����ĸ�ָ�����ʹ��
//			else if(result == 'L')command = 4;
//			else if(result == 'R')command = 3;
//			else if(result == 'l')command = 9;
//			else if(result == 'r')command = 8;
			//for(i;i>0;i--)buffer[i] = 0;
			i = 0;//����
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
            command = 7;//ͣ��
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
		    map[x][y] = 0;//�Ե�
		    if(x != x_turn || y != y_turn)judge_turn_point();//�ж���һ��
		}

		//
		if(x == x_turn && y == y_turn){//����ת���
		    if(turn_flag == 1){
		        command = 9;//��ת
		        turn_flag = 0;
		        direction = (direction + 3) % 4;
		        judge_turn_point();
		    }
		    else if(turn_flag == 2){
			    command = 8;//��ת
			    turn_flag = 0;
			    direction = (direction + 5) % 4;
			    judge_turn_point();
			}
			P1IE &= ~BIT0;//�رո��ж�(ת��)
		}
		if((x != x_turn )||(y != y_turn))speed = 0;
		//����
		if(direction == 0 && x == x_turn - 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 1 && x == x_turn && y == y_turn + 1)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 2 && x == x_turn + 1 && y == y_turn)speed = 1/*,send(x,y,direction+4)*/;
		if(direction == 3 && x == x_turn && y == y_turn - 1)speed = 1/*,send(x,y,direction+4)*/;
		P1IFG &= ~(BIT0+BIT5);
	}
	//ת������ж�
	else if((P1IFG & BIT5) != 0 && (command == 5 || command == 6) && (P1IN & BIT5)==0){
		flag_left = 0,flag_right = 0;
		command = 1;//ֱ��
		speed = 3;//����
        initIrr();
	}
    else if((P1IFG & BIT5) != 0 && (command == 8 || command == 9) && (P1IN & BIT5)==0){
        flag_left = 0,flag_right = 0;
        command = 1;//ֱ��
        speed = 1;//����
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
//        times_sec = 0;//����
//        old_command = command;
//        command = 7;//ͣ��
//        turn_flag = 0;
//    }
}
