
//��������������������������������������������������������������������������������������������������������������������������������
//
//	@ ���ϸ� : wmilib.c
//	@ ���� : ��ɰ���ȸ ����Ϸκ�ƽ�� ���ķκ�(2013~) ������ ���� �����Լ�
//	@ ������ : �����Ƿ����б� ���� ����(carpedm.kr@daum.net)
//
//��������������������������������������������������������������������������������������������������������������������������������


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//���������������������������������� ��� ���� ����������������������������������//

//	<!--- ���� ��� --->	//
#define Pi 3.141592
#define WORD_SIZE_I 65536
#define WORD_SIZE_F 65536.0
#define C_AGL_F 360.0

//	<!--- �κ� Ư�� --->	//
#define WHEEL_R 125.0
#define ROBOT_R 176
#define PSD_AGL 40

//	<!--- ���� Ư�� --->	//
#define MM_ENCODER 191.0
#define MAX_MV ( 39 * WORD_SIZE_I )

//	<!--- ���� ��� --->	//
#define NO_ACC 30000
#define LOW_ACC 100
#define HIGH_ACC 200

//	<!--- �ӵ� ��� --->	//
#define CRAWL 70
#define WALK 250
#define RUN 450

//	<!--- ���� ��� --->	//
enum Color { RED = 1, BLUE, YELLOW, GREEN };

//	<!--- ���� �ּ� --->	//
enum Sensor { D1 = 0x01, D2 = 0x02, P1 = 0x04, P2 = 0x08, P3 = 0x10};

//	<!--- ���� ��� --->	//
enum CondType { DELETED, PSD, PSDRUN, OPT, IND, DISTANCE };

//	<!--- Bool ��� --->	//
enum Bool { false, true };

//  <!--- ���� ��� --->	//
enum Direction { FORWARD, BACK, RIGHT, LEFT };

//���������������������������������� ��� ���� ����������������������������������//


//���������������������������������� ���� ���� ����������������������������������//

//	<!--- ���� ���� ����ü --->	//
//	��� ���� : chkCond(), addCond(), �����
typedef struct ConditionType {
	long type;
	long sensor;
	long value;
	long velocity;
	long ignore;
} Cond;

//	<!--- ����  ����  �迭 --->	//
Cond Conds[10];

//	<!--- ���ڴ� ������ < �÷��� ���� > --->	//
volatile long dX,dY;
volatile double dER, dR, dA;
volatile double aER;

//	<!--- ���ڴ� ������ < �۷ι� ���� > --->	//
volatile double gER,gR,gA;

//	<!--- ���� �ӵ� ���� < ���ӵ� >  --->	 //
volatile double aFx, aFy, aFw;

//	<!--- LCD ���� --->
char lcdLine[21];

//���������������������������������� ���� ���� ����������������������������������//


//���������������������������������� ���� �Լ� ����������������������������������//

//	*******  ���� �Լ��� ���� ��� �Լ��� ���δ�.	*******//

//	<!--- ���밪 ��ȯ --->	//
long labs(long val) {
	return ( val >= 0 ) ? val : -val; 
}

//	<!--- ���� ��ȯ --->	//
//	[ ���� ] -> [ ���� ]
double rad(double deg) {
	return deg * Pi / 180.0; 
} 

//	<!---  ����  ��ȯ --->	//
//	[ ���� ] -> [ ���� ]
double deg(double rad) {
	return rad * 180.0 / Pi; 
}

//	<!--- 0 ~ 359 ���� ��ȯ --->
double u_angle(double agl) {
	return ( ((long)agl + 720) % 360 ); 
}

//	<!--- (-180) ~ 180 ���� ��ȯ --->	//
double h_angle(double agl) {
	agl = u_angle(agl);
	if( agl > 180 ) agl -= 360 ;
	return agl;
}

//	<!--- ���� ū ���� �ε��� ��ȯ --->	//
//	�迭[3] ���� �� ���밪�� ���� ū ������ �ε��� ��ȯ
long amax(long arr[]) { 
	int i = 0;	
	if( labs(arr[1]) > labs(arr[i]) ) i = 1;
	if( labs(arr[2]) > labs(arr[i]) ) i = 2;
	return i;
} 
//���������������������������������� ���� �Լ� ����������������������������������//


//���������������������������������� �������̽� �Լ� ����������������������������������//

//	<!--- ���� < count: �︲ Ƚ�� > --->	//
void BZ(int count ) { 
	int i;

	for( i = 0; i < count ; i++ ) {
		LED_ON(3); _delay_ms(50);
		LED_OFF(3); _delay_ms(50);
	}
}

//	<!--- LCD ���	--->
void display_line(int line, char *str) {
	int p = 0;
	while( str[++p] != '\0' );
	while( p < 20 ) str[p++]=' ';

	str[20]='\0';
	lcd_display_str(line,0,str);
}

//���������������������������������� �������̽� �Լ� ����������������������������������//



//���������������������������������� ���ڴ� ������ �Լ� ����������������������������������//

//	<!--- �÷��� ��ġ �б� --->	//
//	��� ���� ���� : ���ڴ� ������ ����.
//	��� ���� : chkCond(),VELOCITY ����� ��� �Լ�.
//	�÷����� ���ڴ� ���� �о�� ����ڰ� ��� �� �� �ְ� �������ִ� �Լ�.
//	**	�ſ� �߿��� �Լ� **
void readPosition() {
	long i, En[3];
	double ex, ey;

	//	���� 0������ 2�������� ����Ÿ� �б�.
	for ( i=0; i<3; i++) {			
		WriteCommand(i, RDRP);
		En[i] = 0;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i));
	}

	//	���ڴ� ������ ���ϱ�
	dER = ( En[0] + En[1] + En[2] ) / (2.985*WHEEL_R);
	ex = ( En[0] - En[2] ) / sqrt(3);
	ey = ( En[0] - ( 2 * En[1] + En[2] ) ) / 3.0;

	//	�÷��� ���� ���� ��ȯ	< ���ڴ� ���� -> �Ÿ� or ���� ���� >
	dR = dER / MM_ENCODER;	dA = deg(dR);
	dX = ex / MM_ENCODER;	dY = ey / MM_ENCODER;

	//	�۷ι� ���� ���� ��ȯ	< ���ڴ� ���� -> �Ÿ� or ���� ���� >
	gER = aER + dER;
	gR = gER / MM_ENCODER;	gA = u_angle(deg(gR));
	
}

//	<!--- �۷ι� ���� ���� --->	//
//	��� ���� ���� : gER, dER
//	�Է��� ������ ���ڴ� ������ ��ȯ�� �۷ι� ������ �缳�� ���ش�.
void setGA(long ga) {
	//	��ġ �����͸� ����.		
	readPosition();
	//	�۷ι� ������ �� ����.
	aER = ga * Pi * MM_ENCODER / 180.0 - dER;
	// �� �������� �����͸� �ٽ� ����.
	readPosition();
}

//	<!--- ���� ��ġ ���� ���ڴ� �ʱ�ȭ --->	//
//	��� ���� ���� : gER, dER
//	��� ���� :	VELORCITY ����� �Լ���.
//	�� �Լ��� ������ �ʾ����� ��ǥ���� ���� : 
//			- �� ��� �Լ��� �����ǰ� �� ���� ��� �Լ����� �ٷ� ������ ��찡 �߻��Ѵ�. [ �Ÿ� or ������ �����ϴ� �Լ����� ��� ]
//	** �߿��� �Լ� **
void setHome() {

	// ��ġ �����͸� ����
	readPosition();		
	
	// ���ڴ� ���� �ʱ�ȭ
	aER += dER;		
	dER=0;

	// ���ڴ� �ʱ�ȭ
	WriteCommand(0, DFH);
	WriteCommand(1, DFH);
	WriteCommand(2, DFH);

}

//���������������������������������� ���ڴ� ������ �Լ� ����������������������������������//

//���������������������������������� ���� ���� �Լ� ����������������������������������//

//	<!--- ���� �߰� --->	//
//	��� ���� ���� : Conds[10]
//	��� ���� : �����, chkCond(), PD,PT,ND,TG ��ũ��.
//	������ �߰��ϴ� �Լ��̸� ���뵵�� ���� �پ��� ������ �����Ҽ��ִ�.
//	���� ������ ���ÿ� �ɼ��ִ�.
void addCond(int t, int s, int val, int vel, int ig) {
	int i;
	for (i=1; i<10; i++) {
		if ( Conds[i].type == DELETED ) {
			Conds[i].type = t;
			Conds[i].sensor = s;
			Conds[i].value = val;
			Conds[i].velocity = vel;
			Conds[i].ignore = ig;
			break;
		}
	}
}

//	<!--- ���� ����Ʈ üũ --->	//
//	��� ���� ���� : Conds[10]
//	��� ���� :	VELOCITY ��� �Լ���.
//	������ �ɾ��´� �����ϰų� ���ϴ´�� �۵����� �ʾ������ ���Լ��� Ȯ���� ����.	** ��κ� ���⿡ ������ �ִ� �׸��� ���ðŸ��� �� Ȯ���� ���� **
//	** �߿��� �Լ� **
long chkCond() {	
	long rtn = 0;
	long i, dist, err;

	//	��ġ �����͸� �����ϰ� �� �����͸� �������� �̵��Ÿ� ���.
	readPosition();
	dist = sqrt(dX*dX + dY*dY) + labs(WHEEL_R * dR);
	
	for (i=1; i<10; i++) {
		if ( Conds[i].type != DELETED && dist >= Conds[i].ignore) {
			switch(Conds[i].type) {
				case OPT :
				case IND : 
					if( ( READ_SENSOR() & Conds[i].sensor ) == Conds[i].value ) rtn = i;
					break;
				case PSD :  
					if( Conds[i].value >= 0 && psd_value[Conds[i].sensor] >= Conds[i].value ) rtn = i;
					if( Conds[i].value < 0 && psd_value[Conds[i].sensor] < -Conds[i].value ) rtn = i;
					break;
				case PSDRUN :
					err = (Conds[i].value - psd_value[Conds[i].sensor]) * 3;
					if ( err > 0 ) Conds[i].velocity = err;
					else Conds[i].velocity = 0;
					rtn = i;
					break;
				case DISTANCE :
					if (dist >= Conds[i].value) rtn = i;
					break;
			}
		}
	}

	//	������ �������� [ �ӵ��� 0 ���� ]�� ���� ���� ������� [ ���� ���� ]�� �������� �ʴ´�.
	if ( rtn > 0 && Conds[rtn].velocity <= 0 ) {
		for (i=1; i<10; i++) Conds[i].type = DELETED; 
	}

	return rtn;
}

//���������������������������������� ���� ���� �Լ� ����������������������������������//

//���������������������������������� �ӵ� ��� �⺻ �Լ� ����������������������������������//

//	<!--- ���� ��� ���� --->	//
//	��� ���� : moveByAcc().
//	�÷����� �������� ��κ� ����ϴ� �Լ�.
//	�κ��� �������� ��Ģ���̳� ���� �ٲ���ϰų� ������ ���� ���� �����δٰ� �����Ǹ� �� �Լ��� Ȯ�� �ؾߵȴ�.
//	** �߿��� �Լ� **
void moveByVel(double fx, double fy, double fw) {
	int king;
	long V[3]={0.0,0};

	if( fx == 0 && fy == 0 && fw == 0) { StopMotion(9); return; }

	//	����ڰ� ���ϴ� �ӵ��� ���.
	V[0] = ((( 0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[1] = (((-0.065*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[2] = (((-0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;

	king = amax(V);

	// ���� ���� ���͸� �������� �ӵ� �缳��.
	if ( V[king] > MAX_MV ) {
		V[0] = MAX_MV * (V[0] / (double) labs(V[king]));  
		V[1] = MAX_MV * (V[1] / (double) labs(V[king])); 
		V[2] = MAX_MV * (V[2] / (double) labs(V[king])); 
	}
	
	//	��� ����.
	SetVelocity(0, V[0]);
	SetVelocity(1, V[1]);
	SetVelocity(2, V[2]);

	StartMotion();
}

//	<!--- ���� ���ӵ� ���� --->	//
//	��� ���� ���� : aFx, aFy, aFw
//	��� ���� : moveByAngle()
//	Ÿ�̸� ��� �Լ�. < curTime ��� >
//	���� ��� �����Լ� [ moveByVel() ]�� ������ ���ٰ� �Ǵܵ� ���� �κ��� �׷��� �������� ���� ��쿡�� �� �Լ��� Ȯ���ؾ� �ȴ�.
//	Ư�� Ÿ�̸� < curTime > �� �۵��ϴ��� Ȯ�� �غ��� ���� ù��°�� �߿��ϴ�. ** step �� �������� ���� **
//	**	�߿��� �Լ� **
void moveByAcc(long fx, long fy, long fw, long acc) {
	long Gap[3], king;
	double step;

	//	���ϴ� �ӵ��� ����ӵ��� ����
	Gap[0] = fx - aFx;
	Gap[1] = fy - aFy;
	Gap[2] = fw - aFw;

	//	���̰� ���� ū ������ ��ȣ
	king = amax(Gap);

	//	[ 1 ms ] ���� ������ �ӵ��� ( acc�� 1�ʵ��� �� [mm/s] ������ų �������� ������ )
	step = acc / 1000.0;

	//	���� �����ӵ��� ȸ���ӵ� ���� ���̰� ���� ũ�ٸ� �������� [ 2.2 ]�� ����.	** ȸ���� ������ �ӵ����� �����̴�. **
	if ( king == 2 ) step /= 2.2;

	if ( curTime > prevTime ) {

		// �� �Լ��� �� [ms]���� ȣ��Ǿ����� �˻��Ͽ� �׿� �´� ���ӵ��� ����ӵ��� ��ȭ��Ŵ
		if ( ( curTime - prevTime ) <= 200 ) step *= ( curTime - prevTime );
		else step *= 200;

		// ����ӵ��� ���ϴ¼ӵ� ���� Ŀ�� ��� ���ϴ� �ӵ��� ����.
		if ( step > labs(Gap[king]) ) { 
			aFx = fx; 
			aFy = fy; 
			aFw = fw;

		} 
		// �� ������ [ �������� ] ��ŭ �������Ͽ� �ӵ��� �缳��.
		else {						
			aFx += step * Gap[0] / labs(Gap[king]);
			aFy += step * Gap[1] / labs(Gap[king]);
			aFw += step * Gap[2] / labs(Gap[king]);
		}

		//	���ӵ� ������ ���θ�� ����.
		moveByVel(aFx, aFy, aFw);
		
		//	���� �ð��� ����
		prevTime = curTime; 
	}

}

//	<!--- ����� ��� �Լ� --->	//
//	��� ���� : VELOCITY ����� �Լ���.
//  ������ ���� �����Ͽ� ���ӵ� �(agl �ڷ����� double������ ����)
//	����ڰ� ����ϱ� ���� ������ �Լ�.	** ����� ������ ���� ����. **
void moveByAngle(double agl, long velocity, long fw, long acc) {
	long fx, fy;

	agl = u_angle(agl);
	
	fx = velocity * cos(rad(agl));
	fy = velocity * sin(rad(agl));
	
	moveByAcc(fx, fy, fw, acc);
}

//���������������������������������� �ӵ� ��� �⺻ �Լ� ����������������������������������//




//���������������������������������� ��ũ�� ���� ����������������������������������//

//	<!--- ���� ��ž ��ũ�� --->	//
//	�ӵ� ������ ������ �� ��ũ�ο��� ��������.
//	** ���ӵ� �ʱ�ȭ�� ���� �Ұ� **
#define CHK_STOP(X) { 	if( (X) > 0 ) { \
							if( Conds[X].velocity > 0 ) { velocity = Conds[X].velocity; } \
							else { \
								if(Conds[X].velocity == 0 ) { StopMotion(9); aFx = aFy = aFw = 0; } \
								break; \
							} \
						} \
					}

//	<!--- ���� ��ũ�� --->	//
#define PD( psd, gap, ig , vel )		addCond(PSD,psd,gap,vel,ig );
#define PT( sensor , ig )				addCond(OPT,sensor,sensor,0,ig );
#define ND( sensor , ig ) 				addCond(IND,sensor,sensor,0,ig );
#define TG( dist, vel ) 				addCond(DISTANCE,0,dist,vel,0);

//	<!--- ��� ���� ��ũ�� --->	//
#define MOTION_STOP() StopMotion(9); readPosition();

//���������������������������������� ��ũ�� ���� ����������������������������������//


//���������������������������������� �ӵ� ��� Ȱ�� �Լ� ����������������������������������//

//	<!--- ���� ���� --->	//
//	** ������ �ɾ� ���� ������� ������ �ʴ´�. **
//	���� �⺻���� ����.
long driveV(long agl, long velocity, long acc) {
	long rtn;

	setHome();

	while(1) {
		moveByAngle(agl, velocity, 0, acc);
		
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}

//	<!--- ���ڸ� ȸ�� --->
//	speed�� �ʴ� ȸ�� �ӵ��� �Ϲݼӵ� ó�� �������� ����.
//	���� �⺻���� ����.
long rotateV(long degree, int speed) { // ���ڸ� ȸ��
	long rtn;
	long velocity, dist;

	setHome();

	dist = labs(( 2 * Pi * WHEEL_R ) * ( degree / 360 ));
	addCond(DISTANCE,0,dist,0,0);

	if ( degree < 0 ) speed *= -1;

	while(1) {
		moveByAcc(0, 0, speed, HIGH_ACC);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;

} // end of rotateV()


//	<!--- ��ũ ���� --->	//
//	��� ���� ���� : aFx, aFy, aFw
//	Ȧ�γ�� ����
//	Ÿ�̸� ��� �Լ�. < curTime ��� >
long arcV(long agl, long gap, long arc, long velocity ) {  
	long rtn = 0;
	long drt = 1;
	long radius;
	double rotated = 0;
	double start, time;


	setHome();

	if( arc < 0 ) { drt = -1; arc *= -1; } 

	//	��ũ ������ ���� ������ < �÷��� �׵θ����� >	 **���� �߽ɰ� �򰥸��� ���� !!**
	radius = gap + ROBOT_R; 
	
	start = curTime;

	//	������ �������� ���ݱ��� ȸ���� ������ ū�� üũ
	while( labs(arc) >= labs(rotated) ) { 
		time = ( curTime - start ) / 1000.0;

		rotated = ( velocity * time  / ( 2 * Pi * radius ) ) * C_AGL_F; 

		moveByAngle( ( rotated + ( agl - 90 ) ) * drt , velocity , 0, NO_ACC);
		
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	//	���ӵ� �ʱ�ȭ	** �������� ���� ��ǿ� ������ ���� !! **
	aFx = aFy = aFw = 0; 

	MOTION_STOP();

	return rtn;
}

//	<!--- ��ũ ���� --->	//
//	�� Ȧ�γ�� ����
//	���ӵ��� ��� ������ ������ ��ũ �Լ���.
long farcV(long agl, long gap, long arc, long velocity,long acc) {
	long rtn;
	long drt = 1;
	long dist, radius;
	long fx, fy, fw;
	double radian;

	if( arc < 0 ) { drt = -1; arc *= -1; }

	//	��ũ ������ ���� ������ < �÷��� �׵θ����� >	 **���� �߽ɰ� �򰥸��� ���� !!**
	radius = gap + ROBOT_R;

	radian = rad( u_angle(agl-90) );

	dist = 2 * Pi * radius * ( arc / C_AGL_F );

	setHome();

	addCond(DISTANCE,0, dist + WHEEL_R * 2 * Pi * arc / C_AGL_F ,0, 0);

	fx = velocity * cos(radian);
	fy = velocity * sin(radian);
	fw = velocity * WHEEL_R * 2 * Pi * arc * 0.065 / (C_AGL_F * dist * 0.14);

	while(1) {
		moveByAcc((fx * drt), (fy * drt), (fw * drt), acc);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}


//	<!--- ��ũ ���� --->	//
//	��� ���� ���� : aFx, aFy, aFw
//	Ÿ�̸� �Լ� < curTime >
//	[ turnV ]�Լ��� [ arcV ]�Լ��� ��ģ �Լ�
//	�ݰ��� �ſ� Ŀ���ų� �ſ� ������� ��Ȯ���� ������.
//	**	���� �ְ�ӵ����� �� �����ӵ��� �䱸�ϴ� ���� �������� �ʵ��� ����	**
long tarcV(long agl, long gap, long arc, long  turn, long velocity) {  // ��ũ + �ʹ� �
	long rtn = 0;
	long drt = 1;
	long dist = 0, fw = 0, radius = 0;
	double start = 0, time = 0;
	double turned = 0, rotated = 0;

	if( arc < 0 ) { drt = -1; arc *= -1; }

	setHome();

	//	��ũ ������ ���� ������ < �÷��� �׵θ����� >	 **���� �߽ɰ� �򰥸��� ���� !!**
	radius = gap + ROBOT_R;
		
	//	�÷����� �����ġ���� ���ϴ� ���������� ���� ����[ ���� ȣ ]�� ���.
	dist = ( ( 2 * Pi * radius ) / C_AGL_F ) * arc;

	//	����� �����Ÿ�[ ���� ȣ]�� �������� �ʴ� ȸ���ӵ��� ���.
	fw = turn * velocity / dist; 

	start = curTime;

	//	������ �������� ���ݱ��� ȸ���� ������ ū�� üũ
	while( labs(arc) >= labs(rotated) ) {
		time = ( curTime - start ) / 1000.0;

		turned = time * fw;
		rotated =  ( velocity * time ) / ( 2 * Pi * radius ) * C_AGL_F ;

		// ( ��ũ ������� - ������ġ���� �ʹ��� ���� ) =  ���� ���� 
		moveByAngle( ( ( rotated - turned ) + ( agl - 90 ) ) * drt, velocity, fw, NO_ACC); 

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	//	���ӵ� �ʱ�ȭ	** �������� ���� ��ǿ� ������ ���� !! **
	aFx = aFy = aFw = 0;

	MOTION_STOP();

	return rtn;
}

//	<!--- ���� ȸ�� ���� --->
//	��� ���� ���� : aFx, aFy, aFw
//	Ÿ�̸� �Լ� < curTime >
//	**	���� �ְ�ӵ����� �� �����ӵ��� �䱸�ϴ� ���� �������� �ʵ��� ����	**
//	����� �۵����� ������� Ÿ�̸Ӹ� ���� ù��°�� Ȯ���غ���.
long turnV( long agl, long dist, long  turn , long velocity ) { // ( ���� )
	long rtn;
	long fw; 
	double start, time;
	double turned = 0;

	setHome();

	//	������ �����Ÿ��� �������� �ʴ� ȸ���ӵ��� ���.
	fw = turn * velocity / dist;

	start = curTime;

	//	������ �������� ���ݱ��� ȸ���� ������ ū�� üũ
	while( labs(turn) > labs(turned) ) {
		time = curTime - start/ 1000.0;
		turned = time * fw ;
		
		moveByAngle( agl - turned , velocity, fw, NO_ACC);
		
		rtn = chkCond();
     	CHK_STOP(rtn);
	}

	//	���ӵ� �ʱ�ȭ	** �������� ���� ��ǿ� ������ ���� !! **
	aFx = aFy = aFw = 0;

	MOTION_STOP();

	return DISTANCE;

} // end of turning()

//���������������������������������� �ӵ� ��� Ȱ�� �Լ� ����������������������������������//



//���������������������������������� ���� �� �� Ȱ�� �Լ� ����������������������������������//

//	<!--- �� ���� --->
//	��� ���� ���� : aFw
//	fw �� ����Ͽ� � ���̵� ���� ������ �����ϸ� ���� �Լ�
//	����Ÿ�� ���� ������ ��Ÿ��� ������.
//	�ܰŸ����� ������ ���� Ʋ������ **����**
//	Ȧ�γ�� ����
long hSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn;
	long wall, vel, between;

	setHome();

	while(1) {
		// ������ ���ܼ� ���� �����͸� ����Ҽ��ְ� ������.
		wall = psd_value[psd] - gap;
		vel = sqrt(aFx*aFx + aFy*aFy);

		// ����ϴ� ���ܼ� ���� ��ȣ�� �����͸� ����.
		between = ( 9 - psd ) * PSD_AGL - agl;

		// �� �����͸� �̿��Ͽ� ������ ����
		if( u_angle(between) > 180 ) wall *= 1;
		else  wall *= -1;

		aFw = wall * ( 700  - vel ) / 800.0;		

		moveByAngle(agl,velocity,aFw,acc);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}

//	<!--- �� ���� --->
//	���� ������ ���°� �ƴ϶� ���� Ż�������� ���� �Ÿ��� �����ϸ� ����.
//	����Ÿ�� �������� �����Ƿ� �ܰŸ��� ������
//	0�� 120�� 240���� �ڿ��Ͽ� ��Ÿ��� ��밡�� �ٸ������ϰ�� ������ �߻��Ҽ� ����.
//	�� Ȧ�γ�� ����
long nhSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn;
	long wall, go_agl;

	setHome();

	while(1) {
		// ������ ���ܼ� ���� �����͸� ����Ҽ��ְ� ������.
		wall = gap - psd_value[psd];
		go_agl = agl + wall/3;

		moveByAngle(go_agl,velocity,0,acc);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}


//	<!--- ���� ���� --->
//	�� Ȧ�γ�� ����
long nhLine( long agl, long gap, int sensor, long velocity, int line ) {	// ����
	long rtn;
	long linein;

	setHome();

	while(1) {
		if( READ_SENSOR() & sensor ) linein = true;
		else linein = false;
		
		if( line == LEFT && linein == true )
			moveByAngle(agl + gap,velocity,0,NO_ACC);

		else
			moveByAngle(agl - gap,velocity,0,NO_ACC);			 

		if( line == RIGHT && linein == true)
			moveByAngle(agl - gap,velocity,0,NO_ACC);

		else 
			moveByAngle(agl + gap,velocity,0,NO_ACC);

		rtn = chkCond();
		CHK_STOP(rtn);
	}
	return rtn;
}

//	<!--- ���� ���� --->
//	�� Ȧ�γ�� ����
long hLine(long agl, long velocity, long gap, int sensor1, int sensor2) {
	long rtn;
	long both;

	setHome();

	both = sensor1 | sensor2;

	while(1) {
		if( ( READ_SENSOR() & both ) == both ) {
			moveByAngle(agl,velocity,0,NO_ACC);
		}
		else if( ( READ_SENSOR() & sensor1 ) == sensor1 ) {
			moveByAngle(agl,velocity,gap, NO_ACC); 
		}
		else if( ( READ_SENSOR() & sensor2 ) == sensor2 ) {
			moveByAngle(agl,velocity,-gap, NO_ACC);
		}

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}

//���������������������������������� ���� �� �� Ȱ�� �Լ� ����������������������������������//

//���������������������������������� ī�޶� Ȱ�� �Լ� ����������������������������������//

//	<!--- ������ ������ X ��ǥ ��ȯ --->
long getColX( int c ) {
	return Camera_Cmd(c,Color_X);
}

//	<!--- ������ ������ Y ��ǥ ��ȯ --->
long getColY( int c ) {
	return Camera_Cmd(c,Color_Y);
}

//	<!--- ������ ������ ���� ��ȯ --->
long getColH( int c ) {
	return Camera_Cmd(c,Color_H);
}

//	<!--- ������ ������ ���� ��ȯ --->
long getColW( int c ) {
	return Camera_Cmd(c,Color_W);
}

//	<!--- ������ ������ ���� ��ȯ --->
long getColC( int c ) {
	return Camera_Cmd(c,Color_C);
}

//	<!--- X ��ǥ �̵� �Լ� --->
long ColorFB( int color , int posX ) {
	long errX = 100, colX = 100;

	setHome();

	//	��ǥ �̼� ������ ������ ��� ���� �ٲٸ鼭 �Ҽ��ִ�.
	while(labs(errX) > 5) {
		colX = getColX(color);
		errX = colX - posX;

		// errX * ��� [ �̼� ���� ]
		moveByAcc(errX*3, 0, 0 , LOW_ACC);
	}

	MOTION_STOP();

	return colX; 
}

//	<!--- Y ��ǥ �̵� �Լ� --->
//	��忡 0�� ������ ��� �¿� ����� Y ��ǥ�� �����.
//	��忡 1�� ������ ��� ȸ�� ����� Y ��ǥ�� �����.
long ColorLR( int color , int posY , int mode ) {
	long errY = 100, colY = 100;

	setHome();

	//	��ǥ �̼� ������ ������ ��� ���� �ٲٸ鼭 �Ҽ��ִ�.
	while(labs(errY) > 5) {		
		colY = getColY(color);
		errY = colY - posY;

		// errY * ��� [ �̼� ���� ]
		if( mode == 0 )
			moveByAcc(0, errY*3, 0 , LOW_ACC);

		else
			moveByAcc(0, 0, errY*3, LOW_ACC);
	}

	MOTION_STOP();

	return colY; 
}

//���������������������������������� ī�޶� Ȱ�� �Լ� ����������������������������������//
