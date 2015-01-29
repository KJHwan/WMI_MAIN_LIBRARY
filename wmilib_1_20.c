/***********************************************************************************************************************************
	@ ���ϸ� : wmilib.c
	@ ���� : ��ɰ���ȸ ����Ϸκ�ƽ�� ���ķκ�(2013~) ������ ���� �����Լ�
	@ ������ : �����Ƿ����б� ���� ����(carpedm.kr@daum.net)
	@ ������ ������ ; 01.23.2015

***********************************************************************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ��� ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//************* ���� ��� *************//
#define Pi 3.141592
#define WORD_SIZE_I 65536
#define WORD_SIZE_F 65536.0
#define C_AGL_I 360
#define C_AGL_F 360.0

//************** �κ� Ư�� *************//
#define WHEEL_R 125.0
#define ROBOT_R 176
#define PSD_AGL 40

//************** ���� Ư�� *************//
#define MM_ENCODER 191.0
#define MAX_MV ( 39 * WORD_SIZE_I )

//************ ���ӵ� ��� *************//
#define NO_ACC 30000
#define LOW_ACC 100
#define HIGH_ACC 200

//************* �ӵ� ��� *************//
#define CRAWL 70
#define WALK 250
#define RUN 450
#define TURBO 600

//************** ���� �ּ� *************//
enum Sensor{D1=0x01, D2=0x02, P1=0x04, P2=0x08, P3=0x10};

//************* �Ÿ� ��� *************//
#define BREAK 200

//************* ���� ��� *************//
enum Color{ RED = 1, BLUE, YELLOW, GREEN };

//************* ���� ��� *************//
enum CondType { DELETED, PSD, PSDRUN, OPT, IND, DISTANCE, CAMERA };

//*********** BOOLEAN��� ************//
enum Bool { false, true };

//******** �����Լ� ��ȯ��� *********//
enum ColorReturn { MIN, MAX }; 

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ��� ���� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//********* ����� ���ǰ��� ��ȯ����ü **********//
typedef struct ConditionType {
	int type;
	int sensor;
	int value;
	int velocity;
	int ignore;
} Cond;

Cond Conds[10];

//**************** ���ڴ� ������ *****************//
volatile long dX,dY;
volatile double dER, dR, dA;
volatile double aER;
volatile double gER,gR,gA;

//*************** ���� �ӵ� ���� *****************//
volatile double aFx, aFy, aFw;

char lcdLine[21],mi;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� ���� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

long labs(long val) { return ( val >= 0 ) ? val : -val; } // ���밪 ��ȯ

double frad(double deg) { return deg * Pi / 180.0; } // ���� ��ȯ [ ���� �Է� ]

double deg(double rad) { return rad * 180.0 / Pi; } // ���� ��ȯ [ ���� �Է� ]

double u_angle(double agl) { return ( ((long)agl + 720) % 360 ); } // 0~359 ���� ��ȯ

double h_angle(double agl) {  // -180~180 ���� ��ȯ
	agl = u_angle(agl);
	if( agl > 180 ) agl -= 360 ;
	return agl;
}

int amax(long arr[]) {	// �迭 ���� �� ���밪�� ���� ū ������ �ε��� ��ȯ
	int i = 0;	
	if( labs(arr[1]) > labs(arr[i]) ) i = 1;
	if( labs(arr[2]) > labs(arr[i]) ) i = 2;
	return i;
} 
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> �κ� �������̽� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void BZ(int c) { // ���� �︮�� (count: �︲ Ƚ��)
	int i;

	for( i = 0; i < c; i++ ) {
		LED_ON(3); _delay_ms(50);
		LED_OFF(3); _delay_ms(50);
	}
} // end of BZ()


void display_line(int line, char *str) { // LCD �� �� �� ��� (line: ����� �� ��ȣ)
	int p = 0;
	while(str[++p]!='\0');
	while(p < 20) str[p++]=' ';

	str[20]='\0';
	lcd_display_str(line,0,str);
} // end of display_line()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< �κ� �������̽� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� ���ڴ� ���� �м� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void readPosition() {	// ���� ��ġ ���
	char axis[30];
	long i, En[3];
	double ex, ey;

	// ���� 0������ 2�������� ����Ÿ� �б�
	for ( i=0; i<3; i++) {			
		WriteCommand(i, RDRP);
		En[i] = 0;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i));
	}

	// ȸ������ ���ڴ� ������ ���ϱ�
	dER = (En[0] + En[1] + En[2]) / (2.985*WHEEL_R); // [ ( 1_12 ) Modify 2.985 -> 2.785 ]
	ex = (En[0] - En[2]) / sqrt(3);
	ey = (En[0] - 2*En[1] + En[2]) / 3.0;

	dR = dER / MM_ENCODER;	dA = deg(dR);
	dX = ex / MM_ENCODER;	dY = ey / MM_ENCODER;

	gER = aER + dER;
	gR = gER / MM_ENCODER;	gA = u_angle(deg(gR));
	
	// LCD 4°�ٿ� �۷ι�(����) ��ǥ ǥ��
	sprintf(axis, "[%4ld]", (long) gA);	lcd_display_str(3,14,axis);
} // end of readPosition()


void setGA(long ga) {		// �۷ι� ���� ����
	readPosition();
	aER = ga * Pi * MM_ENCODER / 180.0 - dER;
	readPosition();
} // end of setGA

void setHome() {	// ���� �����ǿ��� ���ڴ� �ʱ�ȭ

	// ��� ��ǥ�� �ʱ�ȭ
	readPosition();		
	
	aER += dER;		
	dER=0;

	// ���ڴ� �ʱ�ȭ
	WriteCommand(0, DFH);
	WriteCommand(1, DFH);
	WriteCommand(2, DFH);

} // end of setHome()

unsigned int ReadSignal(int motor) {	// ���� ��ȣ �б�(������ �̵��� ��ǥ���� ���޿��θ� Ȯ���ϱ� ���� ���)
	unsigned int sig = 0;
	WriteCommand(motor, RDSIGS);
	sig = (sig|ReadData(motor)) << 8;
	sig = (sig|ReadData(motor));
	return sig;
} // end of ReadSignal()
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� ���ڴ� ���� �м� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� ���� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
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

long chkCond() {	// �ӵ���� ���� �� ���Ǹ���Ʈ �˻� �� �ӵ��� ��ȯ
	long distance, i, rtn=0, err;
	readPosition();
	distance = sqrt(dX*dX + dY*dY) + labs(WHEEL_R * dR);

	
	for (i=1; i<10; i++) {
		if ( Conds[i].type != DELETED && distance >= Conds[i].ignore) {
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
					if (distance >= Conds[i].value) rtn = i;
					break;
				case CAMERA : break;

			}
		}
	}

	if ( rtn > 0 ) { // ���ǿ� ���� ��
		//if ( Conds[rtn].velocity > 0 ) 	Conds[rtn].type = DELETED;  		// ���� ���Ǹ� ����
		if ( Conds[rtn].velocity <= 0 ) for (i=1; i<10; i++) Conds[i].type = DELETED; 					// ��� ���� ����
	}	

	return rtn;

} //end of chdCond()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� ���� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSITIOION ��� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void movePosition(long dx, long dy, long dw, int maxV, int acc) {	// POSITION ���� �����Լ�
	long king=0, D[3], V[3], A[3];
	double maxD;

	if ( dx==0 && dy==0 && dw==0 ) return;
	StopMotion(9);

	setHome();  

	D[0] = ( 0.5*sqrt(3)*dx + 0.5*dy + dw) * MM_ENCODER;
	D[1] = (-dy + dw) * MM_ENCODER;
	D[2] = (-0.5*sqrt(3)*dx + 0.5*dy + dw) * MM_ENCODER;
	
	king = amax(D);
	maxD = D[king];

	maxV *= 0.065;
	V[0] = maxV * fabs(D[0] / maxD) * WORD_SIZE_I;
	V[1] = maxV * fabs(D[1] / maxD) * WORD_SIZE_I;
	V[2] = maxV * fabs(D[2] / maxD) * WORD_SIZE_I;

	A[0] = acc * fabs(D[0] / maxD);
	A[1] = acc * fabs(D[1] / maxD);
	A[2] = acc * fabs(D[2] / maxD);

	SetPosition(0, 0, A[0], V[0], D[0]);
	SetPosition(1, 0, A[1], V[1], D[1]);
	SetPosition(2, 0, A[2], V[2], D[2]);
	StartMotion();

} //end of movePositon()

void farcP( long agl, long gap, long arc,long velocity, long acc ) { 	///// ���ϴ� ������ ��ũ �
	long distance, radius, dx, dy, dw, dtr = 1;
	double rad;	
	
	if( arc < 0 ) { dtr = -1; arc *= -1; }

	radius = gap + ROBOT_R;
	rad = frad( u_angle(agl-90) );

	distance = 2*Pi*radius*(arc/C_AGL_F);

	dx = distance * cos(rad);
	dy = distance * sin(rad);
	dw = WHEEL_R * 2 * Pi * (arc/C_AGL_F);

	movePosition(dx*dtr, dy*dtr, dw*dtr, velocity, acc);
	while( !(ReadSignal(0) & ReadSignal(1) & ReadSignal(2) & 0x0400) );


} // end of arcP()


void driveP(long agl, long distance){  	///// ���ϴ� ����� ���ϴ� �Ÿ���ŭ �����
	long dx, dy;
	double rad;
	
	rad = frad(u_angle(agl));

	dx = distance * cos(rad);
	dy = distance * sin(rad);

	movePosition( dx, dy, 0 , RUN, HIGH_ACC);
	while( !(ReadSignal(0) & ReadSignal(1) & ReadSignal(2) & 0x0400) );

} // end of nhP();

void rotateP( long degree ){  			///// ���ϴ� ������ ���ڸ� ȸ��(�������� ��� �ð�ݴ����)
	return farcP(0, -ROBOT_R, degree,RUN,HIGH_ACC);
} //end of  rotateP()

void corAngle(long ga) {		///// �۷ι� ������ �̿��Ͽ� ����
	rotateP( h_angle(ga-gA) );
} // end of corAngle()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< POSITION ��� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> VELOCITY ��� �⺻ �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
void moveByVel(double fx, double fy, double fw) {			////// ������ �ӵ��� � (fx, fy, fw �ڷ����� double������ ����)
	int king;
	long V[3]={0.0,0};

	if(fx==0 && fy==0 && fw==0) { StopMotion(9); return; }

	V[0] = ((( 0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[1] = (((-0.065*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[2] = (((-0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;

	king = amax(V);

	if ( V[king] > MAX_MV ) {
		V[0] = MAX_MV * (V[0] / (double) labs(V[king]));  
		V[1] = MAX_MV * (V[1] / (double) labs(V[king])); 
		V[2] = MAX_MV * (V[2] / (double) labs(V[king])); 
	}
	
	SetVelocity(0, V[0]);
	SetVelocity(1, V[1]);
	SetVelocity(2, V[2]);

	StartMotion();
} // end of moveByVel()


void moveByAcc(long fx, long fy, long fw, long acc) {			////// ���ӵ��� �(aFx, aFy, aFw�� ���� �ӵ��� ������)
	long Gap[3], king;
	double step;

	// ���ϴ� �ӵ��� ����ӵ��� ���� (�������� aFx, aFy, aFw �� ������ ���Ϳ� ������ �ӵ���)
	Gap[0] = fx - aFx;
	Gap[1] = fy - aFy;
	Gap[2] = fw - aFw;

	// ���̰� ���� ū ������ ��ȣ
	king = amax(Gap);

	// 1ms���� ������ �ӵ��� (acc�� 1�ʵ��� ��mm/s ������ų �������� ������)
	step = acc / 1000.0;

	// ���� ȸ���ӵ����� ���̰� ���� ũ�ٸ� �������� 2.2�� ����(ȸ���� ������ �ӵ����� ����)
	if ( king == 2 ) step /= 2.2;

	if ( curTime > prevTime ) {	 
		// �� �Լ��� ��ms���� ȣ��Ǿ����� �˻��Ͽ� �׿� �´� ���ӵ��� ����ӵ��� ��ȭ��Ŵ
		if ( (curTime - prevTime) <= 200 ) step *= (curTime-prevTime);
		else step *= 200;

		if ( step > labs(Gap[king]) ) { // ��ȭ��ų �ӵ����� ���ϴ� ��ȭ�� ���� ũ�ٸ� �־��� �ӵ���ŭ �ٷ� ��ȭ��Ŵ
			aFx = fx; aFy = fy; aFw = fw;
		} else {						// �� ������ �������̸�ŭ �������Ͽ� �ӵ��� ����
			aFx += step * Gap[0] / labs(Gap[king]);
			aFy += step * Gap[1] / labs(Gap[king]);
			aFw += step * Gap[2] / labs(Gap[king]);
		}
		// ������ �ӵ��� �
		moveByVel(aFx, aFy, aFw);
		
		prevTime = curTime; // ���� �ð��� ����
	}

} // end of moveByAcc()

void moveByAngle(double agl, long velocity, long fw, long acc) {	//////  ������ ���� �����Ͽ� ���ӵ� �(agl �ڷ����� double������ ����)
	long fx, fy;
	agl = u_angle(agl);
	fx = velocity * cos(frad(agl));
	fy = velocity * sin(frad(agl));
	moveByAcc(fx, fy, fw, acc);
} // end of moveByAngle()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VELOCITY ��� �⺻ �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//




//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ��ũ�� ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
#define CHK_STOP(X) { 	if( (X) > 0 ) { \
							if( Conds[X].velocity > 0 ) { velocity = Conds[X].velocity; } \
							else { \
								if(Conds[X].velocity == 0 ) { StopMotion(9); aFx = aFy = aFw = 0; } \
								break; \
							} \
						} \
					}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ��ũ�� ���� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> VELOCITY ��� Ȱ�� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

long driveV(long agl, long velocity, long acc) { // ������ ������ ����
	long rtn;

	while(1) {
		moveByAngle(agl, velocity, 0, acc);
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
} // end of driveV();

long arcV(long agl, long gap, long arc, long velocity ) {  // Ȧ�γ�� ��ũ �
	long rtn = 0, dtr = 1,radian = 0;
	double start = 0, time = 0;
	double rotated = 0;

   setHome();

   radian = gap + ROBOT_R; // ���� ������

   if( arc < 0 ) { dtr = -1; arc *= -1; } // ���⼳��
   
   start = curTime; // ���� �ð� ����

   while(labs(arc) >= labs(rotated)) { // ������ ���� >= ���� �����ִ� ���� ���� üũ
      time = ( curTime - start ) / 1000.0;
 
      rotated = ( velocity  / ( 2 * Pi * radian ) ) * C_AGL_F; // ��ũ ���� ����

      moveByAngle( ( rotated + ( agl - 90 ) ) * dtr , velocity , 0, HIGH_ACC);
      rtn = chkCond();
      CHK_STOP(rtn);
   }

   aFx = aFy = aFw = 0; // ���ӵ� ���� �ʱ�ȭ
   StopMotion(9);

   readPosition();

   return rtn;
} // end of farcV()

long farcV(long agl, long gap, long arc, long velocity,long acc) {  // ��ũ �
	long rtn, dist, radius,fx,fy,fw, dtr = 1;
	double rad;

	if( arc < 0 ) { dtr = -1; arc *= -1; }

	radius = gap + ROBOT_R;
	rad = frad( u_angle(agl-90) );

	dist = 2*Pi*radius*(arc/C_AGL_F);

	setHome();

	addCond(DISTANCE,0, dist + WHEEL_R * 2 * Pi * arc / C_AGL_F ,0, 0);

	while(1) {
		fx = velocity * cos(rad);
		fy = velocity * sin(rad);
		fw = velocity * WHEEL_R * 2 * Pi * arc * 0.065 / (C_AGL_F * dist * 0.14);

		moveByAcc(fx*dtr, fy*dtr, fw*dtr, acc);
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;

} // end of arcV()

long tarcV(long agl, long gap, long arc, long  turn, long velocity) {  // ��ũ + �ʹ� �
	long rtn = 0, dtr = 1;
	long dist = 0, fw = 0, radian = 0;
	double start = 0, time = 0;
	double turned = 0, rotated = 0;

	if( arc < 0 ) { dtr = -1; arc *= -1; }

	setHome();

	radian = gap + ROBOT_R; // ���� ������
		
	dist = ( ( 2 * Pi * radian ) / C_AGL_F ) * arc; // ��ȣ�� ����

	fw = turn * velocity / dist; // �ʴ� ���� 

	start = curTime;

	while( labs(arc) >= labs(rotated) ) { // ������ ���� >= ���� �����ִ� ���� ���� üũ
		time = ( curTime - start ) / 1000.0;

		turned = time * fw; // �ʹ� ����
		rotated =  ( velocity * time ) / ( 2 * Pi * radian ) * C_AGL_F ; // ��ũ ���� ����


		moveByAngle( ( ( rotated - turned ) + ( agl - 90 ) ) * dtr, velocity, fw, NO_ACC); // ( ������� - ������ġ���� �ʹ��� ���� ) =  ���� ���� 
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	aFx = aFy = aFw = 0; // ���ӵ� ���� �ʱ�ȭ
	StopMotion(9);

	readPosition();

	return rtn;
} // end of tarcV()

long rotateV(long degree, int speed) { // ���ڸ� ȸ��
	long rtn, velocity, distance;

	velocity = speed;
	setHome();

	distance = labs(2*Pi*WHEEL_R*degree/360);
	addCond(DISTANCE,0,distance,0,0);

	while(1) {
		if ( degree < 0 && velocity > 0 ) velocity *= -1;
		velocity /= 6;
		moveByAcc(0, 0, velocity, HIGH_ACC);
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;

} // end of rotateV()


long turningV( long agl, long dist, long  turn , long velocity ) { // ( ���� )
	long  time = 0, start = 0, fw = 0, rtn = 0;
	double turned = 0;

	setHome();

	fw = turn * velocity / dist;

	start = curTime;

	while( labs(turn) > labs(turned) ) {
		time = curTime - start;
		turned = time * fw / 1000.0;
		moveByAngle( agl - turned , velocity, fw, NO_ACC);
		rtn = chkCond();
     	CHK_STOP(rtn);
	}

	aFx = aFy = aFw = 0; // ���ӵ� ���� �ʱ�ȭ

	StopMotion(9);

  	readPosition();	

	return 0;

} // end of turning()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VELOCITY ��� Ȱ�� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� �� �� Ȱ�� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

///////////////////////////
long hSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn = 0;

	return rtn;
}
long nhSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn,wall, go_agl;

	setHome();

	while(1) {
		wall = gap - psd_value[psd];
		go_agl = agl + wall/3;
		moveByAngle(go_agl,velocity,0,acc);
		rtn = chkCond();
		CHK_STOP(rtn);
	}
	return rtn;
}
/////////////////////////////

/////////////////////////
long nhLine( long agl, long velocity, long gap, int sensor1, int sensor2 ) {
	long rtn = 0;

	return rtn;
}
long hLine(long agl, long velocity, long gap, int sensor1, int sensor2 ) {
	long rtn = 0;

	return rtn;
}
////////////////////////


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� �� �� Ȱ�� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> COLOR Ȱ�� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

long getColX( int c ) {
	return Camera_Cmd(c,Color_X);
}

long getColY( int c ) {
	return Camera_Cmd(c,Color_Y);
}

long getColH( int c ) {
	return Camera_Cmd(c,Color_H);
}

long getColW( int c ) {
	return Camera_Cmd(c,Color_W);
}

long getColC( int c ) {
	return Camera_Cmd(c,Color_C);
}

long mvroCol(long color, long psd, long mode) {
	long posY,errY,colY;

	setHome();

	while(1) {
		posY = 120 + psd * ( getColW(color) / 2 + 10 );
		colY = getColY(color);
		errY = colY - posY;

		if( labs(errY) < 10 ) { StopMotion(9); return colY; }
		else { ( mode == 0 ) ? moveByAcc(0, errY*3, 0 , LOW_ACC) : moveByAcc(0, 0, errY*3, LOW_ACC); }

	}
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< COLOR Ȱ�� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ���� �Լ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

/*//////////////// �ڵ� ����,�߰� �� ���� �ؾ������� ���� ////////////////////////
long Line(long agl, long velocity, long gap, int sensor1, int sensor2) {
	int rtn, both, outss=0, intime = 0;

	setHome();
	both = sensor1 | sensor2;

	if( sensor1 && sensor2 ) {
		while(1) {
			if( ( READ_SENSOR() & both ) == both ) moveByAngle(agl,velocity,0,NO_ACC);
			else if( ( READ_SENSOR() & sensor1 ) == sensor1 ) { moveByAngle(agl,velocity,gap, NO_ACC); outss = sensor2; }
			else if( ( READ_SENSOR() & sensor2 ) == sensor2 ) { moveByAngle(agl,velocity,-gap, NO_ACC); outss = sensor1; }
			rtn = chkCond();
			CHK_STOP(rtn);
		}
	}

	else
		while(1) {
			if( sensor1 ) {
				if( ( READ_SENSOR() & sensor1 ) == sensor1 ) { moveByAngle(agl,velocity,gap, NO_ACC); intime = curTime; }
				else { ( curTime - intime > 100 ) ? moveByAngle(agl,CRAWL,-50,LOW_ACC) : moveByAngle(agl,velocity,-gap, NO_ACC); }
			}
			else if( sensor2 ) {
				if( ( READ_SENSOR() & sensor2 ) == sensor2 ) { moveByAngle(agl,velocity, -gap, NO_ACC); intime = curTime; }
				else { ( curTime - intime > 100 ) ? moveByAngle(agl,CRAWL,50,LOW_ACC) : moveByAngle(agl,velocity,gap, NO_ACC); }
			}
			rtn = chkCond();
			CHK_STOP(rtn);
		}
	return 0;
}
*/ 

/*
long ucV(long agl, long gap, long arc, long velocity ) {  // Ȧ�γ�� ��ũ �
   long rtn = 0, dtr = 1,radian = 0;
   double prevTime = 0, time = 0;
   double rotated = 0;

   setHome();

   radian = gap + ROBOT_R; // ���� ������

   if( arc < 0 ) { dtr = -1; arc *= -1; } // ���⼳��
   
   prevTime = curTime; // ���� �ð� ����

   while(labs(arc) >= labs(rotated)) { // ������ ���� >= ���� �����ִ� ���� ���� üũ
      time = ( curTime - prevTime ) / 1000.0;
      prevTime = curTime; 
      rotated += ( sqrt(aFx*aFx+aFy*aFy) * time ) / ( 2 * Pi * radian ) * C_AGL_F; // ��ũ ���� ����

      moveByAngle( ( rotated + ( agl - 90 ) ) * dtr , velocity , 0, NO_ACC);
      rtn = chkCond();
      CHK_STOP(rtn);

   }

   aFx = aFy = aFw = 0; // ���ӵ� ���� �ʱ�ȭ
   StopMotion(9);

   readPosition();

   return rtn;
} // end of ucV()

*/


/*
long side( long agl, long psd, long gap, long velocity, long acc ) { // ( ���� )
	long rtn,wall,vel,between;

	setHome();

	while(1) {
		wall = psd_value[psd] - gap;
		vel = sqrt(aFx*aFx + aFy*aFy);
		between = ( 9 - psd ) * PSD_AGL - agl;

		( u_angle(between) > 180 ) ? ( wall *= 1 ) : ( wall *= ( -1 ) );
		aFw = wall * ( 700  - vel ) / 800.0;
		

		moveByAngle(agl,velocity,aFw,acc);
		rtn = chkCond();
		CHK_STOP(rtn);
	}
	return rtn;
} // end of side()

*/


// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ���� �Լ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< //
