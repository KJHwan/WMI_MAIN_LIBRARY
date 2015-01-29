/***********************************************************************************************************************************
	@ 파일명 : wmilib.c
	@ 설명 : 기능경기대회 모바일로보틱스 공식로봇(2013~) 구동을 위한 구동함수
	@ 개발자 : 원주의료고등학교 교사 강상성(carpedm.kr@daum.net)
	@ 마지막 수정일 ; 01.23.2015

***********************************************************************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 상수 정의 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//************* 숫자 상수 *************//
#define Pi 3.141592
#define WORD_SIZE_I 65536
#define WORD_SIZE_F 65536.0
#define C_AGL_I 360
#define C_AGL_F 360.0

//************** 로봇 특성 *************//
#define WHEEL_R 125.0
#define ROBOT_R 176
#define PSD_AGL 40

//************** 모터 특성 *************//
#define MM_ENCODER 191.0
#define MAX_MV ( 39 * WORD_SIZE_I )

//************ 가속도 상수 *************//
#define NO_ACC 30000
#define LOW_ACC 100
#define HIGH_ACC 200

//************* 속도 상수 *************//
#define CRAWL 70
#define WALK 250
#define RUN 450
#define TURBO 600

//************** 센서 주소 *************//
enum Sensor{D1=0x01, D2=0x02, P1=0x04, P2=0x08, P3=0x10};

//************* 거리 상수 *************//
#define BREAK 200

//************* 색상 상수 *************//
enum Color{ RED = 1, BLUE, YELLOW, GREEN };

//************* 조건 상수 *************//
enum CondType { DELETED, PSD, PSDRUN, OPT, IND, DISTANCE, CAMERA };

//*********** BOOLEAN상수 ************//
enum Bool { false, true };

//******** 색상함수 반환모드 *********//
enum ColorReturn { MIN, MAX }; 

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 상수 정의 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 전역 변수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//********* 모션중 조건감시 반환구조체 **********//
typedef struct ConditionType {
	int type;
	int sensor;
	int value;
	int velocity;
	int ignore;
} Cond;

Cond Conds[10];

//**************** 엔코더 데이터 *****************//
volatile long dX,dY;
volatile double dER, dR, dA;
volatile double aER;
volatile double gER,gR,gA;

//*************** 현재 속도 변수 *****************//
volatile double aFx, aFy, aFw;

char lcdLine[21],mi;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 전역 변수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 수학 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

long labs(long val) { return ( val >= 0 ) ? val : -val; } // 절대값 반환

double frad(double deg) { return deg * Pi / 180.0; } // 라디안 반환 [ 각도 입력 ]

double deg(double rad) { return rad * 180.0 / Pi; } // 각도 반환 [ 라디안 입력 ]

double u_angle(double agl) { return ( ((long)agl + 720) % 360 ); } // 0~359 범위 반환

double h_angle(double agl) {  // -180~180 범위 반환
	agl = u_angle(agl);
	if( agl > 180 ) agl -= 360 ;
	return agl;
}

int amax(long arr[]) {	// 배열 원소 중 절대값이 가장 큰 원소의 인덱스 반환
	int i = 0;	
	if( labs(arr[1]) > labs(arr[i]) ) i = 1;
	if( labs(arr[2]) > labs(arr[i]) ) i = 2;
	return i;
} 
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 수학 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 로봇 인터페이스 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void BZ(int c) { // 부저 울리기 (count: 울림 횟수)
	int i;

	for( i = 0; i < c; i++ ) {
		LED_ON(3); _delay_ms(50);
		LED_OFF(3); _delay_ms(50);
	}
} // end of BZ()


void display_line(int line, char *str) { // LCD 에 한 줄 출력 (line: 출력할 줄 번호)
	int p = 0;
	while(str[++p]!='\0');
	while(p < 20) str[p++]=' ';

	str[20]='\0';
	lcd_display_str(line,0,str);
} // end of display_line()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 로봇 인터페이스 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 모터 엔코더 정보 분석 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void readPosition() {	// 현재 위치 계산
	char axis[30];
	long i, En[3];
	double ex, ey;

	// 모터 0번부터 2번까지의 주행거리 읽기
	for ( i=0; i<3; i++) {			
		WriteCommand(i, RDRP);
		En[i] = 0;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i));
	}

	// 회전각을 엔코더 단위로 구하기
	dER = (En[0] + En[1] + En[2]) / (2.985*WHEEL_R); // [ ( 1_12 ) Modify 2.985 -> 2.785 ]
	ex = (En[0] - En[2]) / sqrt(3);
	ey = (En[0] - 2*En[1] + En[2]) / 3.0;

	dR = dER / MM_ENCODER;	dA = deg(dR);
	dX = ex / MM_ENCODER;	dY = ey / MM_ENCODER;

	gER = aER + dER;
	gR = gER / MM_ENCODER;	gA = u_angle(deg(gR));
	
	// LCD 4째줄에 글로벌(절대) 좌표 표시
	sprintf(axis, "[%4ld]", (long) gA);	lcd_display_str(3,14,axis);
} // end of readPosition()


void setGA(long ga) {		// 글로벌 각도 보정
	readPosition();
	aER = ga * Pi * MM_ENCODER / 180.0 - dER;
	readPosition();
} // end of setGA

void setHome() {	// 현재 포지션에서 엔코더 초기화

	// 상대 좌표값 초기화
	readPosition();		
	
	aER += dER;		
	dER=0;

	// 엔코더 초기화
	WriteCommand(0, DFH);
	WriteCommand(1, DFH);
	WriteCommand(2, DFH);

} // end of setHome()

unsigned int ReadSignal(int motor) {	// 모터 신호 읽기(포지션 이동시 목표지점 도달여부를 확인하기 위해 사용)
	unsigned int sig = 0;
	WriteCommand(motor, RDSIGS);
	sig = (sig|ReadData(motor)) << 8;
	sig = (sig|ReadData(motor));
	return sig;
} // end of ReadSignal()
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 모터 엔코더 정보 분석 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 조건 감시 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
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

long chkCond() {	// 속도모드 주행 중 조건리스트 검사 후 속도값 반환
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

	if ( rtn > 0 ) { // 조건에 맞을 때
		//if ( Conds[rtn].velocity > 0 ) 	Conds[rtn].type = DELETED;  		// 현재 조건만 제거
		if ( Conds[rtn].velocity <= 0 ) for (i=1; i<10; i++) Conds[i].type = DELETED; 					// 모든 조건 제거
	}	

	return rtn;

} //end of chdCond()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 조건 감시 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSITIOION 모드 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void movePosition(long dx, long dy, long dw, int maxV, int acc) {	// POSITION 제어 기초함수
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

void farcP( long agl, long gap, long arc,long velocity, long acc ) { 	///// 원하는 각도로 아크 운동
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


void driveP(long agl, long distance){  	///// 원하는 방향과 원하는 거리만큼 직선운동
	long dx, dy;
	double rad;
	
	rad = frad(u_angle(agl));

	dx = distance * cos(rad);
	dy = distance * sin(rad);

	movePosition( dx, dy, 0 , RUN, HIGH_ACC);
	while( !(ReadSignal(0) & ReadSignal(1) & ReadSignal(2) & 0x0400) );

} // end of nhP();

void rotateP( long degree ){  			///// 원하는 각도로 제자리 회전(음수값일 경우 시계반대방향)
	return farcP(0, -ROBOT_R, degree,RUN,HIGH_ACC);
} //end of  rotateP()

void corAngle(long ga) {		///// 글로벌 각도를 이용하여 보정
	rotateP( h_angle(ga-gA) );
} // end of corAngle()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< POSITION 모드 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> VELOCITY 모드 기본 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
void moveByVel(double fx, double fy, double fw) {			////// 일정한 속도로 운동 (fx, fy, fw 자료형이 double형임을 주의)
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


void moveByAcc(long fx, long fy, long fw, long acc) {			////// 가속도로 운동(aFx, aFy, aFw에 현재 속도를 저장함)
	long Gap[3], king;
	double step;

	// 원하는 속도와 현재속도의 차이 (전역변수 aFx, aFy, aFw 는 직전에 모터에 지정한 속도값)
	Gap[0] = fx - aFx;
	Gap[1] = fy - aFy;
	Gap[2] = fw - aFw;

	// 차이가 가장 큰 모터의 번호
	king = amax(Gap);

	// 1ms동안 증가할 속도값 (acc는 1초동안 몇mm/s 증가시킬 것인지를 지정함)
	step = acc / 1000.0;

	// 만약 회전속도값의 차이가 가장 크다면 증가값은 2.2로 나눔(회전과 직선의 속도차이 때문)
	if ( king == 2 ) step /= 2.2;

	if ( curTime > prevTime ) {	 
		// 이 함수가 몇ms만에 호출되었는지 검사하여 그에 맞는 가속도로 현재속도를 변화시킴
		if ( (curTime - prevTime) <= 200 ) step *= (curTime-prevTime);
		else step *= 200;

		if ( step > labs(Gap[king]) ) { // 변화시킬 속도값이 원하는 변화량 보다 크다면 주어진 속도만큼 바로 변화시킴
			aFx = fx; aFy = fy; aFw = fw;
		} else {						// 각 방향의 비율차이만큼 가감속하여 속도값 지정
			aFx += step * Gap[0] / labs(Gap[king]);
			aFy += step * Gap[1] / labs(Gap[king]);
			aFw += step * Gap[2] / labs(Gap[king]);
		}
		// 지정한 속도로 운동
		moveByVel(aFx, aFy, aFw);
		
		prevTime = curTime; // 현재 시간을 저장
	}

} // end of moveByAcc()

void moveByAngle(double agl, long velocity, long fw, long acc) {	//////  각도로 방향 지정하여 가속도 운동(agl 자료형이 double형임을 유의)
	long fx, fy;
	agl = u_angle(agl);
	fx = velocity * cos(frad(agl));
	fy = velocity * sin(frad(agl));
	moveByAcc(fx, fy, fw, acc);
} // end of moveByAngle()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VELOCITY 모드 기본 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//




//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 매크로 정의 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
#define CHK_STOP(X) { 	if( (X) > 0 ) { \
							if( Conds[X].velocity > 0 ) { velocity = Conds[X].velocity; } \
							else { \
								if(Conds[X].velocity == 0 ) { StopMotion(9); aFx = aFy = aFw = 0; } \
								break; \
							} \
						} \
					}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 매크로 정의 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> VELOCITY 모드 활용 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

long driveV(long agl, long velocity, long acc) { // 지정된 각도로 주행
	long rtn;

	while(1) {
		moveByAngle(agl, velocity, 0, acc);
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
} // end of driveV();

long arcV(long agl, long gap, long arc, long velocity ) {  // 홀로노믹 아크 운동
	long rtn = 0, dtr = 1,radian = 0;
	double start = 0, time = 0;
	double rotated = 0;

   setHome();

   radian = gap + ROBOT_R; // 원의 반지름

   if( arc < 0 ) { dtr = -1; arc *= -1; } // 방향설정
   
   start = curTime; // 시작 시간 설정

   while(labs(arc) >= labs(rotated)) { // 지정한 각도 >= 현재 돌고있는 각도 조건 체크
      time = ( curTime - start ) / 1000.0;
 
      rotated = ( velocity  / ( 2 * Pi * radian ) ) * C_AGL_F; // 아크 현재 각도

      moveByAngle( ( rotated + ( agl - 90 ) ) * dtr , velocity , 0, HIGH_ACC);
      rtn = chkCond();
      CHK_STOP(rtn);
   }

   aFx = aFy = aFw = 0; // 가속도 수동 초기화
   StopMotion(9);

   readPosition();

   return rtn;
} // end of farcV()

long farcV(long agl, long gap, long arc, long velocity,long acc) {  // 아크 운동
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

long tarcV(long agl, long gap, long arc, long  turn, long velocity) {  // 아크 + 터닝 운동
	long rtn = 0, dtr = 1;
	long dist = 0, fw = 0, radian = 0;
	double start = 0, time = 0;
	double turned = 0, rotated = 0;

	if( arc < 0 ) { dtr = -1; arc *= -1; }

	setHome();

	radian = gap + ROBOT_R; // 원의 반지름
		
	dist = ( ( 2 * Pi * radian ) / C_AGL_F ) * arc; // 원호의 길이

	fw = turn * velocity / dist; // 초당 각도 

	start = curTime;

	while( labs(arc) >= labs(rotated) ) { // 지정한 각도 >= 현재 돌고있는 각도 조건 체크
		time = ( curTime - start ) / 1000.0;

		turned = time * fw; // 터닝 각도
		rotated =  ( velocity * time ) / ( 2 * Pi * radian ) * C_AGL_F ; // 아크 현재 각도


		moveByAngle( ( ( rotated - turned ) + ( agl - 90 ) ) * dtr, velocity, fw, NO_ACC); // ( 진행방향 - 시작위치부터 터닝한 각도 ) =  진행 방향 
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	aFx = aFy = aFw = 0; // 가속도 수동 초기화
	StopMotion(9);

	readPosition();

	return rtn;
} // end of tarcV()

long rotateV(long degree, int speed) { // 제자리 회전
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


long turningV( long agl, long dist, long  turn , long velocity ) { // ( 보류 )
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

	aFx = aFy = aFw = 0; // 가속도 수동 초기화

	StopMotion(9);

  	readPosition();	

	return 0;

} // end of turning()

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VELOCITY 모드 활용 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 라인 및 벽 활용 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 라인 및 벽 활용 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> COLOR 활용 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< COLOR 활용 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 보류 함수 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

/*//////////////// 코드 수정,추가 및 보완 해야할점이 많음 ////////////////////////
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
long ucV(long agl, long gap, long arc, long velocity ) {  // 홀로노믹 아크 운동
   long rtn = 0, dtr = 1,radian = 0;
   double prevTime = 0, time = 0;
   double rotated = 0;

   setHome();

   radian = gap + ROBOT_R; // 원의 반지름

   if( arc < 0 ) { dtr = -1; arc *= -1; } // 방향설정
   
   prevTime = curTime; // 시작 시간 설정

   while(labs(arc) >= labs(rotated)) { // 지정한 각도 >= 현재 돌고있는 각도 조건 체크
      time = ( curTime - prevTime ) / 1000.0;
      prevTime = curTime; 
      rotated += ( sqrt(aFx*aFx+aFy*aFy) * time ) / ( 2 * Pi * radian ) * C_AGL_F; // 아크 현재 각도

      moveByAngle( ( rotated + ( agl - 90 ) ) * dtr , velocity , 0, NO_ACC);
      rtn = chkCond();
      CHK_STOP(rtn);

   }

   aFx = aFy = aFw = 0; // 가속도 수동 초기화
   StopMotion(9);

   readPosition();

   return rtn;
} // end of ucV()

*/


/*
long side( long agl, long psd, long gap, long velocity, long acc ) { // ( 보류 )
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


// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 보류 함수 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< //
