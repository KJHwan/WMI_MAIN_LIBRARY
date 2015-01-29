
//┌──────────────────────────────────────────────────────────────┐
//
//	@ 파일명 : wmilib.c
//	@ 설명 : 기능경기대회 모바일로보틱스 공식로봇(2013~) 구동을 위한 구동함수
//	@ 개발자 : 원주의료고등학교 교사 강상성(carpedm.kr@daum.net)
//
//└──────────────────────────────────────────────────────────────┘


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 상수 정의 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 숫자 상수 --->	//
#define Pi 3.141592
#define WORD_SIZE_I 65536
#define WORD_SIZE_F 65536.0
#define C_AGL_F 360.0

//	<!--- 로봇 특성 --->	//
#define WHEEL_R 125.0
#define ROBOT_R 176
#define PSD_AGL 40

//	<!--- 모터 특성 --->	//
#define MM_ENCODER 191.0
#define MAX_MV ( 39 * WORD_SIZE_I )

//	<!--- 가속 상수 --->	//
#define NO_ACC 30000
#define LOW_ACC 100
#define HIGH_ACC 200

//	<!--- 속도 상수 --->	//
#define CRAWL 70
#define WALK 250
#define RUN 450

//	<!--- 색상 상수 --->	//
enum Color { RED = 1, BLUE, YELLOW, GREEN };

//	<!--- 센서 주소 --->	//
enum Sensor { D1 = 0x01, D2 = 0x02, P1 = 0x04, P2 = 0x08, P3 = 0x10};

//	<!--- 조건 상수 --->	//
enum CondType { DELETED, PSD, PSDRUN, OPT, IND, DISTANCE };

//	<!--- Bool 상수 --->	//
enum Bool { false, true };

//  <!--- 방향 상수 --->	//
enum Direction { FORWARD, BACK, RIGHT, LEFT };

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 상수 정의 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//


//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 전역 변수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 조건 감시 구조체 --->	//
//	사용 영역 : chkCond(), addCond(), 사용자
typedef struct ConditionType {
	long type;
	long sensor;
	long value;
	long velocity;
	long ignore;
} Cond;

//	<!--- 조건  감시  배열 --->	//
Cond Conds[10];

//	<!--- 엔코더 데이터 < 플랫폼 기준 > --->	//
volatile long dX,dY;
volatile double dER, dR, dA;
volatile double aER;

//	<!--- 엔코더 데이터 < 글로벌 기준 > --->	//
volatile double gER,gR,gA;

//	<!--- 현재 속도 변수 < 가속도 >  --->	 //
volatile double aFx, aFy, aFw;

//	<!--- LCD 버퍼 --->
char lcdLine[21];

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 전역 변수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//


//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 수학 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	*******  수학 함수는 거의 모든 함수에 쓰인다.	*******//

//	<!--- 절대값 반환 --->	//
long labs(long val) {
	return ( val >= 0 ) ? val : -val; 
}

//	<!--- 라디안 반환 --->	//
//	[ 각도 ] -> [ 라디안 ]
double rad(double deg) {
	return deg * Pi / 180.0; 
} 

//	<!---  각도  반환 --->	//
//	[ 라디안 ] -> [ 각도 ]
double deg(double rad) {
	return rad * 180.0 / Pi; 
}

//	<!--- 0 ~ 359 범위 반환 --->
double u_angle(double agl) {
	return ( ((long)agl + 720) % 360 ); 
}

//	<!--- (-180) ~ 180 범위 반환 --->	//
double h_angle(double agl) {
	agl = u_angle(agl);
	if( agl > 180 ) agl -= 360 ;
	return agl;
}

//	<!--- 제일 큰 값의 인덱스 반환 --->	//
//	배열[3] 원소 중 절대값이 가장 큰 원소의 인덱스 반환
long amax(long arr[]) { 
	int i = 0;	
	if( labs(arr[1]) > labs(arr[i]) ) i = 1;
	if( labs(arr[2]) > labs(arr[i]) ) i = 2;
	return i;
} 
//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 수학 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//


//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 인터페이스 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 부저 < count: 울림 횟수 > --->	//
void BZ(int count ) { 
	int i;

	for( i = 0; i < count ; i++ ) {
		LED_ON(3); _delay_ms(50);
		LED_OFF(3); _delay_ms(50);
	}
}

//	<!--- LCD 출력	--->
void display_line(int line, char *str) {
	int p = 0;
	while( str[++p] != '\0' );
	while( p < 20 ) str[p++]=' ';

	str[20]='\0';
	lcd_display_str(line,0,str);
}

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 인터페이스 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//



//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 엔코더 데이터 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 플랫폼 위치 읽기 --->	//
//	사용 전역 변수 : 엔코더 데이터 변수.
//	사용 영역 : chkCond(),VELOCITY 모드의 모든 함수.
//	플랫폼의 엔코더 값을 읽어와 사용자가 사용 할 수 있게 가공해주는 함수.
//	**	매우 중요한 함수 **
void readPosition() {
	long i, En[3];
	double ex, ey;

	//	모터 0번부터 2번까지의 주행거리 읽기.
	for ( i=0; i<3; i++) {			
		WriteCommand(i, RDRP);
		En[i] = 0;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i))<<8;
		En[i] = (En[i]|ReadData(i));
	}

	//	엔코더 단위로 구하기
	dER = ( En[0] + En[1] + En[2] ) / (2.985*WHEEL_R);
	ex = ( En[0] - En[2] ) / sqrt(3);
	ey = ( En[0] - ( 2 * En[1] + En[2] ) ) / 3.0;

	//	플랫폼 기준 단위 변환	< 엔코더 단위 -> 거리 or 각도 단위 >
	dR = dER / MM_ENCODER;	dA = deg(dR);
	dX = ex / MM_ENCODER;	dY = ey / MM_ENCODER;

	//	글로벌 기준 단위 변환	< 엔코더 단위 -> 거리 or 각도 단위 >
	gER = aER + dER;
	gR = gER / MM_ENCODER;	gA = u_angle(deg(gR));
	
}

//	<!--- 글로벌 각도 설정 --->	//
//	사용 전역 변수 : gER, dER
//	입력한 각도를 엔코더 값으로 전환해 글로벌 각도를 재설정 해준다.
void setGA(long ga) {
	//	위치 데이터를 수집.		
	readPosition();
	//	글로벌 각도를 재 설정.
	aER = ga * Pi * MM_ENCODER / 180.0 - dER;
	// 재 설정부터 데이터를 다시 수집.
	readPosition();
}

//	<!--- 현재 위치 기준 엔코더 초기화 --->	//
//	사용 전역 변수 : gER, dER
//	사용 영역 :	VELORCITY 모드의 함수들.
//	이 함수가 사용되지 않았을때 대표적인 에러 : 
//			- 한 모션 함수가 구동되고 그 다음 모션 함수들이 바로 끝나는 경우가 발생한다. [ 거리 or 각도로 구동하는 함수들일 경우 ]
//	** 중요한 함수 **
void setHome() {

	// 위치 데이터를 수집
	readPosition();		
	
	// 엔코더 변수 초기화
	aER += dER;		
	dER=0;

	// 엔코더 초기화
	WriteCommand(0, DFH);
	WriteCommand(1, DFH);
	WriteCommand(2, DFH);

}

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 엔코더 데이터 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//

//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 조건 감시 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 조건 추가 --->	//
//	사용 전역 변수 : Conds[10]
//	사용 영역 : 사용자, chkCond(), PD,PT,ND,TG 매크로.
//	조건을 추가하는 함수이며 사용용도에 따라 다양한 조건을 구성할수있다.
//	여러 조건을 동시에 걸수있다.
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

//	<!--- 조건 리스트 체크 --->	//
//	사용 전역 변수 : Conds[10]
//	사용 영역 :	VELOCITY 모든 함수들.
//	조건을 걸었는대 무시하거나 원하는대로 작동하지 않았을경우 이함수를 확인해 볼것.	** 대부분 여기에 문제가 있다 그리고 무시거리도 꼭 확인해 볼것 **
//	** 중요한 함수 **
long chkCond() {	
	long rtn = 0;
	long i, dist, err;

	//	위치 데이터를 수집하고 그 데이터를 바탕으로 이동거리 계산.
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

	//	조건이 끝났지만 [ 속도가 0 이하 ]로 설정 되지 않을경우 [ 조건 삭제 ]를 실행하지 않는다.
	if ( rtn > 0 && Conds[rtn].velocity <= 0 ) {
		for (i=1; i<10; i++) Conds[i].type = DELETED; 
	}

	return rtn;
}

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 조건 감시 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//

//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 속도 모드 기본 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 메인 모션 구동 --->	//
//	사용 영역 : moveByAcc().
//	플랫폼의 움직임을 대부분 담당하는 함수.
//	로봇의 움직임이 규칙적이나 축이 바뀐듯하거나 바퀴가 각각 따로 움직인다고 생각되면 이 함수를 확인 해야된다.
//	** 중요한 함수 **
void moveByVel(double fx, double fy, double fw) {
	int king;
	long V[3]={0.0,0};

	if( fx == 0 && fy == 0 && fw == 0) { StopMotion(9); return; }

	//	사용자가 원하는 속도를 계산.
	V[0] = ((( 0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[1] = (((-0.065*fy)+(0.14*fw))) * WORD_SIZE_F;
	V[2] = (((-0.056*fx)+(0.033*fy)+(0.14*fw))) * WORD_SIZE_F;

	king = amax(V);

	// 제일 빠른 모터를 기준으로 속도 재설정.
	if ( V[king] > MAX_MV ) {
		V[0] = MAX_MV * (V[0] / (double) labs(V[king]));  
		V[1] = MAX_MV * (V[1] / (double) labs(V[king])); 
		V[2] = MAX_MV * (V[2] / (double) labs(V[king])); 
	}
	
	//	모션 구동.
	SetVelocity(0, V[0]);
	SetVelocity(1, V[1]);
	SetVelocity(2, V[2]);

	StartMotion();
}

//	<!--- 메인 가속도 구동 --->	//
//	사용 전역 변수 : aFx, aFy, aFw
//	사용 영역 : moveByAngle()
//	타이머 사용 함수. < curTime 사용 >
//	메인 모션 구동함수 [ moveByVel() ]의 문제가 없다고 판단된 이후 로봇이 그래도 움직이지 않을 경우에는 이 함수를 확인해야 된다.
//	특히 타이머 < curTime > 이 작동하는지 확인 해보는 것이 첫번째로 중요하다. ** step 에 직접적인 영향 **
//	**	중요한 함수 **
void moveByAcc(long fx, long fy, long fw, long acc) {
	long Gap[3], king;
	double step;

	//	원하는 속도와 현재속도의 차이
	Gap[0] = fx - aFx;
	Gap[1] = fy - aFy;
	Gap[2] = fw - aFw;

	//	차이가 가장 큰 모터의 번호
	king = amax(Gap);

	//	[ 1 ms ] 동안 증가할 속도값 ( acc는 1초동안 몇 [mm/s] 증가시킬 것인지를 지정함 )
	step = acc / 1000.0;

	//	만약 직선속도와 회전속도 값의 차이가 가장 크다면 증가값은 [ 2.2 ]로 나눔.	** 회전과 직선의 속도차이 때문이다. **
	if ( king == 2 ) step /= 2.2;

	if ( curTime > prevTime ) {

		// 이 함수가 몇 [ms]만에 호출되었는지 검사하여 그에 맞는 가속도로 현재속도를 변화시킴
		if ( ( curTime - prevTime ) <= 200 ) step *= ( curTime - prevTime );
		else step *= 200;

		// 현재속도가 원하는속도 보다 커질 경우 원하는 속도를 유지.
		if ( step > labs(Gap[king]) ) { 
			aFx = fx; 
			aFy = fy; 
			aFw = fw;

		} 
		// 각 방향의 [ 비율차이 ] 만큼 가감속하여 속도값 재설정.
		else {						
			aFx += step * Gap[0] / labs(Gap[king]);
			aFy += step * Gap[1] / labs(Gap[king]);
			aFw += step * Gap[2] / labs(Gap[king]);
		}

		//	가속도 값으로 메인모션 구동.
		moveByVel(aFx, aFy, aFw);
		
		//	현재 시간을 저장
		prevTime = curTime; 
	}

}

//	<!--- 사용자 사용 함수 --->	//
//	사용 영역 : VELOCITY 모드의 함수들.
//  각도로 방향 지정하여 가속도 운동(agl 자료형이 double형임을 유의)
//	사용자가 사용하기 제일 적합한 함수.	** 계산할 내용이 별로 없다. **
void moveByAngle(double agl, long velocity, long fw, long acc) {
	long fx, fy;

	agl = u_angle(agl);
	
	fx = velocity * cos(rad(agl));
	fy = velocity * sin(rad(agl));
	
	moveByAcc(fx, fy, fw, acc);
}

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 속도 모드 기본 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//




//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 매크로 정의 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 조건 스탑 매크로 --->	//
//	속도 유지와 정지가 이 매크로에서 정해진다.
//	** 가속도 초기화에 주의 할것 **
#define CHK_STOP(X) { 	if( (X) > 0 ) { \
							if( Conds[X].velocity > 0 ) { velocity = Conds[X].velocity; } \
							else { \
								if(Conds[X].velocity == 0 ) { StopMotion(9); aFx = aFy = aFw = 0; } \
								break; \
							} \
						} \
					}

//	<!--- 조건 매크로 --->	//
#define PD( psd, gap, ig , vel )		addCond(PSD,psd,gap,vel,ig );
#define PT( sensor , ig )				addCond(OPT,sensor,sensor,0,ig );
#define ND( sensor , ig ) 				addCond(IND,sensor,sensor,0,ig );
#define TG( dist, vel ) 				addCond(DISTANCE,0,dist,vel,0);

//	<!--- 모션 종료 매크로 --->	//
#define MOTION_STOP() StopMotion(9); readPosition();

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 매크로 정의 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//


//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 속도 모드 활용 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 직선 주행 --->	//
//	** 조건을 걸어 주지 않을경우 멈추지 않는다. **
//	가장 기본적인 구동.
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

//	<!--- 제자리 회전 --->
//	speed는 초당 회전 속도다 일반속도 처럼 생각하지 말것.
//	가장 기본적인 구동.
long rotateV(long degree, int speed) { // 제자리 회전
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


//	<!--- 아크 주행 --->	//
//	사용 전역 변수 : aFx, aFy, aFw
//	홀로노믹 주행
//	타이머 사용 함수. < curTime 사용 >
long arcV(long agl, long gap, long arc, long velocity ) {  
	long rtn = 0;
	long drt = 1;
	long radius;
	double rotated = 0;
	double start, time;


	setHome();

	if( arc < 0 ) { drt = -1; arc *= -1; } 

	//	아크 주행할 원의 반지름 < 플랫폼 테두리부터 >	 **원의 중심과 헷갈리지 말것 !!**
	radius = gap + ROBOT_R; 
	
	start = curTime;

	//	지정한 각도보다 지금까지 회전한 각도가 큰지 체크
	while( labs(arc) >= labs(rotated) ) { 
		time = ( curTime - start ) / 1000.0;

		rotated = ( velocity * time  / ( 2 * Pi * radius ) ) * C_AGL_F; 

		moveByAngle( ( rotated + ( agl - 90 ) ) * drt , velocity , 0, NO_ACC);
		
		rtn = chkCond();
		CHK_STOP(rtn);
	}

	//	가속도 초기화	** 빼먹으면 다음 모션에 문제가 생김 !! **
	aFx = aFy = aFw = 0; 

	MOTION_STOP();

	return rtn;
}

//	<!--- 아크 주행 --->	//
//	논 홀로노믹 주행
//	가속도가 사용 가능한 유일한 아크 함수다.
long farcV(long agl, long gap, long arc, long velocity,long acc) {
	long rtn;
	long drt = 1;
	long dist, radius;
	long fx, fy, fw;
	double radian;

	if( arc < 0 ) { drt = -1; arc *= -1; }

	//	아크 주행할 원의 반지름 < 플랫폼 테두리부터 >	 **원의 중심과 헷갈리지 말것 !!**
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


//	<!--- 아크 주행 --->	//
//	사용 전역 변수 : aFx, aFy, aFw
//	타이머 함수 < curTime >
//	[ turnV ]함수와 [ arcV ]함수를 합친 함수
//	반경이 매우 커지거나 매우 작을경우 정확도가 떨어짐.
//	**	모터 최고속도보다 더 빠른속도를 요구하는 값은 설정하지 않도록 주의	**
long tarcV(long agl, long gap, long arc, long  turn, long velocity) {  // 아크 + 터닝 운동
	long rtn = 0;
	long drt = 1;
	long dist = 0, fw = 0, radius = 0;
	double start = 0, time = 0;
	double turned = 0, rotated = 0;

	if( arc < 0 ) { drt = -1; arc *= -1; }

	setHome();

	//	아크 주행할 원의 반지름 < 플랫폼 테두리부터 >	 **원의 중심과 헷갈리지 말것 !!**
	radius = gap + ROBOT_R;
		
	//	플랫폼이 출발위치부터 원하는 각도까지의 직선 길이[ 원의 호 ]를 계산.
	dist = ( ( 2 * Pi * radius ) / C_AGL_F ) * arc;

	//	계산한 직선거리[ 원의 호]를 바탕으로 초당 회전속도를 계산.
	fw = turn * velocity / dist; 

	start = curTime;

	//	지정한 각도보다 지금까지 회전한 각도가 큰지 체크
	while( labs(arc) >= labs(rotated) ) {
		time = ( curTime - start ) / 1000.0;

		turned = time * fw;
		rotated =  ( velocity * time ) / ( 2 * Pi * radius ) * C_AGL_F ;

		// ( 아크 진행방향 - 시작위치부터 터닝한 각도 ) =  진행 방향 
		moveByAngle( ( ( rotated - turned ) + ( agl - 90 ) ) * drt, velocity, fw, NO_ACC); 

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	//	가속도 초기화	** 빼먹으면 다음 모션에 문제가 생김 !! **
	aFx = aFy = aFw = 0;

	MOTION_STOP();

	return rtn;
}

//	<!--- 직선 회전 주행 --->
//	사용 전역 변수 : aFx, aFy, aFw
//	타이머 함수 < curTime >
//	**	모터 최고속도보다 더 빠른속도를 요구하는 값은 설정하지 않도록 주의	**
//	제대로 작동하지 않을경우 타이머를 제일 첫번째로 확인해볼것.
long turnV( long agl, long dist, long  turn , long velocity ) { // ( 보류 )
	long rtn;
	long fw; 
	double start, time;
	double turned = 0;

	setHome();

	//	설정된 직선거리를 바탕으로 초당 회전속도를 계산.
	fw = turn * velocity / dist;

	start = curTime;

	//	지정한 각도보다 지금까지 회전한 각도가 큰지 체크
	while( labs(turn) > labs(turned) ) {
		time = curTime - start/ 1000.0;
		turned = time * fw ;
		
		moveByAngle( agl - turned , velocity, fw, NO_ACC);
		
		rtn = chkCond();
     	CHK_STOP(rtn);
	}

	//	가속도 초기화	** 빼먹으면 다음 모션에 문제가 생김 !! **
	aFx = aFy = aFw = 0;

	MOTION_STOP();

	return DISTANCE;

} // end of turning()

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 속도 모드 활용 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//



//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 라인 및 벽 활용 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 벽 주행 --->
//	사용 전역 변수 : aFw
//	fw 를 사용하여 어떤 각이든 벽과 평행을 유지하며 가는 함수
//	벽을타며 각을 잡으며 장거리에 유리함.
//	단거리에는 오히려 각을 틀수잇음 **주의**
//	홀로노믹 주행
long hSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn;
	long wall, vel, between;

	setHome();

	while(1) {
		// 벽과의 적외선 센서 데이터를 사용할수있게 가공함.
		wall = psd_value[psd] - gap;
		vel = sqrt(aFx*aFx + aFy*aFy);

		// 사용하는 적외선 센서 번호로 데이터를 가공.
		between = ( 9 - psd ) * PSD_AGL - agl;

		// 그 데이터를 이용하여 방향을 설정
		if( u_angle(between) > 180 ) wall *= 1;
		else  wall *= -1;

		aFw = wall * ( 700  - vel ) / 800.0;		

		moveByAngle(agl,velocity,aFw,acc);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}

//	<!--- 벽 주행 --->
//	벽과 평행인 상태가 아니라도 벽을 탈수있으며 벽과 거리를 유지하며 간다.
//	벽을타며 각을잡지 않으므로 단거리에 유리함
//	0도 120도 240도를 자용하여 장거리에 사용가능 다른각도일경우 오차가 발생할수 있음.
//	논 홀로노믹 주행
long nhSide( long agl, long psd, long gap, long velocity, long acc ) {
	long rtn;
	long wall, go_agl;

	setHome();

	while(1) {
		// 벽과의 적외선 센서 데이터를 사용할수있게 가공함.
		wall = gap - psd_value[psd];
		go_agl = agl + wall/3;

		moveByAngle(go_agl,velocity,0,acc);

		rtn = chkCond();
		CHK_STOP(rtn);
	}

	return rtn;
}


//	<!--- 라인 주행 --->
//	논 홀로노믹 주행
long nhLine( long agl, long gap, int sensor, long velocity, int line ) {	// 보류
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

//	<!--- 라인 주행 --->
//	논 홀로노믹 주행
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

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 라인 및 벽 활용 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//

//▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 카메라 활용 함수 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼//

//	<!--- 설정한 색깔의 X 좌표 반환 --->
long getColX( int c ) {
	return Camera_Cmd(c,Color_X);
}

//	<!--- 설정한 색깔의 Y 좌표 반환 --->
long getColY( int c ) {
	return Camera_Cmd(c,Color_Y);
}

//	<!--- 설정한 색깔의 높이 반환 --->
long getColH( int c ) {
	return Camera_Cmd(c,Color_H);
}

//	<!--- 설정한 색깔의 넓이 반환 --->
long getColW( int c ) {
	return Camera_Cmd(c,Color_W);
}

//	<!--- 설정한 색깔의 갯수 반환 --->
long getColC( int c ) {
	return Camera_Cmd(c,Color_C);
}

//	<!--- X 좌표 이동 함수 --->
long ColorFB( int color , int posX ) {
	long errX = 100, colX = 100;

	setHome();

	//	좌표 미세 조정은 오른쪽 상수 값을 바꾸면서 할수있다.
	while(labs(errX) > 5) {
		colX = getColX(color);
		errX = colX - posX;

		// errX * 상수 [ 미세 조정 ]
		moveByAcc(errX*3, 0, 0 , LOW_ACC);
	}

	MOTION_STOP();

	return colX; 
}

//	<!--- Y 좌표 이동 함수 --->
//	모드에 0을 설정할 경우 좌우 운동으로 Y 좌표를 맞춘다.
//	모드에 1을 설정할 경우 회전 운동으로 Y 좌표를 맞춘다.
long ColorLR( int color , int posY , int mode ) {
	long errY = 100, colY = 100;

	setHome();

	//	좌표 미세 조정은 오른쪽 상수 값을 바꾸면서 할수있다.
	while(labs(errY) > 5) {		
		colY = getColY(color);
		errY = colY - posY;

		// errY * 상수 [ 미세 조정 ]
		if( mode == 0 )
			moveByAcc(0, errY*3, 0 , LOW_ACC);

		else
			moveByAcc(0, 0, errY*3, LOW_ACC);
	}

	MOTION_STOP();

	return colY; 
}

//▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 카메라 활용 함수 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲//
