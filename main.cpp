#include "mbed.h"
#include "BNO055.h"
#include "Ping.h"
#include "Motor.h"
#include "Ballsensor.h"
#include "Linesensor.h"

//TerminalDisplay(TeraTerm)
Serial pc(SERIAL_TX,SERIAL_RX);

//UltraSonicSensor
Ping uss_left(PA_0);
Ping uss_right(PB_13);

//BallSensor
Ballsensor ball(A3,A2);

//LineSensor
Linesensor line(A1);

//Motor
Motor motor(PA_10,PB_3,PB_5,PB_4,PB_10,PA_8);

//Timer
Timer time1;

//GyroSensor
I2C i2c(D14,D15);
BNO055 imu(i2c,PC_8);
BNO055_ID_INF_TypeDef bno055_id_inf;
BNO055_EULER_TypeDef  euler_angles;

//HoldCheckSensor
DigitalIn hold_check(PC_3);

//ToggleSwitch
DigitalIn sw_start(PD_2);//program start switch
DigitalIn sw_reset(PC_11);//gyro sensor reset switch
DigitalIn swdebug(PC_10);//reserve switch(non connect)
DigitalIn swkick(PC_12);//reserve switch(non connect)

//declear prototype (function list)
int PID(float kp,float ki,float kd,int target,int degree);
int mawari(int kaku,int kyori);
/*****************************************************************/
/**********************main function******************************/
/*****************************************************************/

int main(){
	int ja, hou, kyori, byou, kaku, umu, kakudo;
//**************************************************************//
////////////////////////initialize setting////////////////////////
//**************************************************************//
    /*ultra sonic sensor set speed*/
    uss_right.Set_Speed_of_Sound(32);//(cm/ms)
    uss_left.Set_Speed_of_Sound(32);//(cm/ms)
    
    /*motor pwm frequency set*/
    motor.setPwmPeriod(0.03);

    /*change Mode IMU,COMPASS,M4G,NDOF_FMC_OFF,NDOF*/
    imu.reset();
    imu.change_fusion_mode(MODE_IMU);
    wait_ms(100);
    imu.get_Euler_Angles(&euler_angles);
    int init_degree = euler_angles.h;
    imu.reset();
    motor.omniWheels(0,0,0);
    int a;
    while(1){
//***************************************************************//
////////////////Play mode(you can write this statement)////////////
//***************************************************************//
        while(sw_start == 1){
            imu.get_Euler_Angles(&euler_angles);
            a = PID(0.4,0.25,0.025,0,euler_angles.h);
            kakudo = line.direction();
            if (kakudo != -999){

            	if (kakudo == 999)
            	{
            	motor.omniWheels(0,0,0);
            	wait(0.1);
            	}
            	else if (kakudo <= 179)
            	{
						kakudo = kakudo + 180;
            	}
            	else if (kakudo >= 180)
            	{
            			kakudo = kakudo - 180;
            	}
            	motor.omniWheels(kakudo,50,0);
            }
            else{
            byou = 1;
            kaku = ball.degree();
            kyori = ball.distance();
            umu = hold_check.read();
            if(umu == 1){
            	hou = mawari(kaku,kyori);
            }else{
                hou = 0;
            }
            if(kyori >= 900){
            	motor.omniWheels(0,0,a);
            }
            else{
            	motor.omniWheels(hou,50,a);
            }
            }
        }
        
        
        
//***************************************************************//
////////////////////////Gyro reset mode////////////////////////////
//***************************************************************//
        if(sw_reset == 1){
            imu.reset();
            wait_ms(100);
            imu.get_Euler_Angles(&euler_angles);
            init_degree = euler_angles.h;
        }
        motor.omniWheels(0,0,0);
        
        
        
//***************************************************************//
//////////////////////////debug mode///////////////////////////////
//***************************************************************//
        if(pc.readable() == 1){
            if(pc.getc() == 'd'){
                //if 'd' is pressed,debug mode will start.
                while(1){
                    if(pc.readable() == 1){
                        if(pc.getc() == 'r'){//if 'r' is pressed,debug mode will be end.
                            break;
                        }
                    }
                    imu.get_Euler_Angles(&euler_angles);
                    pc.printf("Gyro   degree: %d \r\n",(int)euler_angles.h);
                    pc.printf("Ball   degree: %d \r\n",ball.degree());
                    pc.printf("Ball distance: %d \r\n",ball.distance());
                    pc.printf("Line   sensor: %d \r\n",line.direction());
                    pc.printf("Hold   sensor: %d \r\n",hold_check.read());
                    uss_left.Send();
                    uss_right.Send();
                    wait_ms(40);
                    pc.printf("USS      left: %d cm\r\n",uss_left.Read_cm());
                    pc.printf("USS     right: %d cm\r\n",uss_right.Read_cm());
                
                    wait_ms(40);
                    pc.printf("\f");
                }
            }
        }
        
        
        
        
/**********************end main function**************************/
    }
}



int PID(float kp,float ki,float kd,int target,int degree)
{
	static float P = 0, OP = 0, I = 0, D = 0, re = 0;
	if (degree >= 180){
		degree = degree - 360;
	}

	P = degree - target;
	D = (P - OP) / 0.001;
	I += P * 0.001;
	OP = P;
	re = -1 * (kp * P + ki * I + kd * D);

	if (-3 <= degree <= 3 && degree == target){
		I = 0;
	}
	if (re >= 50){
		re = 50;
	}else if (re <= -50){
		re = -50;
	}

	return re;
}

int mawari(int kaku,int kyori){
	int hou;
	if(kyori <= 500){
	 if((-90 <= kaku) && (kaku <= 90)) {
	        	  hou = 2 * kaku;
	           }else if(kaku <= -180){
	        	  hou = kaku - 90;
	           }else{
	              hou = kaku + 90;
	           }
	}else{
	              hou = kaku;
                                        }
	 return hou;
}
