/*
 Western Engineering MSE 2202 Library
 2023 E J Porter

 
  
 */



 #ifndef MSE2202_LIB_H
  #define MSE2202_LIB_H 1

#include <Arduino.h>
#include "esp32-hal.h"
#include "esp32-hal-ledc.h"

//************  Motion *********************************

#define LEDCMAXCHANNELS 8

//TaskHandle_t Core_Zero;

class Motion
{
public:
	    Motion();
	    ~Motion(){ end(); }
/*
  driveBegin with set up two motors as a drive base. DriveID = D1 or D2 ( only two Drives can be begun, this will limit number of servoBegin and motorBegin. Since there is only 8 channels total
 and driveBegin uses 4 channels each.
*/
	    void driveBegin(char cDriveID[2], int iLeftMotorPin1, int iLeftMotorPin2,int iRightMotorPin1, int iRightMotorPin2); 
/*
  Set up one motor to be controlled.  cMotorID = M1 to M4 ( limited number of motors and will be lessened if servos or drives are used. Since there is only 8 channels total
 and MotorBegin uses 2 channels each.
*/		
		void motorBegin(char cMotorID[2], int iMotorPin1, int iMotorPin2);
/*
  Set up one servo to be controlled.  cServoID = S1 to S8 ( limited number of servos and will be lessened if motors or drives are used. Since there is only 8 channels total
 and servoBegin uses 1 channel each.
*/			
		void servoBegin(char cServoID[2], int iServoPin1);
/*
  Will run the motor(s) "Forward" at speed in ucSpeed ( 0 to 255),Can be both Drive or Motor ID = M1 to M4 or D1, D2
*/			
		void Forward(char cID[2], unsigned char ucSpeed);
/*
  Will run the motor(s) "Forward" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Forward(char cID[2],unsigned char ucLeftSpeed, unsigned char ucRightSpeed );
/*
  Will run the motor(s) "Reverse" at speed in ucSpeed ( 0 to 255),Can be both Drive or Motor ID = M1 to M4 or D1, D2
*/		
		void Reverse(char cID[2],unsigned char ucSpeed);
/*
  Will run the motor(s) "Reverse" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Reverse(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed );
/*
  Will run the motor(s) "Left" (Right motor forward and  Left motor in reverse, zero point turn) at speed in ucSpeed ( 0 to 255), Only for Drive ID = D1, D2
*/		
		void Left(char cID[2],unsigned char ucSpeed);
/*
  Will run the motor(s) "Left" (Right motor forward at ucRightSpeed speed and  Left motor in reverse at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Left(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed );
/*
  Will run the motor(s) "Right" (Right motor reverse and  Left motor in forward, zero point turn) at speed in ucSpeed ( 0 to 255), Only for Drive ID = D1, D2
*/		
		void Right(char cID[2],unsigned char ucSpeed);
/*
  Will run the motor(s) "Right" (Right motor reverse at ucRightSpeed speed and  Left motor in forward at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Right(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed );
/*
  Will run the Stop motor(s) 55), Can be Drive,  Motor or Servo ID = M1 to M4 or D1, D2
*/		
		void Stop(char cID[2]);
/*
  Will run the Servo(s) to the raw position in ucServoPosition ( 0 to 16384), ID can only be servo ID = S1 to S8
*/			
		void ToPosition(char cID[2], unsigned int uiServoPosition);
	
		void end();

	  
private:
        unsigned char ucLEDcLastUnUsedChannel;	
		unsigned char ucLEDcChannelError;
		unsigned char ucLEDcDriveChannels[8];
		unsigned char ucLEDcMotorChannels[8];
		unsigned char ucLEDcServoChannels[8];
		unsigned char Get_LEDcChannel();



};

//------------------------------------------------------------------------------------------------------------------------------------------------


class Encoders
{
public:
	    Encoders();
	    ~Encoders(){ end(); }
/*
  Begin with set up both encoders: (ucEncoderType 0 - 2 will set the pin as per the MSE-Duino V4.2 board,ucEncoderType 3 - 5 pin data will need to be sent)
  If ucEncoderType = 0 then the direction and speed pins will be used and switches SW1-3, SW1-4, SW1-9, and SW1-10 should be turned on, on on the MSE-Duino board
   NOT TESTED - If ucEncoderType = 1 then the Quadrature pins A, B will be used and switches SW1-1, SW1-2, SW1-7, and SW1-8 should be turned on, on on the MSE-Duino board
   NOT TESTED -  If ucEncoderType = 2 then the direction, speed, and Quadrature pins A, B pins will be used and switches SW1-1, SW1-2, SW1-3, SW1-4, SW1-7, SW1-8, SW1-9, and SW1-10 should be turned on, on on the MSE-Duino board
  
  
*/
		void Begin(unsigned char ucEncoderType,void (*ISR_callback_Left)(void), void (*ISR_callback_Right)(void)); 
/*
  NOT TESTED -  Begin with set up both encoders: (ucEncoderType 3 - 5 pin data will need to be sent)
  If ucEncoderType = 3 then the direction and speed pins will be used and switches SW1-3, SW1-4, SW1-9, and SW1-10 should be turned on, on on the MSE-Duino board
  If ucEncoderType = 4 then the Quadrature pins A, B will be used and switches SW1-1, SW1-2, SW1-7, and SW1-8 should be turned on, on on the MSE-Duino board
  
*/
	    void Begin(unsigned char ucEncoderType, unsigned char ucEncoderLeftPin1,unsigned char ucEncoderLeftPin2,unsigned char ucEncoderRightPin1,unsigned char ucEncoderRightPin2); 
/*
  NOT TESTED -  Begin with set up both encoders: (ucEncoderType 3 - 5 pin data will need to be sent)
  If ucEncoderType = 5 then the direction, speed, and Quadrature pins A, B pins will be used and switches SW1-1, SW1-2, SW1-3, SW1-4, SW1-7, SW1-8, SW1-9, and SW1-10 should be turned on, on on the MSE-Duino board
  
*/
	    void Begin(unsigned char ucEncoderType, unsigned char ucEncoderLeftA,unsigned char ucEncoderLeftB,unsigned char ucEncoderLeftDir,unsigned char ucEncoderLeftSpd,unsigned char ucEncoderRightA,unsigned char ucEncoderRightB,unsigned char ucEncoderRightDir,unsigned char ucEncoderRightSpd); 	
		
		
		void end();
		
		void getEncoderRawSpeed();
		void getEncoderRawCount();
		void clearEncoder();
		
		void IRAM_ATTR LeftSpd_Encoder_ISR();  
		void IRAM_ATTR RightSpd_Encoder_ISR();  
		
		long lRawEncoderLeftCount;
		long lRawEncoderRightCount;
		long lRawEncoderLeftSpeed;
		long lRawEncoderRightSpeed;
		
private:
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
		
		
      //  static void ENC_isrLeftSpd();
		
		void (*ISR_callback_Left)();
		void (*ISR_callback_Right)();
				
		
	
	    void static ENC_isrLSpd();
		
       

        volatile long ENC_vlLeftEncoderRawSpeed;
		volatile long ENC_vlRightEncoderRawSpeed;
		
				
		volatile long ENC_vlLeftOdometer;
        volatile long ENC_vlRightOdometer;
		
		
	
		
		
};

#endif /* MSE2202_LIB_H */
