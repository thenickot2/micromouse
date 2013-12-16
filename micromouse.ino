//maze parameters
#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8           
#define VISITED      16
#define ONROUTE      32
//micromouse parameters
#define NORTHO		1
#define EASTO		2
#define WESTO		4  
#define SOUTHO		16
#define FORWARD		8
//path parameters
#define STRAWAY		1				//Start of a straightaway
#define RIGHTTURN	2				//Start of a right turn
#define LEFTTURN	4				//Start of a left turn
#define URIGHT		8				//Start of a 180 to the right
#define ULEFT		16				//Start of a 180 to the left
#define LEFTDIAGONAL		32		
#define RIGHTDIAGONAL		64
//tempSensorDataWalls
#define	TFORWARD	1
#define TRIGHT		2
#define	TLEFT		4
#define TUPRIGHT	8
#define TUPLEFT		16
int rotationfactor=153329;
unsigned char SLOW=24;
//unsigned char SLOW=48;
unsigned char FAST=191;
unsigned char SENSOR_OFFSET=0;

//int LF_WALL=115+SENSOR_OFFSET; no longer need this
//int LF_STOP=130+SENSOR_OFFSET;

/////////////////////////////////////////////////////////////
                  //SENSOR VALUES TO ADJUST
/////////////////////////////////////////////////////////////                    

//note 1: MAKE SURE THE BATTERY IS ON WHEN READING NEW VALUES!
//It will still return values through the serial print if it's
//just being powered by the micro USB, but these will be less
//than the values you would get from when it is battery powered.
//so make sure the battery switch is on when taking values.

//note 2: if many constants are changing at once, try
//changing the battery. It could be that it's just a
//low battery.

//These values are for wall detection. We want it to
//consistently detect a wall when it's on the opposite
//edge of the cell. Eg, we want it to detect the left
//wall when it's on the far right of the cell. To set
//the values, make them 4 lower than the lowest value
//read when its placed on the opposite edge of the cell.
int RF_WALL=770; //RF = right front
int L_WALL=348;//330;//345;//349;//343+SENSOR_OFFSET;
int R_WALL=238;//231;//239;//233+SENSOR_OFFSET;

//This value is the right front IR value at which it starts
//breaking. A higher value means it will wait longer to stop.
//A lower value means it will stop earlier.
int RF_STOP=885+SENSOR_OFFSET; //this value is optimized to stop in the center for one cell long stretches. Overshoots for longer stretches.   //844+SENSOR_OFFSET; this value works for both short and long stretches in the maze. Undershoots a bit if a short stretch.

//These values are used for our correction algorithm. They
//should be set to the value at the center of the cell.
//To find the right values, put the micromouse exactly in
//between two walls on the left and right, and set them to
//the values it reads for the left and right
int L_IDEAL=362+SENSOR_OFFSET;
int R_IDEAL=248+SENSOR_OFFSET;

//This is the encoder count for moving forward one cell.
//We want to adjust it such that, along with braking, it will
//move forward one cell pretty exactly, using only the encoders.
int FORWARD_ONE_CELL_ENCODER_COUNT = 7000;//7150;//(106500/16);//7000;//7300//4900;
          
//These are the values for gyro rotation. A higher value
//means it will rotate more in that direction. We want to 
//adjust these such that rotate 90 will turn pretty exactly 
//90 degrees in the specified direction.
int CLOCKWISE_ROTATE = 12050;//12700;//12900;//10100; 
int COUNTERCLOCKWISE_ROTATE = 12400;//13400;//13400;//10600;

//These are the values to make sure it's moving symmetrically.
//the forward motor symmetry constant should be adjusted such
//that it will move forward perfectly straight, and the braking
//motor symmetry constant should be adjusted such that when it
//brakes, it will brake perfectly backwards. A bigger forward
//motor constant will make it move more to the left, and a
//bigger braking motor constant will make it brake more to the
//left (like it will point more to the left after it's done 
//braking)
int FORWARD_MOTOR_SYMMETRY = 10;
int BRAKING_MOTOR_SYMMETRY = 2;
int BRAKING_MOTOR_SYMMETRY2 = 3;

//there are other brake constants, in case it's not stopping, 
//not stopping in the center, or starts moving backwards 
//constantly, but those can be adjusted in the brake function
//and probably won't need to be adjusted as much.

//this value is to be adjusted if it's stopping a long while
//before it gets to the wall (which is indicative that it's
//stopping using the encoders when it should be stopping using
//the IR sensors). A higher value will make it stop using the 
//IR sensors furthur forward (though there is a chance that it
//might mistakenly stop with the IR sensors, if the mouse
//overshoots.)
int EXTRA_MOVING_FORWARD_WALL_DETECT = 0;


//Victor, you probably won't need to correct the values below.

//this value is to be adjusted to change the power of our 
//correction function which adds power when encoder values
//deviate from their ideal values. A higher value means it
//will try to correct it harder. 
int POWER_ADDED_FOR_CENTER_BASED_CORRECTION = 10;

//This value determines how far from the ideal values
//it is allowed to deviate before it starts correcting.
int SLACK_FOR_WHEN_TO_MOTOR_COMPENSATE = 10;

//This value determines how much power is added for the 
//straightening function. 
int POWER_ADDED_FOR_STRAIGHTENING_BASED_CORRECTION = 20;

/////////////////////////////////////////////////////////////
             //END OF SENSOR VALUES TO ADJUST
/////////////////////////////////////////////////////////////                    

int numForceUpdate;
boolean clockwiseForce=0;

//variables used for the encoder
int left_encoder_count = 0; //global
int right_encoder_count = 0; //global
boolean direction_of_left_motor_is_forward = true; //global
boolean direction_of_right_motor_is_forward = true; //global                

boolean L_CHANNEL_A=0;
boolean L_CHANNEL_B=0;
boolean R_CHANNEL_A=0;
boolean R_CHANNEL_B=0;
boolean L_CHANNEL_A_OLD=digitalRead(8);
boolean L_CHANNEL_B_OLD=digitalRead(7);
boolean R_CHANNEL_A_OLD=digitalRead(1);
boolean R_CHANNEL_B_OLD=digitalRead(2);
void left_encoder_A(); //I'm declaring these functions, so that the
void right_encoder_A(); //attachInterrupt in setup won't cause error.
void left_encoder_B(); //defined fully under teensy functions
void right_encoder_B();

//void left_encoder_increment(); //I'm declaring these functions, so that the
//void right_encoder_increment(); //attachInterrupt in setup won't cause error.
                                //defined fully under teensy functions

  void updateMaze();
  void flood(boolean var);
  boolean moveExploratory(boolean var);
  boolean moveSlow(unsigned char dest);	//moves to an adjacent cell, returns true if success, false if collision
  void analyzeFastest();
  void moveSpeed();
  boolean moveForward(boolean slow, boolean turn);
  boolean smoothTurn(boolean left);
  boolean rotate(unsigned char degrees, boolean clockwise);				//stays in same cell
  unsigned char current=0;	//current spot in the maze
  unsigned char maze[256];	//walls
  unsigned char map1[256];	//flood values used to be map
  unsigned char path[256];	//path used for speedroute key turns/actions
  unsigned char orientation=NORTHO;		//orientation use to be status
  boolean pathChanged;

void setup()
{
  pinMode(0,OUTPUT); //LED 4  
  pinMode(1,INPUT); //R Encoder A
  pinMode(2,INPUT); //R Encoder B
  pinMode(3,OUTPUT); //R PWM A    //channel 3 and 6 means both forwards.
  pinMode(4,OUTPUT); //R PWM B    //channel 4 and 5 means both backwards.
  pinMode(5,OUTPUT); //L PWN B
  pinMode(6,OUTPUT); //L PWN A
  pinMode(7,INPUT); //L Encoder B
  pinMode(8,INPUT); //L Encoder A
  pinMode(9,OUTPUT); //Front Sensors
  pinMode(10,OUTPUT); //Right bottom Sensor          ****
  pinMode(11,OUTPUT); //Left bottom Sensor            ****
  pinMode(12,OUTPUT); //middle Sensors                ****
  pinMode(13,OUTPUT); //LED 2
  pinMode(14,OUTPUT); //LED 1
  pinMode(15,INPUT); //left bottom Sensor            ****
  pinMode(16,INPUT); //left middle Sensor            ****
  pinMode(17,INPUT); //Gyro VRef
  pinMode(18,OUTPUT); //LED 3
  pinMode(19,INPUT); //Gyro Pin Z/X
  pinMode(20,INPUT); //Left Front Sensor            ****
  pinMode(21,INPUT); //Voltmeter
  pinMode(22,INPUT); //Right bottom Sensor            ****
  pinMode(23,OUTPUT); //BT1 of LED part of schematic
  pinMode(A10,INPUT); //right middle Sensor          ****
  pinMode(A11,INPUT); //Right Front Sensor            ****
  
  Serial.begin(9600); //Comment out later
  
  //assign basic walls
  for(unsigned char k=0;k<255;k++) {maze[k]=0;} maze[255]=0; 
  maze[0]=(SOUTH | WEST);maze[15]=(NORTH | WEST);maze[240]=(SOUTH | EAST);maze[255]=(NORTH | EAST);
  for(unsigned char k=16;k<240;k=k+16)
      maze[k]=(SOUTH);
  for(unsigned char k=31;k<255;k=k+16)
      maze[k]=(NORTH);
  for(unsigned char k=1;k<15;k++)
      maze[k]=(WEST);
  for(unsigned char k=241;k<255;k++)
      maze[k]=(EAST);
  maze[0]=maze[0]+VISITED;

  //setup to make it so that whenever the encoder is incremented, 
  //it interrupts our main function to add one to the encoder 
  //attachInterrupt(1, left_encoder_increment, CHANGE); //interrupt number corresponds to pin number
  //attachInterrupt(8, right_encoder_increment, CHANGE);                
  attachInterrupt(1, left_encoder_A, CHANGE); //interrupt number corresponds to pin number
  attachInterrupt(8, right_encoder_A, CHANGE);
  attachInterrupt(2, left_encoder_B, CHANGE);
  attachInterrupt(7, right_encoder_B, CHANGE);

  numForceUpdate=0;
}
//TEENSY FUNCTIONS******************************************************************************************************************************
                boolean powerLeftMotor(unsigned char speed1, boolean forward)
		{
                        unsigned char turn_on_pin;
                        unsigned char turn_off_pin;
                        if (forward==1)
                        {  
                          turn_on_pin=6;
                          turn_off_pin=5;
                          direction_of_left_motor_is_forward = 1;
                        }
                        else
                        {
                          turn_on_pin=5;
                          turn_off_pin=6;
                          direction_of_left_motor_is_forward = 0;
                        }
                        analogWrite(turn_on_pin,speed1);
                        analogWrite(turn_off_pin,0);
			return true;
		}
		boolean powerRightMotor(unsigned char speed1, boolean forward)
		{
                        unsigned char turn_on_pin;
                        unsigned char turn_off_pin;
                        if (forward==1)
                        {
                          turn_on_pin=3;
                          turn_off_pin=4;
                          direction_of_right_motor_is_forward = 1;
                        }
                        else
                        {
                          turn_on_pin=4;
                          turn_off_pin=3;
                          direction_of_right_motor_is_forward = 0;
                        }
                        analogWrite(turn_on_pin,speed1);
                        analogWrite(turn_off_pin,0);
			return true;
		}
                void powerOffMotors()
                {
                  analogWrite(3,0);analogWrite(4,0);analogWrite(5,0);analogWrite(6,0);
                }

   	 unsigned char powerSensors()
   	 {
                    	unsigned char value=0;
              	 
                    	digitalWrite(10, 1);
                    	delayMicroseconds(60);
                    	int RFSensor = analogRead(A8);
                    	digitalWrite(10, 0);
                    	delayMicroseconds(80);
                   	 
                    	digitalWrite(9, 1);
                    	delayMicroseconds(60);
                    	int leftSensor = analogRead(A6);
                    	int rightSensor = analogRead(A11);  
                    	digitalWrite(9, 0);
                   	 
                    	digitalWrite(10, 1);
                    	delayMicroseconds(60);
                    	int RFSensor2 = analogRead(A8);
                    	digitalWrite(10, 0);
                    	delayMicroseconds(80);
                   	 
                    	digitalWrite(9, 1);
                    	delayMicroseconds(60);
                    	int leftSensor2 = analogRead(A6);
                    	int rightSensor2 = analogRead(A11);  
                    	digitalWrite(9, 0);
                                                
                    	RFSensor=(RFSensor+RFSensor2)/2;leftSensor=(leftSensor+leftSensor2)/2;rightSensor=(rightSensor+rightSensor2)/2;
                    	if ((RFSensor >RF_WALL))
                      	value=value|TFORWARD;
                    	if (leftSensor > L_WALL)
                      	value=value|TLEFT;
                    	if (rightSensor > R_WALL)
                      	value=value|TRIGHT;
   		 return value;
   	 }
            	unsigned char getFrontSensors() // 1 if wall is within threshold, 2 to stop, 0 if no wall
            	{
                    	unsigned char value=0;                  	 
                    	digitalWrite(10, 1);
                    	delayMicroseconds(60);
                    	int RFSensor = analogRead(A8);
                    	digitalWrite(10, 0);
                    	//digitalWrite(11, 1);
                    	//delayMicroseconds(60);
                    	//int LFSensor = analogRead(A1);
                    	//digitalWrite(11, 0);
                    	delayMicroseconds(80);
                   	 
                    	digitalWrite(10, 1);
                    	delayMicroseconds(60);
                    	int RFSensor2 = analogRead(A8);
                    	digitalWrite(10, 0);
                    	//digitalWrite(11, 1);
                    	//delayMicroseconds(60);
                    	//int LFSensor2 = analogRead(A1);
                    	//digitalWrite(11, 0);
                    	//delayMicroseconds(80);
                   	 
                    	RFSensor=(RFSensor+RFSensor2)/2; //LFSensor=(LFSensor+LFSensor2)/2;
                    	if ((RFSensor > RF_WALL-EXTRA_MOVING_FORWARD_WALL_DETECT))// || (LFSensor > LF_WALL)) //this works, but it relies on the wrong sensors
                      	value=1;
                    	if ((RFSensor > RF_STOP))
                      	value=2;
                    	return value;
            	}

   	 unsigned char compareSideSensors() //1 add to right, 2 add to left
   	 {
   		 digitalWrite(9, 1);
                    	delayMicroseconds(60);
                    	int leftSensor = analogRead(A6);
                    	int rightSensor = analogRead(A11);  
                    	digitalWrite(9, 0);
                    	digitalWrite(9, 1);
                    	delayMicroseconds(60);
                    	int leftSensor2 = analogRead(A6);
                    	int rightSensor2 = analogRead(A11);  
                    	digitalWrite(9, 0);
                   	 
                    	leftSensor=(leftSensor+leftSensor2)/2;rightSensor=(rightSensor+rightSensor2)/2;
                    	if ((leftSensor < L_WALL) && (rightSensor< R_WALL))
                      	  return 3;
                    	if (leftSensor > (L_IDEAL+SLACK_FOR_WHEN_TO_MOTOR_COMPENSATE))
                      	  return 2;
                    	if (rightSensor > (R_IDEAL+SLACK_FOR_WHEN_TO_MOTOR_COMPENSATE))
                      	  return 1;
                    	return 0;
            	}  
            
		int getOrientation()
		{
                        //pin17 VRef    19 z/x
                        return (analogRead(19)-analogRead(17));
		}
		int readLeftEncoder()
		{
			return left_encoder_count;
		}
		int readRightEncoder()
		{
			return right_encoder_count;
		}

                void left_encoder_A()
                {
                if (direction_of_left_motor_is_forward)
                left_encoder_count++;
                else
                left_encoder_count--;
                L_CHANNEL_A_OLD=L_CHANNEL_A;
                L_CHANNEL_A=digitalRead(8);
                }
                void right_encoder_A()
                {
                if (direction_of_right_motor_is_forward)
                right_encoder_count++;
                else
                right_encoder_count--;
                R_CHANNEL_A_OLD=R_CHANNEL_A;
                R_CHANNEL_A=digitalRead(1);
                }
                void left_encoder_B()
                { 
                L_CHANNEL_B_OLD=L_CHANNEL_B;
                L_CHANNEL_B=digitalRead(7);
                }
                void right_encoder_B()
                {
                R_CHANNEL_B_OLD=R_CHANNEL_B;
                R_CHANNEL_B=digitalRead(2);
                }
                

//HIGHER LEVEL FUNCTIONS ************************************************************************************************************************************************
//***********************************************************************************************************************************************************************
        void updateMaze(boolean force)
	{
		if (force==1){
                  numForceUpdate++;
                  if (((maze[current] & NORTH)!=0) && current <255){
			maze[current+1]=maze[current+1] - SOUTH; maze[current]=-NORTH;}
		  if (((maze[current] & EAST)!=0)){
			maze[current+16]=maze[current+16] - WEST; maze[current]=-EAST;}
		  if (((maze[current] & SOUTH)!=0) && current>0){
			(maze[current-1])=maze[current-1] - NORTH; maze[current]=-SOUTH;}
		  if (((maze[current] & WEST)!=0)){
			maze[current-16]=maze[current-16] - EAST; maze[current]=-WEST;}
                  maze[current]=0;
                  delay(500);
                  rotate(90,clockwiseForce);
                  rotate(90,clockwiseForce);
                  updateMaze(0);
                  rotate(90,clockwiseForce);
                  rotate(90,clockwiseForce);
                  clockwiseForce=(!clockwiseForce);
                  if(numForceUpdate>2)
                  {
                    for(unsigned char k=0;k<255;k++) {maze[k]=0;} maze[255]=0; 
                    maze[0]=(SOUTH | WEST);maze[15]=(NORTH | WEST);maze[240]=(SOUTH | EAST);maze[255]=(NORTH | EAST);
                    for(unsigned char k=16;k<240;k=k+16)
                            maze[k]=(SOUTH);
                    for(unsigned char k=31;k<255;k=k+16)
                            maze[k]=(NORTH);
                    for(unsigned char k=1;k<15;k++)
                            maze[k]=(WEST);
                    for(unsigned char k=241;k<255;k++)
                            maze[k]=(EAST);
                    numForceUpdate=0;
                  }
                 }

		//get sensor data and make sense of it
                unsigned char tempwalls;
		unsigned char tempwalls2;
                boolean brake=0;
                do{
                  tempwalls=powerSensors();tempwalls2=powerSensors();
                  if((tempwalls!=tempwalls2)&&(brake==0))
                    {quadBrake();brake=1;}
                }while(tempwalls!=tempwalls2);
                if((tempwalls & TFORWARD)!=0)
		{
			if((orientation & NORTHO)!=0)
				maze[current]=maze[current] | NORTH;
			else if((orientation & EASTO)!=0)
				maze[current]=maze[current] | EAST;
			else if((orientation & WESTO)!=0)
				maze[current]=maze[current] | WEST;
			else if((orientation & SOUTHO)!=0)
				maze[current]=maze[current] | SOUTH;
		}
		if((tempwalls & TRIGHT)!=0)
		{
			if((orientation & NORTHO)!=0)
				maze[current]=maze[current] | EAST;
			else if((orientation & EASTO)!=0)
				maze[current]=maze[current] | SOUTH;
			else if((orientation & WESTO)!=0)
				maze[current]=maze[current] | NORTH;
			else if((orientation & SOUTHO)!=0)
				maze[current]=maze[current] | WEST;
		}
		if((tempwalls & TLEFT)!=0)
		{
			if((orientation & NORTHO)!=0)
				maze[current]=maze[current] | WEST;
			else if((orientation & EASTO)!=0)
				maze[current]=maze[current] | NORTH;
			else if((orientation & WESTO)!=0)
				maze[current]=maze[current] | SOUTH;
			else if((orientation & SOUTHO)!=0)
				maze[current]=maze[current] | EAST;
		}
		//add walls to adjacent cells
		if (((maze[current] & NORTH)!=0) && ((maze[current+1] & SOUTH)==0) && current <255)
			maze[current+1]=maze[current+1] | SOUTH;
		if (((maze[current] & EAST)!=0) && ((maze[current+16] & WEST)==0))
			maze[current+16]=maze[current+16] | WEST;
		if (((maze[current] & SOUTH)!=0) && ((maze[current-1] & NORTH)==0) && current>0)
			(maze[current-1])=maze[current-1] | NORTH;
		if (((maze[current] & WEST)!=0) && ((maze[current-16] & EAST)==0))
			maze[current-16]=maze[current-16] | EAST;
	}
	void flood(boolean var)
	{
			unsigned char i,j;
			unsigned char now1=0,next=1;
			unsigned char passes=0;
			boolean changed;
			map1[255]=255;
			for(unsigned char k=0;k<255;k++)
				map1[k] = 255;
			if (var==1)				//the goal
			    //{map1[1*16+3*1]=0;}	//this value represents the end goal
                            {map1[128+8]=0;map1[128+7]=0;map1[112+8]=0;map1[112+7]=0;}
			else {map1[0]=0;} 
			//////
			do
			{
			changed = 0;
			i = 0;
			do
			{
			if (map1[i]==now1)
			{
				if ((maze[i] & NORTH) == 0)
				{
					j = i+1;
					if (map1[j] == 255){ map1[j] = next; changed=1;}
				}
				if ((maze[i] & EAST) == 0)
				{
					j = i + 16;
					if (map1[j] == 255){ map1[j] = next; changed=1;}
				}
				if ((maze[i] & SOUTH) == 0)
				{
					j = i - 1;
					if (map1[j] == 255){ map1[j] = next; changed=1;}
				}
				if ((maze[i] & WEST) == 0)
				{
					j = i - 16;
					if (map1[j] == 255){ map1[j] = next; changed=1;}
				}
			}
		i--;
		} while(i);
		next = ++now1;
		++next;
		passes++;
	        } while(changed);
	}

        boolean isClockwise(boolean left)
    	{
      	boolean A;boolean B;boolean ANew;boolean BNew;
      	if (left==1){
        	A=L_CHANNEL_A_OLD;B=L_CHANNEL_B_OLD;ANew=L_CHANNEL_A;BNew=L_CHANNEL_B;}
      	if (left==0){
        	A=R_CHANNEL_A_OLD;B=R_CHANNEL_B_OLD;ANew=R_CHANNEL_A;BNew=R_CHANNEL_B;}
      	if((A==0)&&(B==0)){
        	if((ANew==0)&&(BNew==1))
          	return 1;
        	else
          	return 0;}
       	if((A==0)&&(B==1)){
        	if((ANew==1)&&(BNew==1))
          	return 1;
        	else
          	return 0;}
       	if((A==1)&&(B==1)){
        	if((ANew==1)&&(BNew==0))
          	return 1;
        	else
          	return 0;}
       	if((A==1)&&(B==0)){
        	if((ANew==0)&&(BNew==0))
          	return 1;
        	else
          	return 0;}
       	return 0;
    	}
    	boolean isCClockwise(boolean left)
    	{
      	boolean A;boolean B;boolean ANew;boolean BNew;
      	if (left==1){
        	A=L_CHANNEL_A_OLD;B=L_CHANNEL_B_OLD;ANew=L_CHANNEL_A;BNew=L_CHANNEL_B;}
      	if (left==0){
        	A=R_CHANNEL_A_OLD;B=R_CHANNEL_B_OLD;ANew=R_CHANNEL_A;BNew=R_CHANNEL_B;}
      	if((A==1)&&(B==0)){
        	if((ANew==1)&&(BNew==1))
          	  return 1;
        	else
          	  return 0;}
       	if((A==1)&&(B==1)){
        	if((ANew==0)&&(BNew==1))
          	  return 1;
        	else
          	  return 0;}
       	if((A==0)&&(B==1)){
        	if((ANew==0)&&(BNew==0))
          	  return 1;
        	else
          	  return 0;}
       	if((A==0)&&(B==0)){
        	if((ANew==1)&&(BNew==0))
          	  return 1;
        	else
          	  return 0;}
       	return 0;
    	}
   	 
        boolean quadBrake()
    	{
        	powerOffMotors();
        	boolean RChange=0;boolean LChange=0;int change=0;
        	boolean power=0;
        	do
        	{
          	if(isCClockwise(1)){
            	LChange=change;
            	power=1;
            	powerLeftMotor(80+BRAKING_MOTOR_SYMMETRY2,0);}
          	else if (LChange<(change-2))
            	powerLeftMotor(0,0);
           	 
          	if(isClockwise(0)){
            	RChange=change;
            	power=1;
            	powerRightMotor(80-BRAKING_MOTOR_SYMMETRY2,0);}
          	else if (RChange<(change-2))
            	powerRightMotor(0,0);
          	if((isClockwise(1))||(isCClockwise(0))){
            	power=0;
            	change++;}
          	if (power==1){
            	powerLeftMotor(80+BRAKING_MOTOR_SYMMETRY2,0);powerRightMotor(80-BRAKING_MOTOR_SYMMETRY2,0);}
          	delay(1);
        	}while(change<28);
        	powerOffMotors();
        
                RChange=0;LChange=0;change=0;power=0;
                do
                {
                  if(isClockwise(1)){
                    LChange=change;
                    power=1;
                    powerLeftMotor(40+BRAKING_MOTOR_SYMMETRY2,1);}
                  else if (LChange<(change-2))
                    powerLeftMotor(0,0);
                    
                  if(isCClockwise(0)){
                    RChange=change;
                    power=1;
                    powerRightMotor(40-BRAKING_MOTOR_SYMMETRY2,1);}
                  else if (RChange<(change-2))
                    powerRightMotor(0,0);
                  if((isCClockwise(1))||(isClockwise(0))){
                    power=0;
                    change++;}
                  if (power==1){
                    powerLeftMotor(40+BRAKING_MOTOR_SYMMETRY2,1);powerRightMotor(40-BRAKING_MOTOR_SYMMETRY2,1);}
                  delay(1);
                }while(change<10);
                powerOffMotors();
       
            return true;
    	}
    
        boolean brake(boolean slow)
        {
         boolean done_braking = false;
         int period;
         int old_left_encoder_count = left_encoder_count;
         int old_right_encoder_count = right_encoder_count;
         
         if(slow)
         {
         powerRightMotor(SLOW, 0);
         powerLeftMotor(SLOW, 0);
         period = 15;
         }
         else
         {
         powerRightMotor(64-BRAKING_MOTOR_SYMMETRY, 0); //100 power = period 25, distance 60. 130 power = period 50, distance 180. FAST = period 100, distance 250. these values avoid misreadings via "burning rubber" backwards
         powerLeftMotor(64+BRAKING_MOTOR_SYMMETRY, 0);  //we might want a ratio of like period = 1/4 power
         period = 18; //EVEN WITH FAST BRAKES ON, it cannot stop in the center of a cell by braking when it first detects a wall (when going full speed). It must brake earlier.
         }
         delay(period);
         int less_old_left_encoder_count = left_encoder_count;
         int less_old_right_encoder_count = right_encoder_count;
         
         int dist_to_check_for_brake;
         if(slow)
           dist_to_check_for_brake = 10;
         else
           dist_to_check_for_brake = 36;           
         
         while(done_braking == false)
         {
           delay(period);
           //stop applying power in reverse if we traveled 15 encoders more than last time (indicative that we're going in reverse)
           if(  (( less_old_right_encoder_count - right_encoder_count) >  (dist_to_check_for_brake+(old_right_encoder_count - less_old_right_encoder_count)))  
             | ((less_old_left_encoder_count - left_encoder_count) >  (dist_to_check_for_brake+(old_left_encoder_count - less_old_left_encoder_count)))  )
           {
               done_braking = true;
           }
           else
           {
             old_left_encoder_count = less_old_left_encoder_count;
             old_right_encoder_count = less_old_right_encoder_count;
             less_old_left_encoder_count = left_encoder_count;
             less_old_right_encoder_count = right_encoder_count;
           }
         }
         powerOffMotors();
         return true;
       }

boolean moveForward(boolean slow)
{
  
    boolean useencoders=1;
    left_encoder_count = 0;
    right_encoder_count = 0;
    unsigned char speed1,addl,addr,compare;
    if (slow==1)
      speed1=SLOW;
    else
      speed1=FAST;    
    //boolean useencoders=1;
    boolean moveforward=1;
    unsigned char frontsensors=0;
    addl=0;addr=0;
    
    //constants for straightening based correction
    //function
    int leftSensorOld = 0;
    int rightSensorOld = 0;          
    int oldTime = 0;
    boolean isCenterCorrecting = false;
    
    while(moveforward)
    { 
      frontsensors=getFrontSensors();
      if (frontsensors > 0)
        useencoders=0;
      if (useencoders && (left_encoder_count > FORWARD_ONE_CELL_ENCODER_COUNT) && (right_encoder_count > FORWARD_ONE_CELL_ENCODER_COUNT))
        {moveforward=0;}
      if (!useencoders && (frontsensors==2))
        {moveforward=0;}
  
      digitalWrite(9, 1);
      delayMicroseconds(60);
      int leftSensor = analogRead(A6);
      int rightSensor = analogRead(A11);  
      digitalWrite(9, 0);
      digitalWrite(9, 1);
      delayMicroseconds(60);
      int leftSensor2 = analogRead(A6);
      int rightSensor2 = analogRead(A11);  
      digitalWrite(9, 0);
                     	 
      leftSensor=(leftSensor+leftSensor2)/2;rightSensor=(rightSensor+rightSensor2)/2;
                      
//int L_WALL=325//307+SENSOR_OFFSET;
//int R_WALL=226//209+SENSOR_OFFSET;

      //if(false)
      if (((rightSensor < (R_WALL+SLACK_FOR_WHEN_TO_MOTOR_COMPENSATE)) && (rightSensor > R_WALL)) || (leftSensor > 400))
      //if (leftSensor > 400)
      //if ((rightSensor < 246) && (rightSensor > R_WALL))
      {
        powerLeftMotor(speed1+24,1);
        powerRightMotor(0,1);
        isCenterCorrecting = false;
      }

      else if (((leftSensor < (L_WALL+SLACK_FOR_WHEN_TO_MOTOR_COMPENSATE)) && (leftSensor > L_WALL)) || (rightSensor > 300)) //&&&&
      //else if (rightSensor > 300)
      //else if (((leftSensor < 345) && (leftSensor > L_WALL)) || (rightSensor > 300))
      {
        powerLeftMotor(0,1);
        powerRightMotor(speed1+24+FORWARD_MOTOR_SYMMETRY,1);
        isCenterCorrecting = false;
      }
      else
      {
        powerLeftMotor(speed1,1);
        powerRightMotor(speed1+FORWARD_MOTOR_SYMMETRY,1);
      }
    }
//  if(useencoders)
//  {
    //quadBrake();
    //brake(0);
    //delay(500);
//  }  
  if((left_encoder_count < 2750) || (right_encoder_count < 2750))
    return false;
  return true;
}

/*    if(compare == 3)
    {
      addl=0; addr=0;
      isCenterCorrecting = false;
    }
    
    //correction function when deviating from the 
    //center too far
    if((compare == 1) || (compare == 2))
    {
      if (compare==1) //closer to right
      {
        addr = POWER_ADDED_FOR_CENTER_BASED_CORRECTION;
        addl = 0;
      }
      else if(compare==2)
      {
        addl = POWER_ADDED_FOR_CENTER_BASED_CORRECTION;
        addr = 0;
      }
      isCenterCorrecting = false;              
    }

    //correction function for straightening it when
    //it's relatively close to the center
    if(true)
    {
      if(!isCenterCorrecting)
      {
        isCenterCorrecting = true;
        oldTime = micros();
        
        digitalWrite(9, 1);
        delayMicroseconds(60);
        int leftSensor = analogRead(A6);
        int rightSensor = analogRead(A11);  
        digitalWrite(9, 0);
        digitalWrite(9, 1);
        delayMicroseconds(60);
        int leftSensor2 = analogRead(A6);
        int rightSensor2 = analogRead(A11);  
        digitalWrite(9, 0);

        leftSensorOld=(leftSensor+leftSensor2)/2;rightSensorOld=(rightSensor+rightSensor2)/2;
      }

      else if ((micros() - oldTime)>150000)
      {
        digitalWrite(9, 1);
        delayMicroseconds(60);
        int leftSensor = analogRead(A6);
        int rightSensor = analogRead(A11);  
        digitalWrite(9, 0);
        digitalWrite(9, 1);
        delayMicroseconds(60);
        int leftSensor2 = analogRead(A6);
        int rightSensor2 = analogRead(A11);  
        digitalWrite(9, 0);
    
        leftSensor=(leftSensor+leftSensor2)/2;rightSensor=(rightSensor+rightSensor2)/2;
    
        oldTime = micros();
  
        if ((leftSensor > leftSensorOld+2) && (rightSensor < rightSensorOld-2))
        {
          addl = POWER_ADDED_FOR_STRAIGHTENING_BASED_CORRECTION;
          addr = -POWER_ADDED_FOR_STRAIGHTENING_BASED_CORRECTION;
        }  
        else if ((leftSensor < leftSensorOld-2) && (rightSensor > rightSensorOld+2))
        {
          addl = -POWER_ADDED_FOR_STRAIGHTENING_BASED_CORRECTION;
          addr = POWER_ADDED_FOR_STRAIGHTENING_BASED_CORRECTION;
        }  
        else
        {
          addl = 0;
          addr = 0;        
        }
  
        rightSensorOld = rightSensor;
        leftSensorOld = leftSensor;
    
      }

    }

    powerLeftMotor(speed1+addl,1);
    powerRightMotor(speed1+addr+FORWARD_MOTOR_SYMMETRY,1);
  
  }
  return true;
}
*/


    boolean rotate(unsigned char degrees1, boolean clockwise)
    {
      int gyroSum = 0;
      int gyroValue = 0;
      
      //THESE VALUES NEEDS TO BE TWEAKED DEPENDING ON THE SURFACE IT'S ON!
      //ALSO NOTE THAT COUNTER-CLOCKWISE IS NOT THE NEGATIVE OF THE CLOCKWISE TURN VALUE!

      //Also note that clockwiseGyroCountToTurn is NEGATIVE, so you need to check if gyroSum
      //is MORE than it. On the other hand, counterClockwiseGyroCountToTurn is POSITIVE, so 
      //you check if gyroSum is LESS than counterClockwiseGyroCountToTurn.
      
      int clockwiseGyroCountToTurn = -((int) (((float) degrees1)/90*CLOCKWISE_ROTATE));//12000));
      int counterClockwiseGyroCountToTurn = ((int) (((float) degrees1)/90*COUNTERCLOCKWISE_ROTATE));//12000));
      
      if (clockwise == 1)
      {
        powerLeftMotor(80, 1);
        powerRightMotor(80, 0);
      
        while(gyroSum > clockwiseGyroCountToTurn)
        {
          gyroValue = getOrientation()+9;
          if ((gyroValue > 4) || (gyroValue < -4))
            gyroSum += gyroValue;
          delay(1);
        }
      
        powerLeftMotor(100, 0);
        powerRightMotor(100, 1);
      
        while( (getOrientation()+9) < -4)
        {
          delay(1);
        }  
      }
      
      else
      {
        powerLeftMotor(80, 0);
        powerRightMotor(80, 1);
      
        while(gyroSum < counterClockwiseGyroCountToTurn)
        {
          gyroValue = getOrientation()+9;
          if ((gyroValue > 4) || (gyroValue < -4))
            gyroSum += gyroValue;
          delay(1);
        }
      
        powerLeftMotor(100, 1);
        powerRightMotor(100, 0);
      
        while( (getOrientation()+9) > 4)
        {
          delay(1);
        }      
      }
      powerOffMotors();
           	 
      if(orientation==NORTHO)
      {
        if(clockwise==1 && degrees1==90)
          orientation=EASTO;
        else if(degrees1==180)
          orientation=SOUTHO;
        else if(clockwise==0 && degrees1==90)
          orientation=WESTO;
      }
      else if(orientation==EASTO)
      {
        if(clockwise==1 && degrees1==90)
          orientation=SOUTHO;
        else if(degrees1==180)
          orientation=WESTO;
        else if(clockwise==0 && degrees1==90)
          orientation=NORTHO;
      }
      else if(orientation==SOUTHO)
      {
        if(clockwise==1 && degrees1==90)
          orientation=WESTO;
        else if(degrees1==180)
          orientation=NORTHO;
        else if(clockwise==0 && degrees1==90)
          orientation=EASTO;
      }
      else if(orientation==WESTO)
      {
        if(clockwise==1 && degrees1==90)
          orientation=NORTHO;
        else if(degrees1==180)
          orientation=EASTO;
        else if(clockwise==0 && degrees1==90)
          orientation=SOUTHO;
      }
      delay(500);
      return true;
    }


	boolean smoothTurn(boolean left)
	{
		do
		{
		}while(0);
		return true;
	}
        boolean Uturn(boolean left)
	{
                do
		{
                  if (left==0)
                  {
                    powerLeftMotor(2*50,1);
                    powerLeftMotor(50,1);
                  }
		}while(0);
		while(1){};
		return true;
	}

	boolean moveExploratory(boolean var)
	{
  		path[current]=1;
		updateMaze(0);
		flood(var);

		unsigned char left,right,up,down;
		if ((maze[current] & NORTH)==0)
			{up=map1[current+1];}
		else
			{up=255;}
		if ((maze[current] & EAST)==0)
			{right=map1[current+16];}
		else
			{right=255;}
		if ((maze[current] & SOUTH)==0)
			{down=map1[current-1];}
		else
			{down=255;}
		if ((maze[current] & WEST)==0)
			{left=map1[current-16];}
		else
			{left=255;}
		//sort the values
		boolean swapped = false;
		unsigned char A[4]; A[0]=up; A[1]=down; A[2]=left; A[3]=right;
		do
		{
			swapped=false;
			for (int i=1;i<4;i++)
			{
				if (A[i-1] > A[i])
				{
					unsigned char temp=A[i-1];
					A[i-1]=A[i];
					A[i]=temp;
					swapped=true;
				}
			}
		}while(swapped==true);
		if (A[0]==255)
                        updateMaze(1);
		else if (A[0]==up)
			moveSlow(current+1);
		else if (A[0]==right)
			moveSlow(current+16);
		else if (A[0]==left)
			moveSlow(current-16);
		else if (A[0]==down)
			moveSlow(current-1);
                if (A[0]!=255)
                        numForceUpdate=0;
		return true;
	}
	boolean moveSlow(unsigned char dest)
	{
		if((orientation & NORTHO)!=0)
		{
			if(dest==current+16)
				{brake(0);delay(500);rotate(90,1);}
			else if(dest==current-16)
				{brake(0);delay(500);rotate(90,0);}
			else if(dest==current-1)
				{brake(0);delay(500);rotate(90,1);rotate(90,1);}
		}
		else if((orientation & EASTO)!=0)
		{
			if(dest==current+1)
				{brake(0);delay(500);rotate(90,0);}
			else if(dest==current-1)
				{brake(0);delay(500);rotate(90,1);}
			else if(dest==current-16)
				{brake(0);delay(500);rotate(90,1);rotate(90,1);}
		}
		else if((orientation & SOUTHO)!=0)
		{
			if(dest==current+16)
				{brake(0);delay(500);rotate(90,0);}
			else if(dest==current-16)
				{brake(0);delay(500);rotate(90,1);}
			else if(dest==current+1)
				{brake(0);delay(500);rotate(90,0);rotate(90,0);}
		}
		else if((orientation & WESTO)!=0)
		{
			if(dest==current+1)
				{brake(0);delay(500);rotate(90,1);}
			else if(dest==current-1)
				{brake(0);delay(500);rotate(90,0);}
			else if(dest==current+16)
				{brake(0);delay(500);rotate(90,0);rotate(90,0);}
		}
		boolean result = moveForward(1);
                if(result == true)
                {
  		  current=dest;
  		  maze[current]=maze[current]|VISITED;
  		}
                return true;
	}
	void analyzeFastest()
	{
		unsigned char n=0;	//length of route to center
		if (path[255]==0)
			map1[255]=0;
		else
		{
			n++;
			path[255]=0;
		}
		for(unsigned char k=0;k<255;k++)
		{
			if(path[k]==0)
				map1[k]=0;
			else
			{
				n++;
				path[k]=0;
			}
		}				//now map is only the ordered #s leading to the center
		
		for(unsigned char k=0;k<=n;k++)		//k is the value of the map
		{
			unsigned char m=0;				//m is pos in the array
			unsigned char n=0;				//previous pos
			for(m=0;m<255;m++)		//find position of current #
				if(map1[m]==k)
					break;
			//find straightaways
			if((map1[m+1]==k+1) && (map1[m+2]==k+2))
				path[m]=STRAWAY;
			if((map1[m+16]==k+1) && (map1[m+32]==k+2))
				path[m]=STRAWAY;
			if((map1[m-1]==k+1) && (map1[m-2]==k+2))
				path[m]=STRAWAY;
			if((map1[m-16]==k+1) && (map1[m-32]==k+2))
				path[m]=STRAWAY;
			//rightturns
			if((map1[m+1]==k+1) && (map1[m+1+16]==k+2))
				path[m]=RIGHTTURN;
			if((map1[m+16]==k+1) && (map1[m+16-1]==k+2))
				path[m]=RIGHTTURN;
			if((map1[m-1]==k+1) && (map1[m-1-16]==k+2))
				path[m]=RIGHTTURN;
			if((map1[m-16]==k+1) && (map1[m-16+1]==k+2))
				path[m]=RIGHTTURN;
			//leftturns
			if((map1[m+1]==k+1) && (map1[m+1-16]==k+2))
				path[m]=LEFTTURN;
			if((map1[m+16]==k+1) && (map1[m+16+1]==k+2))
				path[m]=LEFTTURN;
			if((map1[m-1]==k+1) && (map1[m-1+16]==k+2))
				path[m]=LEFTTURN;
			if((map1[m-16]==k+1) && (map1[m-16-1]==k+2))
				path[m]=LEFTTURN;
			//URIGHT
			if((map1[m+1]==k+1) && (map1[m+1+16]==k+2) && (map1[m+1+16-1]==k+3))
				path[m]=URIGHT;
			if((map1[m+16]==k+1) && (map1[m+16-1]==k+2) && (map1[m+16-1-16]==k+3))
				path[m]=URIGHT;
			if((map1[m-1]==k+1) && (map1[m-1-16]==k+2) && (map1[m-1-16+1]==k+3))
				path[m]=URIGHT;
			if((map1[m-16]==k+1) && (map1[m-16+1]==k+2) && (map1[m-16+1+16]==k+3))
				path[m]=URIGHT;
			//ULEFT
			if((map1[m+1]==k+1) && (map1[m+1-16]==k+2) && (map1[m+1-16-1]==k+3))
				path[m]=ULEFT;
			if((map1[m+16]==k+1) && (map1[m+16+1]==k+2) && (map1[m+16+1-16]==k+3))
				path[m]=ULEFT;
			if((map1[m-1]==k+1) && (map1[m-1+16]==k+2) && (map1[m-1+16+1]==k+3))
				path[m]=ULEFT;
			if((map1[m-16]==k+1) && (map1[m-16-1]==k+2) && (map1[m-16-1+16]==k+3))
				path[m]=ULEFT;
			//Diagonals
/*
			if(path[n]==LEFTTURN && path[m]==RIGHTTURN)
				path[n]=LEFTDIAGONAL;
			if(path[n]==RIGHTTURN && path[m]==LEFTTURN)
				path[n]=RIGHTDIAGONAL;
*/
			n=m;			//update previous
		}
	}
	void moveSpeed()
	{
		do
		{
                        unsigned char oldpos=current;
			if(path[current]==STRAWAY)
			{
				moveForward(0);
				if(orientation==NORTHO)
					current++;
				else if(orientation==EASTO)
					current+=16;
				else if(orientation==SOUTHO)
					current--;
				else if(orientation==WESTO)
					current+=-16;
			}
			if(path[current]==RIGHTTURN)
			{
				smoothTurn(0);
				if(orientation==NORTHO)
				{
					current=current+1+16;
					orientation=EASTO;
				}
				else if(orientation==EASTO)
				{
					current=current+16-1;
					orientation=SOUTHO;
				}
				else if(orientation==SOUTHO)
				{
					current=current-1-16;
					orientation=WESTO;
				}
				else if(orientation==WESTO)
				{
					current=current-16+1;
					orientation=NORTHO;
				}
			}
			if(path[current]==LEFTTURN)
			{
				smoothTurn(1);
				if(orientation==NORTHO)
				{
					current=current+1-16;
					orientation=WESTO;
				}
				else if(orientation==EASTO)
				{
					current=current+16+1;
					orientation=NORTHO;
				}
				else if(orientation==SOUTHO)
				{
					current=current-1+16;
					orientation=EASTO;
				}
				else if(orientation==WESTO)
				{
					current=current-16-1;
					orientation=SOUTHO;
				}
			}
			if(path[current]==URIGHT)
			{
				Uturn(0);
				if(orientation==NORTHO)
				{
					current=current+16;
					orientation=SOUTHO;
				}
				else if(orientation==EASTO)
				{
					current=current-1;
					orientation=WESTO;
				}
				else if(orientation==SOUTHO)
				{
					current=current-16;
					orientation=NORTHO;
				}
				else if(orientation==WESTO)
				{
					current=current+1;
					orientation=EASTO;
				}
			}
			if(path[current]==ULEFT)
			{
				Uturn(1);
				if(orientation==NORTHO)
				{
					current=current-16;
					orientation=SOUTHO;
				}
				else if(orientation==EASTO)
  				{
					current=current+1;
					orientation=WESTO;
				}
				else if(orientation==SOUTHO)
				{
					current=current+16;
					orientation=NORTHO;
				}
				else if(orientation==WESTO)
				{
					current=current-1;
					orientation=EASTO;
				}
				//todo diagonals
			}
		}while((current!=128+8) && (current!=128+7) && (current!=112+8) && (current!=112+7));
	}

void loop()
{
//  this code waits for the button to be pushed, and it starts moving 
// forward one cell at a time. I was using it to try to fix the motor asymmetry.
  while (analogRead(23) < 200)
  {
  }
  delay(1000);
  
  while(analogRead(21) < 630)
  {
    delay(100);
  }

  flood(1); //initial flood
  
  //first phases, exploratory
		boolean visited[256];
		boolean visitedafter[256];
		boolean changed;
		unsigned char passes=0;
		do
		{
			passes++;
			changed=0;
			path[255]=0;
			for(unsigned char k=0;k<255;k++)		//reset path
				path[k]=0;
			for(unsigned char k=0;k<255;k++)		//record the visited
			{
				if ((maze[k] & VISITED)==0) //notvisited
					visited[k]=0;
				else
					visited[k]=1;
			}
			//
			do
			{
				moveExploratory(1);
			}while(map1[current]!=0);
  
/*
  quadBrake();
  while (analogRead(23) < 200)
  {
  }
  delay(1000);
*/ //uncomment this and delete do while loop below for center retrieval strategy

			do
			{
				moveExploratory(0);
			}while(map1[current]!=0);


			//
			if ((maze[255] & VISITED)==0) //notvisited
				visitedafter[255]=0;
			else
				visitedafter[255]=1;
			for(unsigned char k=0;k<255;k++)
			{
				if ((maze[k] & VISITED)==0) //notvisited
					visitedafter[k]=0;
				else
					visitedafter[k]=1;
			}
			for(unsigned char k=0;k<255;k++)
				if(visited[k]!=visitedafter[k])
					changed=1;
		}while(changed==1);
		//secondphase, speed run
                rotate(90,0);rotate(90,0);
		analyzeFastest();
		//moveSpeed();

}
