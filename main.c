#include "i2c-master-test.h"

/*Uncomment and compile whichever control mode you would like to test.*/
#define POS_CONTROL_MODE
//#define TAU_CONTROL_MODE
//#define VELOCITY_CONTROL_MODE

#define CABLE_TEST

float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}
	int state;

int main()
{
	FILE *log; //log file init of file and cleaning
	log = fopen("log.txt","a");
	fclose(log);
	for (int i=0; i<10; ++i){
		old_cable_data[i]=0;
	}
	time_t now;
	int clock, secsPastMidnight, hours, minutes, seconds;

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	//set_grip(GENERAL_OPEN_CMD,100);
	//set_grip(POWER_GRASP_CMD,100);
	//usleep(3000000);
	//set_grip(GENERAL_OPEN_CMD,100);
	usleep(3000000);

	//set_mode(DISABLE_TORQUE_VELOCITY_SAFETY);	//uncomment for UNSAFE torque and velocity control modes

	/*Setpoint generation start time*/
	struct timeval tv;

	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	pres_union_fmt_i2c pres_fmt;

	int connect_flag[5];
	for (int cnt = 0; cnt<5; cnt++)
		connect_flag[cnt]=1;
	int prev_phase = 0;
	int phase = 0;

	/*Setup for demo motion*/
	uint8_t disabled_stat = 0;

	float q_stop[NUM_CHANNELS] = {0};
	float qd[NUM_CHANNELS] = {0};
	qd[THUMB_ROTATOR] = -80.f;

	float start_ts = current_time_sec(&tv);
	while(1)
	{
		float t = fmod(current_time_sec(&tv) - start_ts, 10);
		if(t >= 1 && t < 2)
		{	state = 1;
			phase = 1;
			float t_off = t-1.f;
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = t_off*50.f + 20.f;	//travel to 70 degrees (close) from 20 degrees (open)
			qd[THUMB_FLEXOR] = t_off*30.f+10.f; //travel to 40 degrees (close) from 10 degrees (open)

			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				q_stop[ch] = i2c_in.v[ch];	//record so when phase 1 is complete you know where the hand stopped
		}

		else if(t >= 6 && t < 7)
		{ state = 3;
			phase = 2;
			float t_off = (t-6.f);
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = t_off*(20.f-q_stop[ch]) + q_stop[ch];	//travel to 20 from where you currently are
			qd[THUMB_FLEXOR] = t_off*(10.f-q_stop[THUMB_FLEXOR])+q_stop[THUMB_FLEXOR];	//travel to 10 from where you currently are
		}
		else if(t > 7)
		{	phase = 3;
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = 20.f;			//enforce start position
			qd[THUMB_FLEXOR] = 10.f;		//for both sets of fingers
			state = 0;
		}
		else{
			phase = -1;
		}

		if(prev_phase != phase && prev_phase == -1)
		{
			//enable_cmd = 0x3f;
			send_enable_word(0x3F);		//should call this only once for optimum behavior
		}

		prev_phase = phase;

		if (t<1) state = 0;
		if (t>=2 && t <6) state = 2;
		/*
		Pressure Indices:
		Index: 	0-3
		Middle: 4-7
		Ring: 	8-11
		Pinky: 	12-15
		Thumb: 	16-19
		*/

		#ifdef CABLE_TEST
			
			int i, pidx;
			for(pidx = 0; pidx < 19; pidx+=2){
					cable_data[pidx/2]=((uint32_t)pres_fmt.v[pidx+1] << 16) +  pres_fmt.v[pidx];
//					printf("%x ", cable_data[pidx/2]);
			}
//			printf("\n");

			for (i= 0; i <10; i+=2){
					int finger = i/2;

					if ((cable_data[i]>old_cable_data[i])&&connect_flag[finger]==1){
						time(&now);
	    					clock = now - 18000;
	    					secsPastMidnight = clock % 86400;
	   					hours = (secsPastMidnight / 3600)-1;
	    					minutes = (secsPastMidnight % 3600) / 60;
	    					seconds = secsPastMidnight % 60;
						connect_flag[finger] = 0;
				 		log = fopen("log.txt","a");
				 		fprintf(log, "%02d:%02d:%02d ", hours, minutes, seconds);
				 		if (state==1)
				 		 {fprintf(log, "Finger %d disconnected during closing\n", finger+1);
							printf("Finger %d disconnected during closing\n", finger+1);}
				 		else if (state==3)
				 			{fprintf(log, "Finger %d disconnected during opening\n", finger+1);
							printf("Finger %d disconnected during opening\n", finger+1);}
						else if (state==0)
							{fprintf(log, "Finger %d disconnected while open \n", finger+1);
							printf("Finger %d disconnected while open \n", finger+1);}
						else
						{fprintf(log, "Finger %d disconnected while closed \n", finger+1);
						printf("Finger %d disconnected while closed \n", finger+1);}
				 		fclose(log);
						old_cable_data[i]=cable_data[i];
						old_cable_data[i+1]=cable_data[i+1];
					}
					else if ((cable_data[i+1]>old_cable_data[i+1])&&connect_flag[finger]==0){
						connect_flag[finger] = 1;
				 		log = fopen("log.txt","a");
						time(&now);
	    					clock = now - 18000;
	   					secsPastMidnight = clock % 86400;
	    					hours = (secsPastMidnight / 3600)-1;
	    					minutes = (secsPastMidnight % 3600) / 60;
	    					seconds = secsPastMidnight % 60;
						fprintf(log, "%02d:%02d:%02d ", hours, minutes, seconds);
				 		if (state==1)
				 		 {fprintf(log, "Finger %d connected during closing\n", finger+1);
							printf("Finger %d connected during closing\n", finger+1);}
				 		else if (state==3)
				 			{fprintf(log, "Finger %d connected during opening\n", finger+1);
							printf("Finger %d connected during opening\n", finger+1);}
						else if (state==0)
							{fprintf(log, "Finger %d connected while open \n", finger+1);
							printf("Finger %d connected while open \n", finger+1);}
						else
						{fprintf(log, "Finger %d connected while closed \n", finger+1);
						printf("Finger %d connected while closed \n", finger+1);}
 				 		fclose(log);
						old_cable_data[i]=cable_data[i];
						old_cable_data[i+1]=cable_data[i+1];
					}
			}

		#endif

		#ifdef POS_CONTROL_MODE
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = qd[ch];
			int rc = send_recieve_floats(POS_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		#endif
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
	}
return 0;
}
