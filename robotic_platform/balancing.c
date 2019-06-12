#include  <wiringPiI2C.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <sys/time.h>
#include <signal.h>

#define PID 0
#define NUM_STATES 256
#define EPISODE 50

double Kp = 2.5; //2.5;   // 2.5
double Ki = 0.8; //0.8;   // 1.0
double Kd = 6.0; //6.0;   // 8.0
double K  = 1.7;


double K0 = (double) 0.98;
double K1 = (double) 0.02;

int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;


double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;

struct timeval tv, tv2;
unsigned long long      timer, t;

double deltaT;


int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}

double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

void read_all()
{
    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    gyroX = read_word_2c(0x43);
    gyroY = read_word_2c(0x45);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / 131.0;
    gyro_scaled_y = gyroY / 131.0;
    gyro_scaled_z = gyroZ / 131.0;
}

unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}


double constrain(double v, double min_v, double max_v)
{
  if (v <= min_v)
    return (double)min_v;
  else if (v >= max_v)
    return (double)max_v;
  else
    return (double)v;
}

double GUARD_GAIN = 100.0;
double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 0; //-0.5;  //1.5

double speed;

double q_angle_offset = 45;
double q_angle_velocity_offset = 200;
double learning_rate = 0.99;
double gamma_q = 0.99;

void pid()
{
  error = last_x - angle_offset;

  pTerm = Kp * error;

  integrated_error = 0.95*integrated_error + error;
  iTerm = Ki * integrated_error;
  
  dTerm = Kd * (error - last_error);
  last_error = error;

  speed = constrain(K*(pTerm + iTerm + dTerm), -GUARD_GAIN, GUARD_GAIN); 
  
}

int discrete_state(double theta, double theta_dot)
{
   int sign_t = 0;
   if ((theta >= q_angle_offset) || (theta <= -q_angle_offset))
   {
      return NUM_STATES;
   }

   int state = 0;
   if (theta < 0)
   {
      sign_t = 1;
      theta = -theta;
   }

   state = floor(8*theta/(q_angle_offset));

   state = sign_t ? state*2 : (state*2)+1;

   state = state << 4;

   sign_t = 0;
   if (theta_dot < 0)
   {
      sign_t = 1;
      theta_dot = -theta_dot;
   }

   int state_angular_velocity = floor(8*theta_dot/(q_angle_velocity_offset));
   state += sign_t ? state_angular_velocity*2 : state_angular_velocity*2+1;

   return state;   
}


FILE *fp;

void intHandler(void) {
  fclose(fp);
  printf("CLOSED\n");
  exit(0);
}

int main()
{
  init_motors();
  delay(200);
  signal(SIGINT, intHandler);

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd,0x6B,0x00);//disable sleep mode 
  printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

  for (int i=0; i<50; i++)
  {
     read_all();
     delay(100);
     gyro_offset_x += gyro_scaled_x;
     gyro_offset_y += gyro_scaled_y;
  }

  gyro_offset_x /= 50.0;
  gyro_offset_y /= 50.0;

  timer = getTimestamp();

  deltaT = (double) (getTimestamp() - timer)/1000000.0;

  last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


  gyro_offset_x = gyro_scaled_x;
  gyro_offset_y = gyro_scaled_y;

  gyro_total_x = last_x - gyro_offset_x;
  gyro_total_y = last_y - gyro_offset_y;

  float Q[NUM_STATES][2];
  float Q_prime[NUM_STATES][2]={0};

  for (int i = 0; i < NUM_STATES; i++)
  {
    Q[i][0] = 1;
    Q[i][1] = 1;
  }
  Q[NUM_STATES][0] = 1; // Terminal state
  Q[NUM_STATES][1] = 1;

  int a = 0;

  int state = 0; //discrete_state(last_x, gyro_scaled_x);

  
  int timeout = 0;
  int s_prime;
  int a_prime;
  int epsode_cnt = 0;
  double reward = 0.0;
  int time_stamp = 0;
  int episode_start = 1;
  int delay_cnt = 0;


  fp = fopen("test.txt", "w+");
  fprintf(fp, "This is testing for fprintf...\n");

  while(1) {
    timeout++;	  

    t = getTimestamp();
    deltaT = (double) (t - timer)/1000000.0;
    timer = t;

    read_all();

    gyro_scaled_x -= gyro_offset_x;
    gyro_scaled_y -= gyro_offset_y;

    gyro_x_delta = (gyro_scaled_x * deltaT);
    gyro_y_delta = (gyro_scaled_y * deltaT);

    gyro_total_x += gyro_x_delta;
    gyro_total_y += gyro_y_delta;

    rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


    last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
    last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);


    //printf("state = %d state_s= %d gyro_scaled=%f\n", discrete_state(last_x, gyro_scaled_x), state, gyro_scaled_x); 
    //if (last_x < -40.0 || last_x > 40.0)
    //{
    //  stop_motors();
    //  break;
    //}
    pid();
    s_prime = discrete_state(last_x, gyro_scaled_x);
    //printf("%f, %f\n", last_x, gyro_scaled_x);
    if ((s_prime == NUM_STATES) && (state == NUM_STATES) || delay_cnt)
    {
        delay(15);
	state = s_prime;
	stop_motors();
	episode_start = 1;
	if (delay_cnt)
	  delay_cnt--;
	continue;
    }

    /*
    if(episode_start && (state == NUM_STATES))
    {
      //state = 0;
      delay(15);
      state = s_prime;
      episode_start = 0;
      continue;
    }*/

    if (state == NUM_STATES)
      reward = -1.0;
    else
      reward = 0.0;


    time_stamp++;    
/*
    double sum_0 = 0;
    double sum_1 = 0;
    for (int i = 0; i<=NUM_STATES; i++)
    {
       sum_0 += Q[i][0];
       sum_1 += Q[i][1];
    }
*/
    if (timeout > 0)
    {   
    	a_prime = Q[s_prime][0] > Q[s_prime][1] ? 0 : 1; 
    	printf("Q learning s_prime=%d a=%d\n", s_prime, a_prime);
        //a_prime = sum_0 > sum_1 ? 0 : 1;
    }
    else
    { 
	printf("PID\n");
    	a_prime = speed > 0 ? 1 : 0;
    }
    double next_q = Q[s_prime][a_prime];
    if (epsode_cnt < EPISODE){
       // legacy
       //Q[state][a] = Q[state][a] + learning_rate*(reward + gamma_q*next_q - Q[state][a]);
       Q_prime[state][a] = learning_rate*(reward + gamma_q*next_q - Q[state][a]);
    }

    if (state == NUM_STATES)
    {
       //Q[state][a] = Q[state][a] + learning_rate*(reward - Q[state][a]);
       if (epsode_cnt < EPISODE)
       {
	  for (int i = 0; i <= NUM_STATES; i++)
       	  {
             Q[i][0] += Q_prime[i][0];
             Q[i][1] += Q_prime[i][1];
          }
	  for (int i = 0; i <= NUM_STATES; i++)
       	  {
             Q_prime[i][0] = 0;
             Q_prime[i][1] = 0;
          }
       }
       else
       {
          printf("Learning ended\n");
       }
       
       printf("Episode ended at %d\n", time_stamp);
       fprintf(fp, "%d\n", time_stamp);
       time_stamp = 0;
       state = s_prime;
       stop_motors();
       delay(15);
       delay_cnt = 100;
       epsode_cnt++;
       continue;
    }

    state = s_prime;
    a = a_prime;
    if (a)
      speed = 10;
    else
      speed = -10;

    motors(speed, 0.0, 0.0);
/*
    for (int i = 0; i <= 256; i++)
       printf("%f, ", Q[i][0]);
    printf("\n");
    for (int i = 0; i <= 256; i++)
       printf("%f, ", Q[i][1]);
    printf("\n");
*/
    delay(15); // delay
  }

  fclose(fp);

  init_motors();
  delay(200);
  return 0;
}
