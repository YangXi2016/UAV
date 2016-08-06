#include <Wire.h>


/*
  Name:    Arduino_camera.ino
  Created: 7/4/2016 2:07:26 PM
  Author:  XiYang
*/

#include <PixyI2C.h>

typedef unsigned char u8;

#define Debug 0


//#define  IIR_ORDER     4
//static double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};
//static double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};
#define  IIR_ORDER     1


int data[8] = {0};
long begin_time = millis();
static int i = 0;
uint16_t blocks;
//char buf[320];
char inByte = 0;

//int frames=0;
static double b_IIR[IIR_ORDER + 1] = {0.0500f, 0.0000f};
static double a_IIR[IIR_ORDER + 1] = {0.0000f, -0.9500f};

static double InPut_IIR[4][IIR_ORDER + 1] = {0};
static double OutPut_IIR[4][IIR_ORDER + 1] = {0};

PixyI2C pixy;
// PixyI2C pixy(0x55); // You can set the I2C address through PixyI2C object

int send_flag = 0;



typedef struct
{
  float average;//Flow in m in x-sensor direction, angular-speed compensated
  float originf;
  int16_t origin;
} FLOW_DATA;


typedef struct
{
  uint64_t  time_sec;
  u8   id;
  FLOW_DATA flow_x;
  FLOW_DATA flow_y;
  FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
  FLOW_DATA flow_comp_y;
  u8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
  FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance
} FLOW;

typedef struct
{
  uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
  uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
  float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
  float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
  float integrated_xgyro; ///< RH rotation around X axis (rad)
  float integrated_ygyro; ///< RH rotation around Y axis (rad)
  float integrated_zgyro; ///< RH rotation around Z axis (rad)
  uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
  float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
  int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
  uint8_t sensor_id; ///< Sensor ID
  uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
} FLOW_RAD;

typedef struct
{
  FLOW_DATA flow_x;
  FLOW_DATA flow_y;
  FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
  FLOW_DATA flow_comp_y;
  float scale_rad_fix;
  float scale_rad_fix_comp;
  FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance
} FLOW_FIX;


FLOW flow;
FLOW_RAD flow_rad;
FLOW_FIX flow_fix;
u8 FLOW_STATE[4];
u8 flow_buf[27];
u8 flow_buf_rad[45];

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  pixy.init();

}





float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}


double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1, z2;
  short i;
  double OutData;

  for (i = nb - 1; i > 0; i--)
  {
    x[i] = x[i - 1];
  }

  x[0] = InData;

  for (z1 = 0, i = 0; i < nb; i++)
  {
    z1 += x[i] * b[i];
  }

  for (i = na - 1; i > 0; i--)
  {
    y[i] = y[i - 1];
  }

  for (z2 = 0, i = 1; i < na; i++)
  {
    z2 += y[i] * a[i];
  }

  y[0] = z1 - z2;
  OutData = y[0];

  return OutData;
}

void FLOW_MAVLINK(unsigned char data)
{


  static u8 s_flow = 0, data_cnt = 0;
  u8 cnt_offset = 0;
  u8 get_one_fame = 0;
  //char floattobyte[4];
  unsigned char floattobyte[4];


  switch (s_flow)
  {
    case 0: if (data == 0xFE)
        s_flow = 1;
      break;
    case 1: if (data == 0x1A || data == 0x2C)
      {
        s_flow = 2;
      }
      else
        s_flow = 0;
      break;
    case 2:
      if (data_cnt < 4)
      {
        s_flow = 2;
        FLOW_STATE[data_cnt++] = data;
      }
      else
      {
        data_cnt = 0;
        s_flow = 3;
        flow_buf[data_cnt++] = data;
      }
      break;
    case 3:
      if (FLOW_STATE[3] == 100) {
        if (data_cnt < 26)
        {
          s_flow = 3;
          flow_buf[data_cnt++] = data;
        }
        else
        {
          data_cnt = 0;
          s_flow = 4;
        }
      }
      else if (FLOW_STATE[3] == 106) {
        if (data_cnt < 44)
        {
          s_flow = 3;
          flow_buf_rad[data_cnt++] = data;
        }
        else
        {
          data_cnt = 0;
          s_flow = 4;
        }
      }
      else
      {
        data_cnt = 0;
        s_flow = 0;
      }
      break;
    case 4: get_one_fame = 1; s_flow = 0; data_cnt = 0; break;
    default: s_flow = 0; data_cnt = 0; break;
  }//--end of s_uart


  if (get_one_fame)
  {
    if (FLOW_STATE[3] == 100) {
      flow.time_sec = (flow_buf[7] << 64) | (flow_buf[6] << 56) | (flow_buf[5] << 48) | (flow_buf[4] << 40) | (flow_buf[3] << 32) | (flow_buf[2] << 16) | (flow_buf[1] << 8) | (flow_buf[0]);
      floattobyte[0] = flow_buf[8];
      floattobyte[1] = flow_buf[9];
      floattobyte[2] = flow_buf[10];
      floattobyte[3] = flow_buf[11];
      flow.flow_comp_x.originf = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf[12];
      floattobyte[1] = flow_buf[13];
      floattobyte[2] = flow_buf[14];
      floattobyte[3] = flow_buf[15];
      flow.flow_comp_y.originf = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf[16];
      floattobyte[1] = flow_buf[17];
      floattobyte[2] = flow_buf[18];
      floattobyte[3] = flow_buf[19];
      flow.hight.originf = ByteToFloat(floattobyte); //ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance
      flow.flow_x.origin = (int16_t)((flow_buf[20]) | (flow_buf[21] << 8));
      flow.flow_y.origin = (int16_t)((flow_buf[22]) | (flow_buf[23] << 8));
      flow.id = flow_buf[24];
      flow.quality = flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
      flow_fix.flow_x.average = IIR_I_Filter(flow_fix.flow_x.origin , InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
      //   flow_fix.flow_y.average = IIR_I_Filter(flow_fix.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
      flow.flow_y.average      = IIR_I_Filter(flow.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
      flow.flow_comp_x.average = IIR_I_Filter(flow.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
      flow.flow_comp_y.average = IIR_I_Filter(flow.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
      //   flow_fix.flow_comp_x.average = IIR_I_Filter(flow_fix.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
      // flow_fix.flow_comp_y.average = IIR_I_Filter(flow_fix.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
      send_flag = 1;
    }
    else if (FLOW_STATE[3] == 106)
    {
      flow_rad.time_usec = (flow_buf_rad[7] << 64) | (flow_buf_rad[6] << 56) | (flow_buf_rad[5] << 48) | (flow_buf_rad[4] << 40)
                           | (flow_buf_rad[3] << 32) | (flow_buf_rad[2] << 16) | (flow_buf_rad[1] << 8) | (flow_buf_rad[0]);
      flow_rad.integration_time_us = (flow_buf_rad[11] << 32) | (flow_buf_rad[10] << 16) | (flow_buf_rad[9] << 8) | (flow_buf_rad[8]);
      floattobyte[0] = flow_buf_rad[12];
      floattobyte[1] = flow_buf_rad[13];
      floattobyte[2] = flow_buf_rad[14];
      floattobyte[3] = flow_buf_rad[15];
      flow_rad.integrated_x = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf_rad[16];
      floattobyte[1] = flow_buf_rad[17];
      floattobyte[2] = flow_buf_rad[18];
      floattobyte[3] = flow_buf_rad[19];
      flow_rad.integrated_y = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf_rad[20];
      floattobyte[1] = flow_buf_rad[21];
      floattobyte[2] = flow_buf_rad[22];
      floattobyte[3] = flow_buf_rad[23];
      flow_rad.integrated_xgyro = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf_rad[24];
      floattobyte[1] = flow_buf_rad[25];
      floattobyte[2] = flow_buf_rad[26];
      floattobyte[3] = flow_buf_rad[27];
      flow_rad.integrated_ygyro = ByteToFloat(floattobyte);
      floattobyte[0] = flow_buf_rad[28];
      floattobyte[1] = flow_buf_rad[29];
      floattobyte[2] = flow_buf_rad[30];
      floattobyte[3] = flow_buf_rad[31];
      flow_rad.integrated_zgyro = ByteToFloat(floattobyte);
      flow_rad.time_delta_distance_us = (flow_buf_rad[35] << 32) | (flow_buf_rad[34] << 16) | (flow_buf_rad[33] << 8) | (flow_buf_rad[32]);
      floattobyte[0] = flow_buf_rad[36];
      floattobyte[1] = flow_buf_rad[37];
      floattobyte[2] = flow_buf_rad[38];
      floattobyte[3] = flow_buf_rad[39];
      flow_rad.distance = ByteToFloat(floattobyte);
      flow_rad.temperature = (flow_buf_rad[41] << 8) | (flow_buf_rad[40]);
      flow_rad.sensor_id = (flow_buf_rad[42]);
      flow_rad.quality == (flow_buf_rad[43]);

      flow_fix.flow_x.origin = flow.flow_x.origin + flow_rad.integrated_ygyro * flow_fix.scale_rad_fix;
      flow_fix.flow_y.origin = flow.flow_y.origin - flow_rad.integrated_xgyro * flow_fix.scale_rad_fix;
      flow_fix.flow_comp_x.originf = flow.flow_comp_x.originf - flow_rad.integrated_ygyro * flow_fix.scale_rad_fix_comp;
      flow_fix.flow_comp_y.originf = flow.flow_comp_y.originf + flow_rad.integrated_xgyro * flow_fix.scale_rad_fix_comp;
      send_flag = 2;
    }
  }
}

int limit(int velocity)
{
  if (velocity > 127) {
    velocity = 127;
  }
  else if (velocity < -127) {
    velocity = -127;
  }
  velocity += 127;
  return velocity;
}




//long positions[4][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
//long old_positions[4][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
//long new_positions[4][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
bool previous_status[6]={0,0,0,0,0,0};
unsigned char rec_data;
long last_time = micros();
int times=1;
void loop()
{
  
    if (Serial.available() > 0) {
    while(Serial.read() >= 0){}
    //Serial.print(inByte);

      for (int j = 0; j < 8; j++) {
      //  Serial.print(data[j]);
        Serial.write(data[j]);
      }
    }
  
  //Serial.println(micros()-last_time);
  //last_time=micros();
  blocks = pixy.getBlocks();
  
if(times%3==0){
  times=1;
    for(int i=0;i<6;i++){
    data[i]=0;
    }
}else{
  times+=1;
}
  
  if (blocks)
  {
    /*long sum_x[4] = { 0, 0, 0, 0 };
    long sum_y[4] = { 0, 0, 0, 0 };
    long weight[4] = { 0, 0, 0, 0 };
    long sum_weight[4] = { 0, 0, 0, 0 };*/
    //begin_time = micros();
    for (int j = 0; j < blocks; j++)
    {
      //sprintf(buf, "  block %d: ", j);
      //pixy.blocks[j].print();
      int signature = pixy.blocks[j].signature - 1;
      /*weight[signature] = pixy.blocks[j].width * pixy.blocks[j].height;
      sum_x[signature] += pixy.blocks[j].x * weight[signature];
      sum_y[signature] += pixy.blocks[j].y * weight[signature];
      sum_weight[signature] += weight[signature];*/
      data[signature]=1;
      }
    }



    while(Serial1.read() >= 0){}  
    while(send_flag!=1)
    {
    if(Serial1.available())
    {
      unsigned char rec_data=Serial1.read();
      FLOW_MAVLINK(rec_data);
    }
    }
    send_flag=0;

    data[6] = limit(int(flow.flow_comp_x.average * 100));
    data[7] = limit(int(flow.flow_comp_y.average * 100));
    //delay(20);

}



/*
void serialEvent() {
  while (Serial.read() >= 0) {}
  for (int j = 0; j < 8; j++) {
    //  Serial.print(data[j]);
    Serial.write(data[j]);

  }
}*/
