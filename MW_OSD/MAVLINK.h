
/*
NAZA for reference:
 
 void NAZA_NewData(uint8_t c){
 uint8_t decodedMessage = NazaDecoder.decode(c);
 int8_t sattemp;
 switch (decodedMessage){
 uint8_t GPS_fix_temp;
 case NAZA_MESSAGE_GPS:
 sattemp=NazaDecoder.getNumSat();
 if (sattemp>4){
 #ifdef GPSACTIVECHECK
 timer.GPS_active=GPSACTIVECHECK;
 #endif //GPSACTIVECHECK
 GPS_numSat=NazaDecoder.getNumSat();
 GPS_coord[LAT]=(int32_t)(10000000*NazaDecoder.getLat());
 GPS_coord[LON]=(int32_t)(10000000*NazaDecoder.getLon());
 GPS_altitude=NazaDecoder.getGpsAlt();
 GPS_fix_temp=NazaDecoder.getFixType();
 GPS_numSat=NazaDecoder.getNumSat();
 GPS_speed=100*NazaDecoder.getSpeed();
 gpsvario();            
 if (GPS_fix_temp>0){
 GPS_fix=1;
 }
 GPS_NewData();
 }            
 break;
 case NAZA_MESSAGE_COMPASS:
 GPS_ground_course=10*NazaDecoder.getHeadingNc();
 int16_t MwHeading360=GPS_ground_course/10;
 if (MwHeading360>180)
 MwHeading360 = MwHeading360-360;
 MwHeading   = MwHeading360;
 break;
 }
 }
 */

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_MAGIC 50
#define AVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_VFR_HUD_MAGIC 20
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_MAGIC 39
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC 244
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22    
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_MAGIC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31  

uint8_t mav_message_length;
uint8_t mav_message_cmd;
uint16_t mav_serial_checksum;



void serialMAVCheck(){
#ifdef MSPACTIVECHECK
  timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
#endif
  switch(mav_message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    break;
  case MAVLINK_MSG_ID_VFR_HUD:
    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    break;
  case MAVLINK_MSG_ID_GPS_RAW_INT:
#ifdef GPSACTIVECHECK
    timer.GPS_active=GPSACTIVECHECK;
#endif //GPSACTIVECHECK
    //    GPS_lat =serialBuffer[8]+serialBuffer[9]<<8+serialBuffer[10]<<16+serialBuffer[11]<<32;
    //    GPS_lon =serialBuffer[12]+serialBuffer[13]<<8+serialBuffer[14]<<16+serialBuffer[15]<<32;
    GPS_speed=serialBuffer[24]+serialBuffer[25]<<8;
    GPS_fix=serialBuffer[28];
    GPS_numSat=serialBuffer[29];
    //   GPS_fix=3;
    //   GPS_numSat=9;
    debug[0]=GPS_speed;
    debug[1]=GPS_fix;
    debug[2]=GPS_numSat;
    
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    //debug[0]=serialBuffer[11];
    //debug[1]=serialBuffer[12];
    //debug[2]=serialBuffer[1];
    //debug[3]++;

    MwVBat=serialBuffer[11]+(serialBuffer[12]<<8)/100;
    break;
  }
}


void mav_checksum(uint8_t val) {
  uint16_t tmp;
  tmp = val ^ mav_serial_checksum &0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mav_serial_checksum = (mav_serial_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


void serialMAVreceive(uint8_t c)
{
  static uint8_t mav_payload_index; 
  static uint16_t mav_checksum_rcv; 

  static enum _serial_state {
    MAV_IDLE,
    MAV_HEADER_START,
    MAV_HEADER_LEN,
    MAV_HEADER_SEQ,
    MAV_HEADER_SYS,
    MAV_HEADER_COMP,
    MAV_HEADER_MSG,
    MAV_PAYLOAD,
    MAV_CHECKSUM,
  }

  mav_state = MAV_IDLE;

  if ((mav_state == MAV_IDLE)||(mav_state == MAV_PAYLOAD))
  {
  }
  else
  {
    mav_checksum(c);
  }

  if (mav_state == MAV_IDLE)
  {
    if (c==0xFE)
    {
      mav_serial_checksum=0xFFFF;
      mav_payload_index=0;
      mav_state = MAV_HEADER_START;
    }
    else
    {
      mav_state = MAV_IDLE;
    } 
  }
  else if (mav_state == MAV_HEADER_START)
  {
    mav_message_length = c;
    mav_state = MAV_HEADER_LEN;
    if ((mav_payload_index) > SERIALBUFFERSIZE){  // too much data so reset check
      mav_state = MAV_IDLE;
    }
  }
  else if (mav_state == MAV_HEADER_LEN)
  {
    mav_state = MAV_HEADER_SEQ;
  }
  else if (mav_state == MAV_HEADER_SEQ)
  {
    mav_state = MAV_HEADER_SYS;
  }
  else if (mav_state == MAV_HEADER_SYS)
  {
    mav_state = MAV_HEADER_COMP;
  }
  else if (mav_state == MAV_HEADER_COMP)
  {
    mav_message_cmd = c;
    mav_state = MAV_HEADER_MSG;
  }
  else if (mav_state == MAV_HEADER_MSG) // header received, its a packet!
  {
    serialBuffer[mav_payload_index]=c;
    mav_payload_index++;
    if (mav_payload_index==mav_message_length){  // end of data
      mav_state = MAV_PAYLOAD;
    }

  }
  else if (mav_state == MAV_PAYLOAD)
  {
    if (mav_payload_index==mav_message_length){  // CKbyte1
      mav_checksum_rcv=c;
      mav_payload_index++;
    }
    else{
      mav_checksum_rcv+=(c<<8);
      int8_t mav_magic;
      switch(mav_message_cmd) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mav_magic = MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
        break;
      case MAVLINK_MSG_ID_VFR_HUD:
        mav_magic = MAVLINK_MSG_ID_VFR_HUD_MAGIC;
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        mav_magic = MAVLINK_MSG_ID_ATTITUDE_MAGIC;
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mav_magic = MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
        break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        mav_magic = MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        mav_magic = MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
        break;
      }

      mav_checksum(mav_magic);
      if(mav_checksum_rcv == mav_serial_checksum) {
        serialMAVCheck(); // valid packet go MAVshit
      }
      mav_state = MAV_IDLE;
    }
  }
}












