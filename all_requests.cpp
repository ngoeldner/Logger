// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <stdint.h>

#include <time.h>
#include <sys/time.h>
#include <math.h>

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


#include <stdint.h>

struct XYAngle {
  int16_t angleX;
  int16_t angleY;
  int16_t heading;
};

struct IMUValues {
  int16_t accX;
  int16_t accY;
  int16_t accZ;

  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

};

struct RCInput {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
};

struct BatInput {
  int8_t v;
  int16_t pMeterSum;
  int16_t rssi;
  int16_t amperage;
};

struct MotorValues {
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
  int16_t motor4;
};

struct GPSValues {
  uint8_t hasFix;
  uint8_t satNumber;
  uint32_t lat;
  uint32_t  lng;
  uint16_t altitude;
  uint16_t groundSpeed;
};


static enum _serial_state {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD,
}
c_state;


int serial_port;

void request(int code){
    uint8_t checksum = 0;

    unsigned char msg[] = { '$', 'M', '<'};
    // unsigned char msg_sent[10];
    write(serial_port, msg, 3);
    // msg_sent[0] = '$';
    // msg_sent[1] = 'M';
    // msg_sent[2] = 'C';

    unsigned char zero[1];
    zero[0] = 0;
    write(serial_port, zero, 1);
    checksum ^= 0;
    // printf("zero:%c\n", zero[0]);

    char opcode[1];
    opcode[0] = char(code);

    write(serial_port, opcode,1 );

    // printf("opcode:%c\n", opcode[0]);

    checksum ^= code;

    char check_sum_c[1];
    check_sum_c[0] = char(checksum);
    // printf("check_sum_c:%c\n", check_sum_c[0]);

    write(serial_port, check_sum_c, 1);
    // printf("sent\n");
}


int16_t readInt16(uint8_t * inBuf,int index) {
  return  (inBuf[index*2+1] << 8) | inBuf[index*2];
}

XYAngle evalAtt(uint8_t * inBuf) {

    int16_t angx = readInt16(inBuf,0);
    int16_t angy = readInt16(inBuf,1);
    int16_t angle = readInt16(inBuf,2);


    XYAngle angles;
    return angles = {angx, angy, angle};
}

IMUValues evalIMU(uint8_t * inBuf) {

  IMUValues result = {readInt16(inBuf,0), readInt16(inBuf,1), readInt16(inBuf,2), readInt16(inBuf,3), \
  readInt16(inBuf,4), readInt16(inBuf,5)};
  return result;

}


RCInput evalRC(uint8_t inBuf[]) {

  int16_t roll = readInt16(inBuf,0);
  int16_t pitch = readInt16(inBuf,1);
  int16_t yaw = readInt16(inBuf,2);
  int16_t throttle = readInt16(inBuf,3);

  RCInput result = {roll, pitch, yaw, throttle};

  return result;

}

BatInput evalBat(uint8_t inBuf[]) {
  BatInput result;
  int index = 0;
  result.v = inBuf[index]; index++;
  result.pMeterSum = (inBuf[index+1] << 8) | inBuf[index]; index++; index++;
  result.rssi = (inBuf[index+1] << 8) | inBuf[index]; index++; index++;
  result.amperage = (inBuf[index+1] << 8) | inBuf[index];
  return result;
}

void logXYAngle(XYAngle angles) {
    printf(" : %d %d %d", angles.angleX, angles.angleY, angles.heading);
}

void logIMU(IMUValues imu) {
    printf(" : %d %d %d %d %d %d", imu.accX, imu.accY, imu.accZ, imu.gyroX, imu.gyroY, imu.gyroZ);
}

void logRC(RCInput rc) {
    printf(" : %d %d %d %d", rc.roll, rc.pitch, rc.yaw, rc.throttle);
}

void logBat(BatInput bat) {
    printf(" : %d %d %d %d", bat.v, bat.pMeterSum, bat.rssi, bat.amperage);
}

void logTime(int it){
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long micro = te.tv_sec*1000000LL + te.tv_usec; // calculate milliseconds
    printf("it : %d ; microseconds : %lld\n",it, micro);
}

// void logTime2(){
//   struct timespec ts_start;
//   struct timespec ts_end;

//   clock_gettime(CLOCK_MONOTONIC, &ts_start);

//   printf("%ld\n", ts_start.tv_nsec);
//   // ts_end = ts_start;

// }


#define MAX_SIZE 50

int dataSize, checksum, offset,code;

void read(){

    uint8_t inBuf[100];

    char read_buf [100];

    memset(&read_buf, '\0', sizeof(read_buf));

    while (true) {

        char c_read;
        int num_bytes = read(serial_port, &c_read, 1);
        // printf("c_state = %d\n", c_state);
        // printf("c_read = %c\n", c_read);
        
        if (c_state == IDLE) {
            c_state = (c_read=='$') ? HEADER_START : IDLE;
        }
        else if (c_state == HEADER_START) {
            c_state = (c_read=='M') ? HEADER_M : IDLE;
        }
        else if (c_state == HEADER_M) {
            c_state = (c_read=='>') ? HEADER_ARROW : IDLE;
        }
        else if (c_state == HEADER_ARROW) {

            if ((int)c_read > MAX_SIZE) {  // now we are expecting the payload size
                c_state = IDLE;

            }
            else {
                dataSize = (int)c_read;
                offset = 0;
                checksum = 0;
                checksum ^= dataSize;
                c_state = HEADER_SIZE;
            }
        }
        else if (c_state == HEADER_SIZE) {
            code = (int)c_read;
            checksum ^= code;
            c_state = HEADER_CMD;
        }
        else if (c_state == HEADER_CMD && offset < dataSize) {
            checksum ^= c_read;
            inBuf[offset++] = c_read;
        }
        else if (c_state == HEADER_CMD && offset >= dataSize) {

            if (checksum == c_read) {
                // printf("Read %i bytes. Received message: (%c)(%c)(%c)(%d)(%d)\n",dataSize+6, '$', 'M', '>',dataSize,code);
                if (code == MSP_ATTITUDE) {
                    XYAngle result = evalAtt(inBuf);
                    logXYAngle(result);
                }
                if (code == MSP_RAW_IMU) {
                    IMUValues result = evalIMU(inBuf);
                    logIMU(result);
                }
                if (code == MSP_RC) {
                    RCInput result = evalRC(inBuf);
                    logRC(result);
                }
                if (code == MSP_ANALOG) {
                    BatInput result = evalBat(inBuf);
                    logBat(result);
                }
            }

            c_state = IDLE;
        }

        if (c_state == IDLE) {break;}

    }
}




int main() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Write to serial port
  int it=0;
  while(1){

    struct timespec tim, tim2;
    tim.tv_sec = 1;
    // tim.tv_nsec = 0L*1000*1000*100;
    tim.tv_nsec = 1000*1000*250;
    nanosleep(&tim , NULL); //NULL or &tim2?

    logTime(it++);
    printf("Angles");
    request(MSP_ATTITUDE);
    read();
    printf("\n");
    printf("IMU");
    request(MSP_RAW_IMU);
    read();
    printf("\n");
    printf("RC");
    request(MSP_RC);
    read();
    printf("\n");
    printf("Bat");
    request(MSP_ANALOG);
    read();
    printf("\n\n");
    fflush(stdout);

  }


  close(serial_port);
  return 0; // success
}
