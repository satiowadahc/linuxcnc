/*
 *  ****************************************
 *  rmdteensyhal.cc - RMD Teensy Hal Driver
 *  COPYRIGHT (C) 2020 RMD Engineering Inc.
 *  Written By Chad Woitas
 *
 * *****************************************
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <signal.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include <unistd.h>
#include <stdarg.h>

#include <hal.h>
#include <inifile.hh>

//#include "config.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <cjson/cJSON.h>

const char *modname = "rmdteensy";
int hal_comp_id;
const char *section = "RMD-TEENSY";
bool simu_mode = false;

// All Hal pins for Teensy
typedef struct{
	hal_float_t *x1, *y1, *z1;
	hal_float_t *x2, *y2, *z2;
	hal_float_t *x3, *y3, *z3;
	hal_float_t *x4, *y4, *z4;
	hal_float_t *x5, *y5, *z5;
	hal_float_t *x6, *y6, *z6;
	hal_float_t *x7, *y7, *z7;
	hal_float_t *x8, *y8, *z8;
	hal_float_t *x9, *y9, *z9;
} teensy_hal_t;

// Teensy Object
typedef struct{
	teensy_hal_t *hal;

} teensy_t;

static teensy_t teensy;

//static int do_exit = 0;
//static int do_reconnect = 0;
//static bool wait_for_mcu_before_HAL = false;


//Not Sure what we'll need in here
//Serial port maybe?
extern "C" const char * iniFind(FILE *fp, const char *tag, const char *section){
    IniFile  f(false, fp);

    return(f.Find(tag, section));
}

static int hal_pin_simu(char *pin_name, void **ptr, int s){
	printf("Creating pin: %s\n", pin_name);
	*ptr = calloc(s, 1);
	return 0;
}

int _hal_pin_bit_newf(hal_pin_dir_t dir, hal_bit_t ** data_ptr_addr, int comp_id, const char *fmt, ...){
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, ( void**)data_ptr_addr, sizeof(*data_ptr_addr));
    }
    else {
    	return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

int _hal_pin_float_newf(hal_pin_dir_t dir, hal_float_t ** data_ptr_addr, int comp_id, const char *fmt, ...){
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, ( void**)data_ptr_addr, sizeof(*data_ptr_addr));
    }
    else {
    	return hal_pin_float_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

static void hal_setup(){
	int r;

	// Might not actually include in the simulator for now
	if (!simu_mode) {
		hal_comp_id = hal_init(modname);
		if (hal_comp_id < 1) {
			fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
			exit(1);
		}

		teensy.hal = (teensy_hal_t *)hal_malloc(sizeof(teensy_hal_t));
		if (teensy.hal == NULL) {
			fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
			exit(1);
		}
	}
	else {
		teensy.hal = (teensy_hal_t *)calloc(sizeof(teensy_hal_t), 1);
	}

	r = 0;

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x1), hal_comp_id, "%s.x1-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y1), hal_comp_id, "%s.y1-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z1), hal_comp_id, "%s.z1-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x2), hal_comp_id, "%s.x2-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y2), hal_comp_id, "%s.y2-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z2), hal_comp_id, "%s.z2-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x3), hal_comp_id, "%s.x3-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y3), hal_comp_id, "%s.y3-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z3), hal_comp_id, "%s.z3-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x4), hal_comp_id, "%s.x4-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y4), hal_comp_id, "%s.y4-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z4), hal_comp_id, "%s.z4-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x5), hal_comp_id, "%s.x5-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y5), hal_comp_id, "%s.y5-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z5), hal_comp_id, "%s.z5-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x6), hal_comp_id, "%s.x6-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y6), hal_comp_id, "%s.y6-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z6), hal_comp_id, "%s.z6-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x7), hal_comp_id, "%s.x7-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y7), hal_comp_id, "%s.y7-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z7), hal_comp_id, "%s.z7-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x8), hal_comp_id, "%s.x8-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y8), hal_comp_id, "%s.y8-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z8), hal_comp_id, "%s.z8-vector", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->x9), hal_comp_id, "%s.x9-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->y9), hal_comp_id, "%s.y9-vector", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(teensy.hal->z9), hal_comp_id, "%s.z9-vector", modname);

    return;
}


// Setting Serial Connection Parameters
int set_interface_attribs (int fd, int speed, int parity){
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0){
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0){
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0){
                printf ("error %d setting term attributes", errno);
        }
}


void read_until_newline(int fd, char* out, int outsize){
    int pos = 0;
	  char buf[512] = {0};
    int start = 0;
    int end = 0;
    bool start_string = false;
    bool end_string = false;

    //Find the string
    while(pos < outsize){
    	if(read(fd, buf+pos, 1)>0){
    	if(buf[pos]=='{'){
    		start_string=true;
    		start = pos;
    	}
    	if(buf[pos]=='}'){
    		end_string=true;
    		end = pos;
    		break;
    	}
			pos++;
			}
    }

    //Check we found a start and end
    if(!start_string || !end_string){
    	out[0]='\0';
      return;
    }
    //Copy to out string
    for(int i=start; i<=end; i++){
    	out[i-start]=buf[i];
    }
    out[end]='\0';

}


int main (int argc,char **argv){

	/*
	 * The Logic
	 * 0) Set Variables
	 * 1) Open Port
	 *
	 * 2) Setup Hal
	 * 3) Ready Hal
	 *
	 * 4) while !disconnect
	 * 	-> read the teensy
	 *  -> parse data
	 * 5) on disconnect signal
	 * 6) -close hal
	 * 7) -close serial
	 *
	 */

	// *********** Run Time Vars ***************
	bool hal_ready_done = false;
	char strbuf[512];
	char* intConvert;
	cJSON *json;
	const cJSON *sensReading = NULL;

	memset(&teensy, 0, sizeof(teensy));
	// *********** OPEN PORT  ******************
	//TODO Get from INI
	char portname[32] = "/dev/magSensor";

	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0){
			printf ("error %d opening %s: %s", errno, portname, strerror (errno));
			return -1;
	}
	else{printf("RMDTEENSY: Serial Opened successfully\n");}

	// ************ HAL Setup ******************
	hal_setup();
	printf("RMDTEENSY: Loaded Hal\n");
	// *********** HAL READY  ******************
    if (!hal_ready_done) {
    	hal_ready(hal_comp_id);
    	hal_ready_done = true;
	printf("RMDTEENSY: Hal ready\n");
    }

    // *********** MAIN LOOP *******************

    do {
      read_until_newline(fd, strbuf, sizeof(strbuf));
      // printf("\n TEENSYDBG1: main: %s \n", strbuf);
      json = cJSON_Parse(strbuf);
			if (json != NULL){
      	// printf("\n\nTEENSYDBG2: %s\n", cJSON_Print(json));

			  sensReading = cJSON_GetObjectItemCaseSensitive(json, "1-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x1) = atof(intConvert);
			  }

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "1-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y1) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "1-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z1) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "2-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x2) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "2-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y2) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "2-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z2) = atof(intConvert);
				}

				// sensReading = cJSON_GetObjectItemCaseSensitive(json, "3-x");
				// if(sensReading->valuestring != NULL){
				// 	printf("3-x Read:\n");
				// 	intConvert = sensReading->valuestring;
				// 	*(teensy.hal->x3) = atof(intConvert);
			  // }
				//
				// sensReading = cJSON_GetObjectItemCaseSensitive(json, "3-y");
				// if(sensReading->valuestring != NULL){
				// 	printf("3-y Read:\n");
				// 	intConvert = sensReading->valuestring;
				// 	*(teensy.hal->y3) = atof(intConvert);
				// }
				//
				// sensReading = cJSON_GetObjectItemCaseSensitive(json, "3-z");
				// if(sensReading->valuestring != NULL){
				// 	printf("3-z Read:\n");
				// 	intConvert = sensReading->valuestring;
				// 	*(teensy.hal->z3) = atof(intConvert);
				// }

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "4-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x4) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "4-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y4) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "4-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z4) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "5-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x5) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "5-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y5) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "5-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z5) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "6-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x6) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "6-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y6) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "6-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z6) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "7-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x7) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "7-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y7) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "7-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z7) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "8-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x8) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "8-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y8) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "8-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z8) = atof(intConvert);
				}


				sensReading = cJSON_GetObjectItemCaseSensitive(json, "9-x");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->x9) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "9-y");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->y9) = atof(intConvert);
				}

				sensReading = cJSON_GetObjectItemCaseSensitive(json, "9-z");
				if(sensReading->valuestring != NULL){
					intConvert = sensReading->valuestring;
					*(teensy.hal->z9) = atof(intConvert);
				}

		}
    	sleep(1);
    } while (1);

		hal_exit(hal_comp_id);
    // *********** Disconnect *******************
    // TODO: Shutdown
    return 0;
}
