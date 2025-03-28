#include <GCS_MAVLink/GCS.h>
#include "AP_OpticalFlow_CHAD.h"

#if AP_OPTICALFLOW_CHAD_ENABLED

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <GCS_MAVLink/GCS.h>
#include <cstring>


#include<iostream>


//AP_OpticalFlow_Chad* AP_OpticalFlow_Chad::detect(AP_OpticalFlow &_frontend)
//{
//	AP_OpticalFlow_Chad *sensor = new AP_OpticalFlow_Chad(_frontend);
//}



void AP_OpticalFlow_Chad::init() {

    socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    //timeout.tv_sec = 0;
    //timeout.tv_usec = 1;
    //setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&timeout, sizeof(timeout));

    src_addr = {};

    src_addr_len = sizeof(src_addr);

    addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr));
    addr.sin_port = htons(1106);

    bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr));

}




void AP_OpticalFlow_Chad::update(void) {

    int ret = recvfrom(socket_fd, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr*)(&src_addr), &src_addr_len);

    if (ret <= 0) {
        return;
    }
    
    ret = message_parser(buffer, &Vx, &Vy, &Vz, &Qual, '@');
     
    if (ret != 0) {
	    return;
    }

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Speed values : (%f, %f)", Vx, Vy);

    struct AP_OpticalFlow::OpticalFlow_state state {};

    state.surface_quality = Qual;
    state.flowRate.x = Vx;
    state.flowRate.y = Vy;
    state.flow_Vz = Vz;
    
    //TODO Add rotatuion of the camera
    state.bodyRate.x = Vx;
    state.bodyRate.y = Vy;
    state.body_Vz = Vz;

    _update_frontend(state);

}


int AP_OpticalFlow_Chad::message_parser(uint8_t* message, float *Vx_, float *Vy_, float *Vz_, uint8_t *Qual_, char id) {
	char id_ = (char)message[0];
	
	if (id != id_) {
	   return 1; //Wrong id
	}

	union {
		float f;
		char bytes[sizeof(float)];
	} convert;


	memcpy(convert.bytes, message + 1, sizeof(float));
	*Vx_ = convert.f;

	memcpy(convert.bytes, message + 5, sizeof(float));
	*Vy_ = convert.f;
	
	memcpy(convert.bytes, message + 9, sizeof(float));
	*Vz_ = convert.f;

	*Qual_ = message[13];

	
	return 0;
}

#endif
