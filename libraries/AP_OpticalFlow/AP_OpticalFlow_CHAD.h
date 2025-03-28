#pragma once
#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_CHAD_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include "AP_OpticalFlow.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>




class AP_OpticalFlow_Chad : public OpticalFlow_backend
{	
    public :

        using OpticalFlow_backend::OpticalFlow_backend;

        CLASS_NO_COPY(AP_OpticalFlow_Chad);

        void init() override;

        void update() override;


    private :

        uint8_t buffer[15];

        float Vx;
        float Vy;
        float Vz;

        uint8_t Qual;	

        int socket_fd;


        struct sockaddr_in src_addr;
        socklen_t src_addr_len;

        struct sockaddr_in addr;


        int message_parser(uint8_t *message, float *Vx, float *Vy, float *Vz, uint8_t *Qual, char id);
};

#endif
