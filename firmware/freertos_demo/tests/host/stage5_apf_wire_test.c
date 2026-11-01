/* QEMU-only embedded APF preview contract. No motor command or actuation.
 * C protocol.h is authoritative, not a Python redefinition of wire bits. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "protocol.h"

static unsigned int failures=0U;
#define CHECK(x) do { if(!(x)) { \
    fprintf(stderr,"failed line %d: %s\\n",__LINE__,#x); failures++; \
} } while(0)

static void set_i16(uint8_t *dst, int16_t value)
{
    uint16_t v=(uint16_t)value;
    dst[0]=(uint8_t)v;
    dst[1]=(uint8_t)(v>>8U);
}
static void set_float(uint8_t *dst, float value)
{
    uint32_t v;
    memcpy(&v,&value,4U);
    dst[0]=(uint8_t)v; dst[1]=(uint8_t)(v>>8U);
    dst[2]=(uint8_t)(v>>16U); dst[3]=(uint8_t)(v>>24U);
}
int main(void)
{
    protocol_frame_t frame={0};
    protocol_apf_step_request_t request;
    uint8_t response[PROTOCOL_APF_STEP_RESPONSE_FRAME_LEN]={0};
    uint8_t original[sizeof(request)];
    float values[6]={1.0F,50000.0F,150.0F,12.0F,10.0F,2.0F};
    float result[3]={101.25F,-202.5F,303.75F};
    size_t i;
    frame.command=CMD_REQUEST_APF_STEP;
    frame.length=PROTOCOL_APF_STEP_REQUEST_PAYLOAD_LEN;
    frame.payload[0]=57U;
    for(i=0U;i<9U;i++)
        set_i16(frame.payload+1U+2U*i,(int16_t)((int)i*10-60));
    for(i=0U;i<6U;i++)
        set_float(frame.payload+19U+4U*i,values[i]);
    memset(&request,0x5a,sizeof(request));
    CHECK(protocol_parse_apf_step_request(&frame,&request)==ROBOT_STATUS_OK);
    CHECK(request.sequence==57U);
    CHECK(request.goal_offset_mm[0]==-60);
    CHECK(request.obstacle_min_offset_mm[1]==-20);
    CHECK(request.obstacle_max_offset_mm[2]==20);
    for(i=0U;i<6U;i++) CHECK(request.apf_parameters[i]==values[i]);

    memcpy(original,&request,sizeof(request));
    frame.length--;
    CHECK(protocol_parse_apf_step_request(&frame,&request)==
          ROBOT_STATUS_ERROR_INVALID_LENGTH);
    CHECK(memcmp(original,&request,sizeof(request))==0);
    frame.length++;
    set_float(frame.payload+19U, NAN);
    CHECK(protocol_parse_apf_step_request(&frame,&request)==
          ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(memcmp(original,&request,sizeof(request))==0);

    CHECK(protocol_build_apf_step_response(57U,ROBOT_STATUS_OK,result,response)
          ==ROBOT_STATUS_OK);
    CHECK(response[0]==PROTOCOL_HEADER_0 && response[1]==PROTOCOL_HEADER_1);
    CHECK(response[2]==CMD_APF_STEP_RESPONSE);
    CHECK(response[3]==PROTOCOL_APF_STEP_RESPONSE_PAYLOAD_LEN);
    CHECK(response[4]==57U && response[5]==0U);
    for(i=0U;i<3U;i++) {
        float decoded;
        uint32_t raw=(uint32_t)response[6U+4U*i] |
            ((uint32_t)response[7U+4U*i]<<8U) |
            ((uint32_t)response[8U+4U*i]<<16U) |
            ((uint32_t)response[9U+4U*i]<<24U);
        memcpy(&decoded,&raw,4U);
        CHECK(decoded==result[i]);
    }
    {
        unsigned int sum=0U;
        for(i=2U;i<sizeof(response)-1U;i++) sum+=response[i];
        CHECK(response[sizeof(response)-1U]==(uint8_t)sum);
    }
    CHECK(protocol_build_apf_step_response(58U,
          ROBOT_STATUS_ERROR_NO_SOLUTION,NULL,response)==ROBOT_STATUS_OK);
    CHECK(response[4]==58U && (int8_t)response[5]==
          (int8_t)ROBOT_STATUS_ERROR_NO_SOLUTION);
    for(i=6U;i<sizeof(response)-1U;i++) CHECK(response[i]==0U);
    CHECK(protocol_build_apf_step_response(58U,
          ROBOT_STATUS_OK,NULL,response)==ROBOT_STATUS_ERROR_NULL_POINTER);
    printf("QEMU APF preview wire tests: %u failures\\n",failures);
    return failures ? 1 : 0;
}
