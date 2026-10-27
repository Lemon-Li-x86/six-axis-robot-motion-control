/*
 * Stage5 fault injection against the EXACT production C byte parser.
 * In-memory serial byte stream, not a claimed UART/FreeRTOS integration.
 * No alternative checksum implementation or duplicated wire constants.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "protocol.h"

static unsigned checks, failed;
#define CHECK(expr) do { ++checks; if(!(expr)) { ++failed; \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#expr); } } while(0)

static unsigned feed(protocol_parser_t *parser,const uint8_t *bytes,
                     size_t length,protocol_frame_t *out)
{
    size_t i;
    unsigned received=0U;
    for(i=0U;i<length;++i)
        received+=(unsigned)protocol_parser_process_byte(parser,bytes[i],out);
    return received;
}

static void verify_chunks_and_noise(const uint8_t frame[PROTOCOL_JOINT_FRAME_LEN])
{
    const uint8_t noise[]={0x00U,0x88U,PROTOCOL_HEADER_0,
                           0x00U,PROTOCOL_HEADER_0};
    protocol_parser_t parser;
    protocol_frame_t result={0};
    unsigned count=0U;
    protocol_parser_init(&parser);
    CHECK(feed(&parser,noise,sizeof(noise),&result)==0U);
    /* Last junk byte was a candidate header. Another header must
     * keep the parser synchronized, rather than incorrectly reset. */
    count+=feed(&parser,frame,1U,&result);
    count+=feed(&parser,frame+1U,2U,&result);
    count+=feed(&parser,frame+3U,4U,&result);
    count+=feed(&parser,frame+7U,PROTOCOL_JOINT_FRAME_LEN-7U,&result);
    CHECK(count==1U);
    CHECK(result.command==CMD_SET_JOINT_TARGETS);
    CHECK(result.length==PROTOCOL_JOINT_PAYLOAD_LEN);
}

static void verify_checksum_and_overlength(
    const uint8_t frame[PROTOCOL_JOINT_FRAME_LEN])
{
    uint8_t damaged[PROTOCOL_JOINT_FRAME_LEN];
    const uint8_t overlength[]={
        PROTOCOL_HEADER_0,PROTOCOL_HEADER_1,CMD_SET_JOINT_TARGETS,
        (uint8_t)(PROTOCOL_MAX_PAYLOAD_LEN+1U)
    };
    protocol_parser_t parser;
    protocol_frame_t out={0};
    protocol_parser_init(&parser);
    memcpy(damaged,frame,sizeof(damaged));
    damaged[PROTOCOL_JOINT_FRAME_LEN-1U]^=0x40U;
    CHECK(feed(&parser,damaged,sizeof(damaged),&out)==0U);
    CHECK(feed(&parser,frame,PROTOCOL_JOINT_FRAME_LEN,&out)==1U);
    CHECK(feed(&parser,overlength,sizeof(overlength),&out)==0U);
    CHECK(feed(&parser,frame,PROTOCOL_JOINT_FRAME_LEN,&out)==1U);
}

static void verify_lost_byte_followed_by_retransmit(
    const uint8_t frame[PROTOCOL_JOINT_FRAME_LEN])
{
    protocol_parser_t parser;
    protocol_frame_t out={0};
    unsigned count=0U;
    const size_t dropped_index=6U; /* in payload, not length/header */
    protocol_parser_init(&parser);
    count+=feed(&parser,frame,dropped_index,&out);
    count+=feed(&parser,frame+dropped_index+1U,
                PROTOCOL_JOINT_FRAME_LEN-dropped_index-1U,&out);
    /* The next header lands in the checksum slot of the damaged
     * previous frame: it MUST be reconsidered as a possible header.
     * Otherwise one corrupted frame silently loses its retransmit. */
    count+=feed(&parser,frame,PROTOCOL_JOINT_FRAME_LEN,&out);
    CHECK(count==1U);
    CHECK(out.command==CMD_SET_JOINT_TARGETS);
    CHECK(out.length==PROTOCOL_JOINT_PAYLOAD_LEN);
}

static void verify_explicit_timeout_reset(
    const uint8_t frame[PROTOCOL_JOINT_FRAME_LEN])
{
    protocol_parser_t parser;
    protocol_frame_t out={0};
    protocol_parser_init(&parser);
    CHECK(feed(&parser,frame,7U,&out)==0U);
    /* This parser has NO timer. The UART owner must call init upon
     * a detected inter-frame timeout; this only tests reset semantics. */
    protocol_parser_init(&parser);
    CHECK(feed(&parser,frame,PROTOCOL_JOINT_FRAME_LEN,&out)==1U);
}

int main(void)
{
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN];
    const robot_joint_angles_t joints={{0,0,0,0,0,0}};
    CHECK(protocol_build_joint_target_frame(&joints,frame)==ROBOT_STATUS_OK);
    verify_chunks_and_noise(frame);
    verify_checksum_and_overlength(frame);
    verify_lost_byte_followed_by_retransmit(frame);
    verify_explicit_timeout_reset(frame);
    printf("Stage5 serial byte fault cases: %u assertions, %u failures\n",
           checks,failed);
    return failed==0U?0:1;
}
