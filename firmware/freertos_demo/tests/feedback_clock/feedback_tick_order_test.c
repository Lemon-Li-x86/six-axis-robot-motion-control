/* Runs the real motor_state.c gateway with deterministic FreeRTOS mocks.
 * Reproduce an RX frame accepted after a trajectory task's old Tick read
 * but before it obtains the mutex-protected feedback snapshot.
 * No fake controller or alternative motion planner is involved.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "motor_state.h"

static int mock_mutex;
static uint32_t now_tick;
static uint32_t snapshot_clock_tick;
static uint32_t clock_read_count;
static int feedback_available;
static motor_feedback_snapshot_t snapshot;

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &mock_mutex; }
int xSemaphoreTake(SemaphoreHandle_t mutex,uint32_t wait)
{
    (void)wait;
    return mutex==&mock_mutex ? pdTRUE : 0;
}
int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    return mutex==&mock_mutex ? pdTRUE : 0;
}
uint32_t xTaskGetTickCount(void)
{
    clock_read_count++;
    return now_tick;
}
robot_status_t motor_driver_update_feedback(const robot_joint_angles_t *raw,
                                             robot_real_t delta)
{
    (void)delta;
    if(raw==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    snapshot.canonical_positions=*raw;
    snapshot.sample_sequence++;
    feedback_available=1;
    return ROBOT_STATUS_OK;
}
robot_status_t motor_driver_get_feedback_snapshot(motor_feedback_snapshot_t *out)
{
    if(!feedback_available) return ROBOT_STATUS_ERROR_NOT_READY;
    /* ProtocolRX advances guest time while the planner waits for mutex. */
    now_tick=snapshot_clock_tick;
    *out=snapshot;
    return ROBOT_STATUS_OK;
}
robot_status_t motor_driver_set_target_positions(const robot_joint_angles_t *raw)
{ return raw==NULL ? ROBOT_STATUS_ERROR_NULL_POINTER : ROBOT_STATUS_OK; }
robot_status_t motor_driver_get_target_positions(robot_joint_angles_t *raw)
{ if(raw==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER; *raw=snapshot.canonical_positions; return ROBOT_STATUS_OK; }

#define REQUIRE(cond) do {if(!(cond)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#cond);return 1;}} while(0)
int main(void)
{
    const robot_joint_angles_t feedback={{0}};
    motor_state_observation_t out;
    motor_state_observation_t unchanged;
    uint32_t old_time=100U;

    REQUIRE(motor_state_init()==ROBOT_STATUS_OK);
    REQUIRE(motor_state_update_feedback(&feedback,0.01F,101U)==ROBOT_STATUS_OK);
    /* Same race as original bug: old planner read was tick 100, newer
     * ProtocolRX accepted tick 101. Unsigned age wraps to UINT32_MAX. */
    now_tick=old_time;
    snapshot_clock_tick=101U;
    REQUIRE(motor_state_feedback_is_fresh(&(motor_state_observation_t){
        .feedback={.sample_sequence=1U},.accepted_tick=101U
    },old_time,3000U)==0U);
    REQUIRE(motor_state_get_fresh_feedback(&out,3000U)==ROBOT_STATUS_OK);
    REQUIRE(clock_read_count==1U);
    REQUIRE(out.accepted_tick==101U);
    REQUIRE(out.feedback.sample_sequence==1U);

    /* Legitimate stale measurements still reject and do not publish. */
    memset(&out,0xA5,sizeof(out));
    unchanged=out;
    now_tick=5000U;
    snapshot_clock_tick=5000U;
    REQUIRE(motor_state_get_fresh_feedback(&out,3000U)==ROBOT_STATUS_ERROR_NOT_READY);
    REQUIRE(memcmp(&out,&unchanged,sizeof(out))==0);

    /* uint32 tick wrap: a snapshot accepted at UINT32_MAX-1 is still
     * fresh when read just after the counter wraps. */
    REQUIRE(motor_state_update_feedback(&feedback,0.01F,UINT32_MAX-1U)==ROBOT_STATUS_OK);
    now_tick=1U;
    snapshot_clock_tick=1U;
    REQUIRE(motor_state_get_fresh_feedback(&out,5U)==ROBOT_STATUS_OK);

    unchanged=out;
    REQUIRE(motor_state_get_fresh_feedback(NULL,5U)==ROBOT_STATUS_ERROR_NULL_POINTER);
    REQUIRE(memcmp(&out,&unchanged,sizeof(out))==0);
    REQUIRE(motor_state_get_fresh_feedback(&out,0U)==ROBOT_STATUS_ERROR_NOT_READY);
    puts("feedback clock ordering/wraparound/fail-closed: PASS");
    return 0;
}
