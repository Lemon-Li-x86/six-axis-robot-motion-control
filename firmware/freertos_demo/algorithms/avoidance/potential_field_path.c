#include <math.h>
#include <stddef.h>
#include "potential_field_path.h"

static double point_distance(const robot_apf_point_t *a,
                             const robot_apf_point_t *b)
{
    const double dx=a->mm[0]-b->mm[0];
    const double dy=a->mm[1]-b->mm[1];
    const double dz=a->mm[2]-b->mm[2];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

static robot_status_t trace_pass(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *points,
    size_t capacity,
    size_t *count)
{
    robot_apf_point_t current=*start;
    size_t i;
    for(i=0U;i<capacity;++i) {
        robot_apf_point_t next;
        robot_apf_step_info_t info;
        robot_status_t status=robot_apf_step(
            &current,goal,obstacles,obstacle_count,config,&next,&info);
        if(status!=ROBOT_STATUS_OK) return status;
        if(info.reached_goal!=0U) {
            *count=i;
            return ROBOT_STATUS_OK;
        }
        /* Reject stagnation even if a mathematically nonzero force exists. */
        if(!(point_distance(&current,&next)>1e-9))
            return ROBOT_STATUS_ERROR_NO_SOLUTION;
        if(points!=NULL) points[i]=next;
        current=next;
        if(point_distance(&current,goal)<=config->goal_tolerance_mm) {
            *count=i+1U;
            return ROBOT_STATUS_OK;
        }
    }
    return ROBOT_STATUS_ERROR_NO_SOLUTION;
}

robot_status_t robot_apf_plan_path(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *points,
    size_t capacity,
    size_t *count)
{
    size_t expected=0U,actual=0U;
    robot_status_t s;
    if(count==NULL || points==NULL || start==NULL || goal==NULL ||
       config==NULL || (obstacle_count!=0U && obstacles==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    *count=0U;
    if(capacity==0U) return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    /* Preflight without touching caller's buffer. A failed APF step,
     * local minimum, invalid geometry or exhausted capacity is atomic. */
    s=trace_pass(start,goal,obstacles,obstacle_count,config,NULL,
                 capacity,&expected);
    if(s!=ROBOT_STATUS_OK) return s;
    /* Deterministic recomputation uses identical immutable inputs. */
    s=trace_pass(start,goal,obstacles,obstacle_count,config,points,
                 capacity,&actual);
    if(s!=ROBOT_STATUS_OK || actual!=expected) {
        /* Only possible with concurrent mutations of caller-owned inputs.
         * This function is not thread-safe against such mutation. */
        return ROBOT_STATUS_ERROR_INTERNAL;
    }
    *count=actual;
    return ROBOT_STATUS_OK;
}
