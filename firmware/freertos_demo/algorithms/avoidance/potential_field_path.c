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


/* Find bounded detours across a single axis-aligned obstacle. Candidate
 * screening is Cartesian only. It cannot license any robot motion.
 * Three clearance-dependent offsets per lateral direction are tried; the
 * caller, not this routine, selects one after full C APF/IK/link checks. */
size_t robot_apf_bypass_capacity(void)
{
    return ROBOT_APF_BYPASS_MAX_CANDIDATES;
}

robot_status_t robot_apf_generate_bypass_vias(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacle,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *via_candidates,
    size_t capacity,
    size_t *out_count)
{
    const size_t offsets_per_side=3U;
    const size_t dimension=3U;
    robot_apf_point_t choices[ROBOT_APF_BYPASS_MAX_CANDIDATES];
    size_t primary=0U, lateral[2], count=0U, direction, ring, axis;
    double span=0.0;
    unsigned int blocked=0U;
    robot_status_t status;
    if(start==NULL || goal==NULL || config==NULL ||
       via_candidates==NULL || out_count==NULL ||
       (obstacle_count!=0U && obstacle==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    *out_count=0U;
    if(obstacle_count!=1U ||
       capacity<ROBOT_APF_BYPASS_MAX_CANDIDATES ||
       !(config->maximum_step_mm>0.0) ||
       !(config->clearance_mm>=0.0) ||
       !isfinite(config->maximum_step_mm) ||
       !isfinite(config->clearance_mm))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    /* Validate geometry through the same C obstacle test as robot_apf_step.
     * No bounding box may be silently skipped. */
    status=robot_apf_segment_blocked(start,goal,obstacle,obstacle_count,
                                    config->clearance_mm,&blocked);
    if(status!=ROBOT_STATUS_OK) return status;
    if(blocked==0U) return ROBOT_STATUS_OK; /* direct route; no via required */

    for(axis=0U;axis<dimension;++axis) {
        const double delta=fabs(goal->mm[axis]-start->mm[axis]);
        if(delta>span) { span=delta; primary=axis; }
    }
    if(!(span>0.0)) return ROBOT_STATUS_ERROR_NO_SOLUTION;
    /* Prefer going over the box when X/Y dominates. This is a search
     * order, not a fixed via or a scene-specific numerical constant. */
    if(primary==0U) { lateral[0]=2U; lateral[1]=1U; }
    else if(primary==1U) { lateral[0]=2U; lateral[1]=0U; }
    else { lateral[0]=1U; lateral[1]=0U; }
    for(direction=0U;direction<4U;++direction) {
        const size_t side_axis=lateral[direction/2U];
        const int positive=(direction%2U)==0U;
        for(ring=1U;ring<=offsets_per_side;++ring) {
            robot_apf_point_t via={{0.0,0.0,0.0}};
            unsigned int first=0U,second=0U;
            const double gap=config->clearance_mm+
                             (double)(2U*ring)*config->maximum_step_mm;
            for(axis=0U;axis<dimension;++axis) {
                const double lo=obstacle->min_corner.mm[axis];
                const double hi=obstacle->max_corner.mm[axis];
                if(!isfinite(lo) || !isfinite(hi) || !(lo<hi))
                    return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
                via.mm[axis]=0.5*lo+0.5*hi;
            }
            via.mm[side_axis]=positive?
                obstacle->max_corner.mm[side_axis]+gap :
                obstacle->min_corner.mm[side_axis]-gap;
            if(!isfinite(via.mm[side_axis])) return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
            status=robot_apf_segment_blocked(
                start,&via,obstacle,1U,config->clearance_mm,&first);
            if(status!=ROBOT_STATUS_OK) return status;
            status=robot_apf_segment_blocked(
                &via,goal,obstacle,1U,config->clearance_mm,&second);
            if(status!=ROBOT_STATUS_OK) return status;
            if(first==0U && second==0U) {
                choices[count++]=via;
            }
        }
    }
    if(count==0U) return ROBOT_STATUS_ERROR_NO_SOLUTION;
    for(axis=0U;axis<count;++axis) via_candidates[axis]=choices[axis];
    *out_count=count;
    return ROBOT_STATUS_OK;
}
