#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "potential_field.h"

static int valid_point(const robot_apf_point_t *p)
{
    size_t axis;
    for(axis=0U;axis<3U;++axis)
        if(!isfinite(p->mm[axis])) return 0;
    return 1;
}

static double clamp_double(double value,double lo,double hi)
{
    return value<lo?lo:(value>hi?hi:value);
}

static int intersect_segment_aabb(
    const robot_apf_point_t *a,
    const robot_apf_point_t *b,
    const double lo[3],
    const double hi[3])
{
    double tmin=0.0,tmax=1.0;
    size_t axis;
    for(axis=0U;axis<3U;++axis) {
        const double delta=b->mm[axis]-a->mm[axis];
        double near_t,far_t;
        if(fabs(delta)<1e-12) {
            if(a->mm[axis]<lo[axis] || a->mm[axis]>hi[axis])
                return 0;
            continue;
        }
        near_t=(lo[axis]-a->mm[axis])/delta;
        far_t=(hi[axis]-a->mm[axis])/delta;
        if(near_t>far_t) {
            double swap=near_t;
            near_t=far_t;
            far_t=swap;
        }
        if(near_t>tmin) tmin=near_t;
        if(far_t<tmax) tmax=far_t;
        if(tmin>tmax) return 0;
    }
    return 1; /* touching a clearance boundary is forbidden */
}

/* Public Cartesian geometry query used by scenario/mission acceptance.
 * Uses the same private slab intersection as the C APF safety gate.
 * Never changes output or global state on malformed input. */
robot_status_t robot_apf_segment_blocked(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    double clearance_mm,
    unsigned int *blocked)
{
    size_t i,axis;
    unsigned int hit=0U;
    if(start==NULL || goal==NULL || blocked==NULL ||
       (obstacle_count>0U && obstacles==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(obstacle_count>ROBOT_APF_MAX_OBSTACLES ||
       !valid_point(start) || !valid_point(goal) ||
       !isfinite(clearance_mm) || clearance_mm<0.0)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    for(i=0U;i<obstacle_count;++i) {
        double lo[3],hi[3];
        for(axis=0U;axis<3U;++axis) {
            double l=obstacles[i].min_corner.mm[axis];
            double h=obstacles[i].max_corner.mm[axis];
            if(!isfinite(l) || !isfinite(h) || l>=h)
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            lo[axis]=l-clearance_mm;
            hi[axis]=h+clearance_mm;
            if(!isfinite(lo[axis]) || !isfinite(hi[axis]))
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        }
        if(intersect_segment_aabb(start,goal,lo,hi))
            hit=1U;
    }
    *blocked=hit;
    return ROBOT_STATUS_OK;
}

robot_status_t robot_apf_step(
    const robot_apf_point_t *current,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *next,
    robot_apf_step_info_t *details)
{
    double force[3]={0.0,0.0,0.0};
    double delta[3],goal_distance_squared=0.0;
    double force_squared=0.0,goal_distance,step_length;
    double nearest_distance=INFINITY;
    robot_apf_point_t candidate;
    robot_apf_step_info_t info={INFINITY,0.0,0U};
    size_t axis,i;

    if(current==NULL || goal==NULL || config==NULL || next==NULL ||
       (obstacle_count>0U && obstacles==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(obstacle_count>ROBOT_APF_MAX_OBSTACLES ||
       !valid_point(current) || !valid_point(goal) ||
       !(config->attraction_gain>0.0) ||
       !(config->repulsion_gain>=0.0) ||
       !(config->influence_distance_mm>0.0) ||
       !(config->clearance_mm>=0.0) ||
       !(config->maximum_step_mm>0.0) ||
       !(config->goal_tolerance_mm>=0.0) ||
       !isfinite(config->attraction_gain) ||
       !isfinite(config->repulsion_gain) ||
       !isfinite(config->influence_distance_mm) ||
       !isfinite(config->clearance_mm) ||
       !isfinite(config->maximum_step_mm) ||
       !isfinite(config->goal_tolerance_mm))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    for(axis=0U;axis<3U;++axis) {
        delta[axis]=goal->mm[axis]-current->mm[axis];
        goal_distance_squared+=delta[axis]*delta[axis];
    }
    goal_distance=sqrt(goal_distance_squared);
    if(!isfinite(goal_distance))
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

    for(i=0U;i<obstacle_count;++i) {
        double lo[3],hi[3],away[3],distance_squared=0.0;
        double distance;
        for(axis=0U;axis<3U;++axis) {
            const double raw_lo=obstacles[i].min_corner.mm[axis];
            const double raw_hi=obstacles[i].max_corner.mm[axis];
            if(!isfinite(raw_lo) || !isfinite(raw_hi) || raw_lo>=raw_hi)
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            lo[axis]=raw_lo-config->clearance_mm;
            hi[axis]=raw_hi+config->clearance_mm;
            if(!isfinite(lo[axis]) || !isfinite(hi[axis]))
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            away[axis]=current->mm[axis]-
                clamp_double(current->mm[axis],lo[axis],hi[axis]);
            distance_squared+=away[axis]*away[axis];
        }
        if(goal->mm[0]>=lo[0] && goal->mm[0]<=hi[0] &&
           goal->mm[1]>=lo[1] && goal->mm[1]<=hi[1] &&
           goal->mm[2]>=lo[2] && goal->mm[2]<=hi[2])
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        distance=sqrt(distance_squared);
        if(!isfinite(distance))
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        if(distance==0.0) /* at or within the inflated obstacle */
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        if(distance<nearest_distance) nearest_distance=distance;
        if(distance<config->influence_distance_mm &&
           config->repulsion_gain>0.0) {
            const double inv_d=1.0/distance;
            const double inv_influence=1.0/
                config->influence_distance_mm;
            const double scale=config->repulsion_gain*
                (inv_d-inv_influence)*inv_d*inv_d;
            if(!isfinite(scale)) return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
            for(axis=0U;axis<3U;++axis)
                force[axis]+=scale*away[axis]*inv_d;
        }
    }

    info.nearest_surface_distance_mm=nearest_distance;

    if(goal_distance<=config->goal_tolerance_mm) {
        /* Within tolerance: hold position; do not jump farther than a
         * configured step merely to snap to an exact floating-point goal. */
        candidate=*current;
        info.reached_goal=1U;
        step_length=0.0;
    } else {
        for(axis=0U;axis<3U;++axis)
            force[axis]+=config->attraction_gain*
                delta[axis]/goal_distance;

        for(axis=0U;axis<3U;++axis)
            force_squared+=force[axis]*force[axis];
        if(!isfinite(force_squared))
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        if(force_squared<=1e-24)
            return ROBOT_STATUS_ERROR_NO_SOLUTION; /* local minimum */

        step_length=config->maximum_step_mm;
        if(step_length>goal_distance) step_length=goal_distance;
        for(axis=0U;axis<3U;++axis)
            candidate.mm[axis]=current->mm[axis]+
                step_length*force[axis]/sqrt(force_squared);
    }
    if(!valid_point(&candidate))
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

    /* No silent tunnelling past thin or inflated AABBs. */
    for(i=0U;i<obstacle_count;++i) {
        double lo[3],hi[3];
        for(axis=0U;axis<3U;++axis) {
            lo[axis]=obstacles[i].min_corner.mm[axis]-
                config->clearance_mm;
            hi[axis]=obstacles[i].max_corner.mm[axis]+
                config->clearance_mm;
        }
        if(intersect_segment_aabb(current,&candidate,lo,hi))
            return ROBOT_STATUS_ERROR_NO_SOLUTION;
    }
    info.chosen_step_mm=step_length;
    *next=candidate;
    if(details!=NULL) *details=info;
    return ROBOT_STATUS_OK;
}
