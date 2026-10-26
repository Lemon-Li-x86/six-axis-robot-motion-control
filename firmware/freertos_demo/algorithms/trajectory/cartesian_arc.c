#include <math.h>
#include <stddef.h>
#include "cartesian_internal.h"

/* Circle through 3 positions. Cross-product handles arbitrary 3-D plane. */
robot_status_t cartesian_arc_init(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *via,
    const robot_transform_t *end,
    double duration_s, double interpolation_period_s)
{
    cartesian_trajectory_plan_t next;
    double a[3],b[3],normal[3],center[3],v[3],radius2;
    double aa=0.0,bb=0.0,ab=0.0,det,ca,cb,normn,midang,endang;
    double angle0=0.0,angle1;
    unsigned i;
    robot_status_t status;
    if(out==NULL || start==NULL || via==NULL || end==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;

    status=cartesian_prepare(&next,start,end,duration_s,
                             interpolation_period_s,CARTESIAN_PATH_ARC);
    if(status!=ROBOT_STATUS_OK) return status;

    for(i=0U;i<3U;i++) {
        if(!isfinite((double)via->matrix[i][3]))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        a[i]=(double)via->matrix[i][3]-(double)start->matrix[i][3];
        b[i]=(double)end->matrix[i][3]-(double)start->matrix[i][3];
        aa+=a[i]*a[i]; bb+=b[i]*b[i]; ab+=a[i]*b[i];
    }
    det=aa*bb-ab*ab;
    if(!(aa>1e-12) || !(bb>1e-12) ||
       !(det>aa*bb*1e-12) || !isfinite(det))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    /* Center = ca*a + cb*b; center dot a=|a|²/2, dot b=|b|²/2. */
    ca=(0.5*aa*bb-0.5*bb*ab)/det;
    cb=(0.5*bb*aa-0.5*aa*ab)/det;
    for(i=0U;i<3U;i++)
        center[i]=(double)start->matrix[i][3]+ca*a[i]+cb*b[i];

    normal[0]=a[1]*b[2]-a[2]*b[1];
    normal[1]=a[2]*b[0]-a[0]*b[2];
    normal[2]=a[0]*b[1]-a[1]*b[0];
    normn=sqrt(det);
    for(i=0U;i<3U;i++) normal[i]/=normn;
    radius2=0.0;
    for(i=0U;i<3U;i++) {
        next.radial_u[i]=(double)start->matrix[i][3]-center[i];
        radius2+=next.radial_u[i]*next.radial_u[i];
    }
    if(!(radius2>1e-12) || !isfinite(radius2))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    next.radius_mm=sqrt(radius2);
    for(i=0U;i<3U;i++) next.radial_u[i]/=next.radius_mm;
    next.radial_v[0]=normal[1]*next.radial_u[2]-normal[2]*next.radial_u[1];
    next.radial_v[1]=normal[2]*next.radial_u[0]-normal[0]*next.radial_u[2];
    next.radial_v[2]=normal[0]*next.radial_u[1]-normal[1]*next.radial_u[0];
    for(i=0U;i<3U;i++) {
        next.center_mm[i]=center[i];
        v[i]=(double)via->matrix[i][3]-center[i];
    }
    midang=atan2(v[0]*next.radial_v[0]+v[1]*next.radial_v[1]+v[2]*next.radial_v[2],
                 v[0]*next.radial_u[0]+v[1]*next.radial_u[1]+v[2]*next.radial_u[2]);
    for(i=0U;i<3U;i++) v[i]=(double)end->matrix[i][3]-center[i];
    endang=atan2(v[0]*next.radial_v[0]+v[1]*next.radial_v[1]+v[2]*next.radial_v[2],
                 v[0]*next.radial_u[0]+v[1]*next.radial_u[1]+v[2]*next.radial_u[2]);
    /* Orientation chosen by traversal through the middle point. */
    if(midang < angle0) midang+=2.0*acos(-1.0);
    if(endang < angle0) endang+=2.0*acos(-1.0);
    if(midang < endang) angle1=endang;
    else angle1=endang-2.0*acos(-1.0);
    if(!isfinite(angle1) || fabs(angle1)<1e-9 ||
       fabs(angle1)>2.0*acos(-1.0)) return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    next.signed_sweep_rad=angle1;
    *out=next;
    return ROBOT_STATUS_OK;
}
