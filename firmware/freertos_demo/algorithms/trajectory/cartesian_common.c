#include <math.h>
#include <float.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "cartesian_internal.h"

/* Algorithmic SO(3) validation precision, not UR5 geometry parameters. */
static int rotation_quaternion(const robot_transform_t *pose, double q[4])
{
    double r[3][3],norm,trace;
    int i,j,k;
    double determinant, scale;

    if (pose == NULL) return 0;
    for (i=0;i<4;i++)
        for (j=0;j<4;j++)
            if (!isfinite((double)pose->matrix[i][j])) return 0;
    if (fabs((double)pose->matrix[3][0]) > 1e-5 ||
        fabs((double)pose->matrix[3][1]) > 1e-5 ||
        fabs((double)pose->matrix[3][2]) > 1e-5 ||
        fabs((double)pose->matrix[3][3]-1.0) > 1e-5)
        return 0;
    for(i=0;i<3;i++)
        for(j=0;j<3;j++)
            r[i][j] = (double)pose->matrix[i][j];
    for(i=0;i<3;i++)
        for(j=0;j<3;j++) {
            double dot=0.0;
            for(k=0;k<3;k++) dot+=r[k][i]*r[k][j];
            if(fabs(dot-((i==j)?1.0:0.0))>0.002) return 0;
        }
    determinant = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
        -r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
        +r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
    if (fabs(determinant-1.0)>0.004) return 0;

    trace = r[0][0]+r[1][1]+r[2][2];
    if (trace > 0.0) {
        scale=sqrt(trace+1.0)*2.0;
        q[0]=0.25*scale;
        q[1]=(r[2][1]-r[1][2])/scale;
        q[2]=(r[0][2]-r[2][0])/scale;
        q[3]=(r[1][0]-r[0][1])/scale;
    } else if(r[0][0]>=r[1][1] && r[0][0]>=r[2][2]) {
        scale=sqrt(1.0+r[0][0]-r[1][1]-r[2][2])*2.0;
        q[0]=(r[2][1]-r[1][2])/scale;
        q[1]=0.25*scale; q[2]=(r[0][1]+r[1][0])/scale;
        q[3]=(r[0][2]+r[2][0])/scale;
    } else if(r[1][1]>=r[2][2]) {
        scale=sqrt(1.0+r[1][1]-r[0][0]-r[2][2])*2.0;
        q[0]=(r[0][2]-r[2][0])/scale;
        q[1]=(r[0][1]+r[1][0])/scale; q[2]=0.25*scale;
        q[3]=(r[1][2]+r[2][1])/scale;
    } else {
        scale=sqrt(1.0+r[2][2]-r[0][0]-r[1][1])*2.0;
        q[0]=(r[1][0]-r[0][1])/scale;
        q[1]=(r[0][2]+r[2][0])/scale;
        q[2]=(r[1][2]+r[2][1])/scale; q[3]=0.25*scale;
    }
    norm=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!(norm>0.0) || !isfinite(norm)) return 0;
    for(i=0;i<4;i++) q[i]/=norm;
    return 1;
}

robot_status_t cartesian_prepare(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *end,
    double duration_s, double period_s,
    cartesian_path_kind_t kind)
{
    cartesian_trajectory_plan_t next;
    double steps;
    int i;
    if (out==NULL || start==NULL || end==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(!(duration_s>0.0) || !(period_s>0.0) ||
       !isfinite(duration_s) || !isfinite(period_s) ||
       (kind!=CARTESIAN_PATH_LINE && kind!=CARTESIAN_PATH_ARC))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    steps=ceil(duration_s/period_s);
    if(!isfinite(steps) || steps<1.0 || steps>(double)(UINT32_MAX-1U))
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    memset(&next,0,sizeof(next));
    if (!rotation_quaternion(start,next.start_quat) ||
        !rotation_quaternion(end,next.end_quat))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    next.start=*start;
    next.end=*end;
    next.duration_s=duration_s;
    next.period_s=period_s;
    next.interval_count=(uint32_t)steps;
    next.kind=kind;
    for(i=0;i<3;i++) {
        if(!isfinite((double)start->matrix[i][3]) ||
           !isfinite((double)end->matrix[i][3]))
           return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }
    *out=next;
    return ROBOT_STATUS_OK;
}

void cartesian_interpolate_rotation(
    const cartesian_trajectory_plan_t *plan,
    double u, robot_transform_t *out)
{
    double a[4],b[4],dot,angle,k0,k1,q[4],norm;
    int i;
    for(i=0;i<4;i++) {
        a[i]=plan->start_quat[i];
        b[i]=plan->end_quat[i];
    }
    dot=0.0;
    for(i=0;i<4;i++) dot+=a[i]*b[i];
    if(dot<0.0) {
        for(i=0;i<4;i++) b[i]=-b[i];
        dot=-dot;
    }
    if(dot>1.0) dot=1.0;
    if(dot>0.9995) {
        k0=1.0-u; k1=u;
    } else {
        angle=acos(dot);
        k0=sin((1.0-u)*angle)/sin(angle);
        k1=sin(u*angle)/sin(angle);
    }
    norm=0.0;
    for(i=0;i<4;i++) {
        q[i]=k0*a[i]+k1*b[i];
        norm+=q[i]*q[i];
    }
    norm=sqrt(norm);
    for(i=0;i<4;i++) q[i]/=norm;

    {
        double w=q[0],x=q[1],y=q[2],z=q[3];
        out->matrix[0][0]=(robot_real_t)(1.0-2.0*(y*y+z*z));
        out->matrix[0][1]=(robot_real_t)(2.0*(x*y-z*w));
        out->matrix[0][2]=(robot_real_t)(2.0*(x*z+y*w));
        out->matrix[1][0]=(robot_real_t)(2.0*(x*y+z*w));
        out->matrix[1][1]=(robot_real_t)(1.0-2.0*(x*x+z*z));
        out->matrix[1][2]=(robot_real_t)(2.0*(y*z-x*w));
        out->matrix[2][0]=(robot_real_t)(2.0*(x*z-y*w));
        out->matrix[2][1]=(robot_real_t)(2.0*(y*z+x*w));
        out->matrix[2][2]=(robot_real_t)(1.0-2.0*(x*x+y*y));
    }
    out->matrix[3][0]=0.0F;
    out->matrix[3][1]=0.0F;
    out->matrix[3][2]=0.0F;
    out->matrix[3][3]=1.0F;
}
