/**
 * @file potential_field_path_test_main.c
 * @brief 回归测试或夹具：potential field path。
 */

#include <math.h>
#include <stdio.h>
#include "potential_field_path.h"

#define CHECK(x) do { ++checks; if(!(x)) { ++failures; \
  fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); } } while(0)
static unsigned checks,failures;
static const robot_apf_config_t config={1.0,10000.0,30.0,0.5,2.0,0.1};

static void normal_path(void)
{
    const robot_apf_point_t start={{0.0,0.0,0.0}};
    const robot_apf_point_t goal={{10.0,0.0,0.0}};
    robot_apf_point_t path[8];
    size_t n=77U,i;
    CHECK(robot_apf_plan_path(&start,&goal,NULL,0U,&config,path,8U,&n)==ROBOT_STATUS_OK);
    CHECK(n==5U);
    for(i=0U;i<n;i++) {
        CHECK(fabs(path[i].mm[0]-2.0*(double)(i+1U))<1e-9);
        CHECK(path[i].mm[1]==0.0 && path[i].mm[2]==0.0);
    }
    n=77U;
    CHECK(robot_apf_plan_path(&goal,&goal,NULL,0U,&config,path,8U,&n)==ROBOT_STATUS_OK);
    CHECK(n==0U);
}

static void rejection_is_atomic(void)
{
    const robot_apf_point_t start={{0.0,0.0,0.0}};
    const robot_apf_point_t goal={{10.0,0.0,0.0}};
    const robot_apf_aabb_t obstacle={{{4.0,-1.0,-1.0}},{{6.0,1.0,1.0}}};
    robot_apf_config_t disabled=config;
    robot_apf_point_t path[8];
    size_t n=77U,i;
    for(i=0U;i<8U;i++) path[i].mm[0]=-777.0;
    CHECK(robot_apf_plan_path(&start,&goal,NULL,0U,&config,path,2U,&n)
          ==ROBOT_STATUS_ERROR_NO_SOLUTION);
    CHECK(n==0U);
    for(i=0U;i<8U;i++) CHECK(path[i].mm[0]==-777.0);
    disabled.repulsion_gain=0.0;
    disabled.maximum_step_mm=5.0;
    n=77U;
    CHECK(robot_apf_plan_path(&start,&goal,&obstacle,1U,&disabled,path,8U,&n)
          ==ROBOT_STATUS_ERROR_NO_SOLUTION);
    CHECK(n==0U && path[0].mm[0]==-777.0);
    n=77U;
    CHECK(robot_apf_plan_path(&start,&goal,NULL,0U,&config,path,0U,&n)
          ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(n==0U && path[0].mm[0]==-777.0);
    n=77U;
    CHECK(robot_apf_plan_path(NULL,&goal,NULL,0U,&config,path,8U,&n)
          ==ROBOT_STATUS_ERROR_NULL_POINTER);
}

static void geometric_bypass_candidates(void)
{
    const robot_apf_point_t start={{0.0,0.0,0.0}};
    const robot_apf_point_t goal={{-180.0,0.0,0.0}};
    const robot_apf_aabb_t box={{{-105.0,-15.0,-15.0}},
                                {{-75.0,15.0,15.0}}};
    robot_apf_config_t apf=config;
    robot_apf_point_t vias[ROBOT_APF_BYPASS_MAX_CANDIDATES];
    size_t count=99U,i;
    unsigned int blocked=1U;
    apf.clearance_mm=12.0;
    apf.maximum_step_mm=10.0;
    CHECK(robot_apf_generate_bypass_vias(
        &start,&goal,&box,1U,&apf,vias,
        ROBOT_APF_BYPASS_MAX_CANDIDATES,&count)==ROBOT_STATUS_OK);
    CHECK(count>=2U && count<=ROBOT_APF_BYPASS_MAX_CANDIDATES);
    /* The first candidate uses obstacle-derived Z clearance rather than
     * the historical fixed +90mm via from scene JSON. */
    CHECK(vias[0].mm[2]>box.max_corner.mm[2]+apf.clearance_mm);
    for(i=0U;i<count;i++) {
        CHECK(robot_apf_segment_blocked(
            &start,&vias[i],&box,1U,apf.clearance_mm,&blocked)==ROBOT_STATUS_OK);
        CHECK(blocked==0U);
        CHECK(robot_apf_segment_blocked(
            &vias[i],&goal,&box,1U,apf.clearance_mm,&blocked)==ROBOT_STATUS_OK);
        CHECK(blocked==0U);
    }
    /* Enlarging the obstacle changes the candidates, no fixed pose reused. */
    {
        const robot_apf_aabb_t larger={{{-110.0,-20.0,-20.0}},
                                       {{-70.0,20.0,20.0}}};
        const robot_apf_point_t old_first=vias[0];
        CHECK(robot_apf_generate_bypass_vias(
            &start,&goal,&larger,1U,&apf,vias,
            ROBOT_APF_BYPASS_MAX_CANDIDATES,&count)==ROBOT_STATUS_OK);
        CHECK(count>0U);
        CHECK(vias[0].mm[2]>old_first.mm[2]);
    }
}
static void bypass_rejects_unsupported_or_invalid_without_writing_buffer(void)
{
    const robot_apf_point_t start={{0.0,0.0,0.0}};
    const robot_apf_point_t goal={{-180.0,0.0,0.0}};
    const robot_apf_aabb_t box={{{-105.0,-15.0,-15.0}},
                                {{-75.0,15.0,15.0}}};
    robot_apf_point_t vias[ROBOT_APF_BYPASS_MAX_CANDIDATES];
    size_t n=123U;
    vias[0].mm[0]=-98765.0;
    CHECK(robot_apf_generate_bypass_vias(
        &start,&goal,&box,2U,&config,vias,
        ROBOT_APF_BYPASS_MAX_CANDIDATES,&n)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(n==0U && vias[0].mm[0]==-98765.0);
    CHECK(robot_apf_generate_bypass_vias(
        &start,&goal,&box,1U,&config,vias,
        1U,&n)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(n==0U && vias[0].mm[0]==-98765.0);
    CHECK(robot_apf_generate_bypass_vias(
        NULL,&goal,&box,1U,&config,vias,
        ROBOT_APF_BYPASS_MAX_CANDIDATES,&n)==ROBOT_STATUS_ERROR_NULL_POINTER);
    CHECK(vias[0].mm[0]==-98765.0);
}

int main(void)
{
    normal_path();
    rejection_is_atomic();
    geometric_bypass_candidates();
    bypass_rejects_unsupported_or_invalid_without_writing_buffer();
    printf("Stage5 bounded APF path: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
