#include <math.h>
#include <stdio.h>
#include "potential_field.h"

static unsigned checks=0U, failures=0U;
#define CHECK(condition) do { ++checks; if(!(condition)) { ++failures; \
  fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); } } while(0)

static const robot_apf_config_t normal={
    1.0, 10000.0, 30.0, 0.5, 2.0, 0.1
};
static const robot_apf_aabb_t box={
    {{4.0,-1.0,-1.0}}, {{6.0,1.0,1.0}}
};

static void free_space(void)
{
    const robot_apf_point_t start={{0.0,0.0,0.0}};
    const robot_apf_point_t goal={{10.0,0.0,0.0}};
    robot_apf_point_t next={{-777.0,-777.0,-777.0}};
    robot_apf_step_info_t info={0};
    CHECK(robot_apf_step(&start,&goal,NULL,0U,&normal,&next,&info)
          ==ROBOT_STATUS_OK);
    CHECK(fabs(next.mm[0]-2.0)<1e-9);
    CHECK(next.mm[1]==0.0 && next.mm[2]==0.0);
    CHECK(info.reached_goal==0U);
    CHECK(isinf(info.nearest_surface_distance_mm));

    {
        robot_apf_point_t near={{9.95,0.0,0.0}};
        CHECK(robot_apf_step(&near,&goal,NULL,0U,&normal,&next,&info)
              ==ROBOT_STATUS_OK);
        CHECK(info.reached_goal==1U);
        CHECK(next.mm[0]==near.mm[0]);
        CHECK(info.chosen_step_mm==0.0);
    }
}

static void repulsion_and_safety(void)
{
    robot_apf_point_t start={{0.0,2.0,0.0}};
    const robot_apf_point_t goal={{10.0,2.0,0.0}};
    robot_apf_point_t next={{-777.0,-777.0,-777.0}};
    robot_apf_step_info_t info={0};
    robot_apf_config_t config=normal;
    CHECK(robot_apf_step(&start,&goal,&box,1U,&config,&next,&info)
          ==ROBOT_STATUS_OK);
    CHECK(next.mm[1]>start.mm[1]); /* repulsion away from cuboid */
    CHECK(isfinite(info.nearest_surface_distance_mm));
    CHECK(info.nearest_surface_distance_mm>=0.5);
    CHECK(hypot(next.mm[0]-start.mm[0],next.mm[1]-start.mm[1])
          <=config.maximum_step_mm+1e-9);

    /* Head-on obstacle, repulsion disabled: reject a swept-segment
     * collision and preserve both output structures atomically. */
    start.mm[1]=0.0;
    config.maximum_step_mm=5.0;
    config.repulsion_gain=0.0;
    info.reached_goal=77U;
    CHECK(robot_apf_step(&start,&goal,&box,1U,&config,&next,&info)
          ==ROBOT_STATUS_OK); /* goal y=2: path stays outside AABB */
    {
        const robot_apf_point_t blocked_goal={{10.0,0.0,0.0}};
        next.mm[0]=-777.0;
        info.reached_goal=77U;
        CHECK(robot_apf_step(&start,&blocked_goal,&box,1U,&config,
              &next,&info)==ROBOT_STATUS_ERROR_NO_SOLUTION);
        CHECK(next.mm[0]==-777.0);
        CHECK(info.reached_goal==77U);
    }

    start.mm[0]=5.0;
    CHECK(robot_apf_step(&start,&goal,&box,1U,&config,&next,&info)
          ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    start.mm[0]=0.0;
    {
        const robot_apf_point_t invalid_goal={{5.0,0.0,0.0}};
        CHECK(robot_apf_step(&start,&invalid_goal,&box,1U,&config,
              &next,&info)==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    }
}

static void invalid_arguments(void)
{
    const robot_apf_point_t p={{0.0,0.0,0.0}};
    robot_apf_point_t out={{1.0,1.0,1.0}};
    robot_apf_config_t bad=normal;
    robot_apf_aabb_t invalid=box;
    CHECK(robot_apf_step(NULL,&p,&box,1U,&bad,&out,NULL)
          ==ROBOT_STATUS_ERROR_NULL_POINTER);
    CHECK(robot_apf_step(&p,&p,NULL,1U,&bad,&out,NULL)
          ==ROBOT_STATUS_ERROR_NULL_POINTER);
    CHECK(robot_apf_step(&p,&p,&box,ROBOT_APF_MAX_OBSTACLES+1U,
          &bad,&out,NULL)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    bad.maximum_step_mm=NAN;
    CHECK(robot_apf_step(&p,&p,NULL,0U,&bad,&out,NULL)
          ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(out.mm[0]==1.0);
    bad=normal;
    invalid.max_corner.mm[0]=invalid.min_corner.mm[0];
    CHECK(robot_apf_step(&p,&p,&invalid,1U,&bad,&out,NULL)
          ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(out.mm[0]==1.0);
}

int main(void)
{
    free_space();
    repulsion_and_safety();
    invalid_arguments();
    printf("Stage5 potential field: %u checks, %u failures\n",checks,failures);
    return failures==0U?0:1;
}
