/**
 * @file stage4_sim_bridge_smoke.c
 * @brief 回归测试或夹具：stage4 sim bridge smoke。
 */
#include <stdint.h>
#include <stdio.h>

extern void *stage4_sim_create(const int32_t *,const int32_t *);
/* Test-only gain layout: six joints by [Kp, Ki, Kd]. */
extern void *stage4_sim_create_with_pid_gains(
    const int32_t *,const int32_t *,const double [6][3]);
extern double stage4_sim_duration(const void *);
extern int stage4_sim_step(void *,const int32_t *,double,double,int32_t *,int32_t *);
extern void stage4_sim_destroy(void *);

/* Stage5 user-observed first C APF/IK waypoint as a regression fixture.
 * The simulation bridge links the PRODUCTION trajectory, PID and limiter.
 * An ideal follower distinguishes a C-control equilibrium error from the
 * PyBullet motor model or firmware task/transport scheduling.
 * This does not change any production parameter or safety threshold.
 */
static int stage5_first_leg_ideal_feedback(void)
{
    const int32_t start[6]={3000,-4500,6000,2000,500,-1000};
    const int32_t target[6]={3030,-4652,6131,1831,525,-809};
    int32_t feedback[6]={3000,-4500,6000,2000,500,-1000};
    int32_t command[6]={0},reference[6]={0};
    void *sim=stage4_sim_create(start,target);
    unsigned i,j;
    int32_t max_error=0;
    if(sim==NULL) return 12;
    for(i=0U;i<4000U;i++) {
        if(stage4_sim_step(sim,feedback,(double)i*0.01,0.01,
                          command,reference)!=0) {
            stage4_sim_destroy(sim);
            return 13;
        }
        /* Ideal plant: commanded integer centidegrees are exact feedback. */
        for(j=0U;j<6U;j++) feedback[j]=command[j];
    }
    for(j=0U;j<6U;j++) {
        int32_t diff=feedback[j]-target[j];
        if(diff<0) diff=-diff;
        if(diff>max_error) max_error=diff;
    }
    printf("Stage5 first-leg ideal C controller residual: %ld centideg\\n",
           (long)max_error);
    stage4_sim_destroy(sim);
    return max_error<=15 ? 0 : 14;
}

int main(void)
{
    const int32_t initial[6]={3000,-4500,6000,2000,500,-1000};
    const int32_t final[6]={3000,-4300,6000,2000,500,-1000};
    int32_t feedback[6]={3000,-4500,6000,2000,500,-1000};
    int32_t command[6]={0},ref[6]={0};
    double duration;
    unsigned i;
    void *handle=stage4_sim_create(initial,final);
    if(handle==NULL) return 1;
    duration=stage4_sim_duration(handle);
    if(!(duration>0.0 && duration<120.0)) return 2;
    for(i=0U;i<120U;i++)
    {
        double time=(double)i*0.01;
        if(stage4_sim_step(handle,feedback,time,0.01,command,ref)!=0)
            return 3;
        if(command[1]<-36000 || command[1]>36000)
            return 4;
        if(ref[1]<-4500 || ref[1]>-4300)
            return 5;
        /* Idealized plant follows last position command immediately.
         * This isolates C API mechanics from PyBullet actuator dynamics.
         */
        feedback[1]=command[1];
    }
    if(ref[1]!=final[1]) return 6;
    if(stage4_sim_step(handle,feedback,0.0,0.01,command,ref)==0)
        return 7;  /* time reversal forbidden */
    stage4_sim_destroy(handle);
    /*
     * The test-only C bridge must accept independent nonzero I/D gains,
     * while refusing invalid inputs before any PID state is published.
     */
    {
        double gains[6][3]={{0}};
        void *experimental;
        for(i=0U;i<6U;i++)
        {
            gains[i][0]=0.2;
            gains[i][1]=0.01;
            gains[i][2]=0.001;
        }
        experimental=stage4_sim_create_with_pid_gains(initial,final,gains);
        if(experimental==NULL) return 8;
        if(stage4_sim_step(experimental,initial,0.0,0.01,command,ref)!=0)
            return 9;
        stage4_sim_destroy(experimental);
        gains[0][1]=-0.01;
        if(stage4_sim_create_with_pid_gains(initial,final,gains)!=NULL)
            return 10;
        if(stage4_sim_create_with_pid_gains(initial,final,NULL)!=NULL)
            return 11;
    }
    { int regression=stage5_first_leg_ideal_feedback();
      if(regression!=0) return regression; }
    puts("stage4_sim_bridge_smoke passed");
    return 0;
}
