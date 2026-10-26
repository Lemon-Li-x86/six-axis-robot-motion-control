/*
 * Headless native bridge smoke test. Does not require Python or PyBullet.
 * Ensures the shared C boundary invokes the actual trajectory, PID
 * and command limiter and rejects invalid calls atomically.
 */
#include <stdint.h>
#include <stdio.h>

extern void *stage4_sim_create(const int32_t *,const int32_t *);
extern double stage4_sim_duration(const void *);
extern int stage4_sim_step(void *,const int32_t *,double,double,int32_t *,int32_t *);
extern void stage4_sim_destroy(void *);

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
        if(command[1]<-4500 || command[1]>-4300)
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
    puts("stage4_sim_bridge_smoke passed");
    return 0;
}
