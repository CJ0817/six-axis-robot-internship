#include "robot_planning.h"
#include <math.h>
#include <string.h>
/* 仅几何路径，不做IK/碰撞；由上层提供冻结的base系几何参数。 */
int robot_path_position(const robot_cartesian_path *p, double s,
                        double *position, double *first, double *second, size_t count)
{
    double q[3],v[3],a[3];
    if (!p || !position || !first || !second || count!=3 || !isfinite(s) || s<0 || s>1) return 1001;
    if (p->method!=1 && p->method!=2) return 1008;
    double uu=0,vv=0,uv=0;
    for (size_t i=0;i<3;++i) {
        if (!isfinite(p->origin[i]) || !isfinite(p->u[i]) || !isfinite(p->v[i])) return 1001;
        uu+=p->u[i]*p->u[i];vv+=p->v[i]*p->v[i];uv+=p->u[i]*p->v[i];
    }
    if (p->method==2 && (!isfinite(p->radius) || p->radius<=0 || !isfinite(p->sweep) ||
        fabs(p->sweep)<=1e-12 || fabs(p->sweep)>=2*acos(-1.0) ||
        fabs(uu-1)>1e-9 || fabs(vv-1)>1e-9 || fabs(uv)>1e-9)) return 1001;
    for (size_t i=0;i<3;++i) {
        if (p->method==1) { q[i]=p->origin[i]+s*p->u[i];v[i]=p->u[i];a[i]=0; }
        else {
            double c=cos(p->sweep*s),z=sin(p->sweep*s);
            q[i]=p->origin[i]+p->radius*(p->u[i]*c+p->v[i]*z);
            v[i]=p->radius*p->sweep*(-p->u[i]*z+p->v[i]*c);
            a[i]=-p->radius*p->sweep*p->sweep*(p->u[i]*c+p->v[i]*z);
        }
        if (!isfinite(q[i]) || !isfinite(v[i]) || !isfinite(a[i])) return 1001;
    }
    memcpy(position,q,sizeof q);memcpy(first,v,sizeof v);memcpy(second,a,sizeof a);return 0;
}
