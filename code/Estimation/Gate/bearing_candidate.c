#include "bearing_candidate.h"
#include "bearing_model.h"
#include <math.h>
#include <string.h>
#define RADF (0.01745329251994329577f)
#define DEGF (57.29577951308232088f)
static float mod180f_local(float x){x=fmodf(x+90,180);if(x<0)x+=180;return x-90;}
static float delta360(float x){x=fmodf(x+180,360);if(x<0)x+=360;return x-180;}
static float response(unsigned ch,float q,float *derivative){
 unsigned i;float dx;const float *p;
 if(ch>=4||!isfinite(q)){if(derivative)*derivative=NAN;return NAN;}
 if(q<bc_fast_knots[ch][0]){if(derivative)*derivative=0;return 0;}
 if(q>bc_fast_knots[ch][6]){if(derivative)*derivative=bc_fast_end_derivative[ch];return bc_fast_values[ch][6]+bc_fast_end_derivative[ch]*(q-bc_fast_knots[ch][6]);}
 for(i=0;i<5&&q>=bc_fast_knots[ch][i+1];i++){}
 dx=q-bc_fast_knots[ch][i];p=bc_fast_coefficients[ch][i];
 if(derivative)*derivative=(3*p[0]*dx+2*p[1])*dx+p[2];
 return ((p[0]*dx+p[1])*dx+p[2])*dx+p[3];
}
double BC_Response(unsigned ch,double q,double *derivative){float d=0;float value=response(ch,(float)q,derivative?&d:0);if(derivative)*derivative=d;return value;}
static float inverse(unsigned ch,float adc,int envelope){
 unsigned i,k;float value=adc-bc_fast_background[ch],lo,hi,mid;
 if(value<=0)return 0;
 if(value>bc_fast_values[ch][6]){
  float d=bc_fast_end_derivative[ch];
  if(!(d>1e-6f))return envelope?bc_fast_knots[ch][6]+1:NAN;
  return bc_fast_knots[ch][6]+(value-bc_fast_values[ch][6])/d;
 }
 for(i=0;i<5&&value>=bc_fast_values[ch][i+1];i++){}
 lo=bc_fast_knots[ch][i];hi=bc_fast_knots[ch][i+1];
 /* Monotone calibrated cubic; fixed bounded work, no root iteration to convergence. */
 for(k=0;k<16;k++){mid=.5f*(lo+hi);if(response(ch,mid,0)<value)lo=mid;else hi=mid;}
 return .5f*(lo+hi);
}
double BC_InverseResponse(unsigned ch,double adc){if(ch>=4||!isfinite(adc))return NAN;return inverse(ch,(float)adc,0);}
static void level_rotation(float pitch,float roll,float R[9]){
 float cp=cosf(pitch*RADF),sp=sinf(pitch*RADF),cr=cosf(roll*RADF),sr=sinf(roll*RADF);
 R[0]=cr;R[1]=0;R[2]=sr;R[3]=sp*sr;R[4]=cp;R[5]=-sp*cr;R[6]=-cp*sr;R[7]=sp;R[8]=cp*cr;
}
static void from_level(float pitch,float roll,const float level[3],float body[3]){
 float cp=cosf(pitch*RADF),sp=sinf(pitch*RADF),cr=cosf(roll*RADF),sr=sinf(roll*RADF);
 body[0]=cr*level[0]+sp*sr*level[1]-cp*sr*level[2];
 body[1]=cp*level[1]+sp*level[2];
 body[2]=sr*level[0]-sp*cr*level[1]+cp*cr*level[2];
}
static void predict_body(const float body[3],double prediction[4]){
 unsigned ch;for(ch=0;ch<4;ch++){
  float q=fabsf(bc_fast_axes[ch][0]*body[0]+bc_fast_axes[ch][1]*body[1]+bc_fast_axes[ch][2]*body[2]);
  prediction[ch]=bc_fast_background[ch]+response(ch,q,0);
 }
}
void BC_ForwardVector(const double input[3],double pitch,double roll,double prediction[4]){
 float level[3],body[3];unsigned i;for(i=0;i<3;i++)level[i]=(float)input[i];from_level((float)pitch,(float)roll,level,body);predict_body(body,prediction);
}
void BC_Forward(double beta,double amplitude,double pitch,double roll,double prediction[4]){
 double level[3]={amplitude*sinf((float)beta*RADF),amplitude*cosf((float)beta*RADF),0};BC_ForwardVector(level,pitch,roll,prediction);
}
static int separated(const float a[3],const float b[3]){
 float aa=a[0]*a[0]+a[1]*a[1],bb=b[0]*b[0]+b[1]*b[1],dot=a[0]*b[0]+a[1]*b[1];
 return aa>1e-12f&&bb>1e-12f&&dot*dot<.9924038765f*aa*bb; /* cos(5deg)^2 */
}
static void publish(BC_Solver *s,unsigned best){
 BC_Result *r=&s->result;unsigned i,k,alt=BC_STARTS;float norm,err=0,energy=0;
 r->extrapolated_mask=0;r->flags&=~(uint32_t)(BC_BOUND|BC_EXTRAPOLATED|BC_FIELD_INCOHERENT);
 s->best=best;norm=hypotf(s->level[best][0],s->level[best][1]);
 r->bearing_deg=mod180f_local(atan2f(s->level[best][0],s->level[best][1])*DEGF);r->field_norm=norm;r->cost=s->costs[best];
 predict_body(s->body[best],r->prediction);
 for(i=0;i<4;i++){
  float e=(float)r->prediction[i]-s->observed[i],q=fabsf(bc_fast_axes[i][0]*s->body[best][0]+bc_fast_axes[i][1]*s->body[best][1]+bc_fast_axes[i][2]*s->body[best][2]);
  err+=e*e;energy+=s->observed[i]*s->observed[i];if(q>bc_fast_knots[i][6])r->extrapolated_mask|=(uint8_t)(1U<<i);
 }
 r->adc_error_percent=100*sqrtf(err)/fmaxf(sqrtf(energy),1);
 if(r->extrapolated_mask)r->flags|=BC_EXTRAPOLATED;
 if(fabs(r->bearing_deg)>89.99||norm>10)r->flags|=BC_BOUND;
 if(!(norm>=.02f)){r->flags|=BC_NO_SIGNAL;r->available=r->success=0;r->bearing_deg=NAN;return;}
 for(k=0;k<BC_STARTS;k++)if(k!=best&&separated(s->level[k],s->level[best])&&(alt==BC_STARTS||s->costs[k]<s->costs[alt]))alt=k;
 r->alternative_deg=r->alternative_gap=r->alternative_separation=NAN;
 r->flags&=~(uint32_t)(BC_ALTERNATIVE|BC_SIGN_UNCERTAIN);
 if(alt<BC_STARTS){r->flags|=BC_ALTERNATIVE;r->alternative_deg=mod180f_local(atan2f(s->level[alt][0],s->level[alt][1])*DEGF);
  r->alternative_gap=s->costs[alt]-s->costs[best];r->alternative_separation=fabsf(mod180f_local((float)r->alternative_deg-(float)r->bearing_deg));
  if(fabs(r->alternative_gap)<=1)r->flags|=BC_SIGN_UNCERTAIN;
 }
 if(r->cost>25)r->flags|=BC_FIELD_INCOHERENT;
 r->success=r->available=1;r->evaluations=BC_STARTS;r->iterations=0;
}
void BC_Begin(BC_Solver *s,const double adc[4],double pitch,double roll){
 unsigned i,k,j,best=0;float denominator=0,R[9];memset(s,0,sizeof(*s));s->done=1;s->result.flags=BC_LINEAR_FAST;
 s->result.bearing_deg=s->result.field_norm=s->result.cost=s->result.adc_error_percent=NAN;
 s->result.alternative_deg=s->result.alternative_gap=s->result.alternative_separation=NAN;
 for(i=0;i<4;i++)s->result.prediction[i]=NAN;
 if(!isfinite(pitch)||!isfinite(roll)){s->result.flags|=BC_INPUT_BAD;return;}
 for(i=0;i<4;i++){
  float noise,upper,lower;
  if(!isfinite(adc[i])){s->result.flags|=BC_INPUT_BAD;return;}
  s->observed[i]=(float)adc[i];if(adc[i]<0||adc[i]>4095)s->result.flags|=BC_ADC_RANGE;
  if(adc[i]>=4090)s->result.saturated_mask|=(uint8_t)(1U<<i);
  if(i&&adc[i]-bc_fast_background[i]<5)s->result.weak_mask|=(uint8_t)(1U<<i);
  s->projection[i]=inverse(i,s->observed[i],0);noise=fmaxf(5,.015f*s->observed[i]);
  upper=inverse(i,s->observed[i]+noise,1);lower=inverse(i,s->observed[i]-noise,1);
  s->sigma[i]=fmaxf(1e-5f,.5f*(upper-lower));denominator+=bc_fast_null[i]*bc_fast_null[i]*s->sigma[i]*s->sigma[i];
  if(!isfinite(s->projection[i])||!isfinite(denominator)){s->result.flags|=BC_NUMERIC;return;}
 }
 if(s->result.saturated_mask)s->result.flags|=BC_SATURATED;
 if(s->result.weak_mask)s->result.flags|=BC_WEAK;
 if(s->result.weak_mask==14){s->result.flags|=BC_NO_SIGNAL;return;}
 if(s->result.flags&BC_ADC_RANGE)return;
 level_rotation((float)pitch,(float)roll,R);
 for(k=0;k<BC_STARTS&&k<BC_LINEAR_SIGN_LIMIT;k++){
  float signed_q[4],residual=0;
  signed_q[0]=s->projection[0];for(i=1;i<4;i++)signed_q[i]=(k&(1U<<(i-1)))?s->projection[i]:-s->projection[i];
  for(i=0;i<4;i++)residual+=bc_fast_null[i]*signed_q[i];
  s->costs[k]=residual*residual/fmaxf(denominator,1e-10f);
  for(j=0;j<3;j++)for(i=0;i<4;i++)s->body[k][j]+=bc_fast_inverse[j][i]*signed_q[i];
  for(j=0;j<3;j++)s->level[k][j]=R[j*3]*s->body[k][0]+R[j*3+1]*s->body[k][1]+R[j*3+2]*s->body[k][2];
  if(s->level[k][1]<0)for(j=0;j<3;j++){s->body[k][j]=-s->body[k][j];s->level[k][j]=-s->level[k][j];}
  if(k&&s->costs[k]<s->costs[best])best=k;
  s->start++;s->total_evaluations++;
 }
 if(s->start!=BC_STARTS){s->result.flags|=BC_LIMIT;s->result.evaluations=s->start;return;}
 publish(s,best);
}
int BC_MathStep(BC_Solver *s){return s->done;}
void BC_Run(const double adc[4],double pitch,double roll,BC_Result *r){BC_Solver s;BC_Begin(&s,adc,pitch,roll);*r=s.result;}
void BC_TrackApply(BC_Solver *s,BC_Track *track,float yaw,uint32_t stamp){
 unsigned k,best=s->best;float dt,dy=0,pred=0,pa=0,pb=0,scale=1,score_min=INFINITY,raw_min=s->costs[s->best];int eligible=0;
 if(!s->result.available||!isfinite(yaw)){track->initialized=0;return;}
 dt=(uint32_t)(stamp-track->timestamp_us)*1e-6f;
 if(track->initialized&&dt>=.001f&&dt<=.05f){
  dy=delta360(yaw-track->yaw_deg);
  if(fabsf(dy)<=720*dt+1){pred=track->angle_deg-dy+track->world_rate_dps*dt;pa=sinf(pred*RADF);pb=cosf(pred*RADF);scale=sinf((3+40*dt)*RADF);scale=scale*scale;eligible=1;}
 }
 if(eligible){
  for(k=0;k<BC_STARTS;k++)if(s->costs[k]<=raw_min+4){
   float a=s->level[k][0],b=s->level[k][1],n=a*a+b*b,cross=a*pb-b*pa,dot=a*pa+b*pb,score;
   if(n<.0004f||dot*dot<.5f*n)continue;
   score=s->costs[k]+cross*cross/(n*scale);
   if(score<score_min){score_min=score;best=k;}
  }
  if(best!=s->best){s->result.flags|=BC_TRACKED_BRANCH;publish(s,best);}
 }
 if(track->initialized&&eligible){float rate=(mod180f_local((float)s->result.bearing_deg-track->angle_deg)+dy)/dt;rate=fmaxf(-360,fminf(360,rate));track->world_rate_dps+=.2f*(rate-track->world_rate_dps);}else track->world_rate_dps=0;
 track->angle_deg=(float)s->result.bearing_deg;track->yaw_deg=yaw;track->timestamp_us=stamp;track->initialized=1;
}
