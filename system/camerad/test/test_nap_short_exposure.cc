#include "system/camerad/cameras/nap_short_exposure.h"
#include <cassert>
const float G[] = {1.0,1.0625,1.125,1.1875,1.25,1.3125,1.375,1.4375,1.5,1.5625,1.6875,1.8125,1.9375,2.0,2.125,2.25,2.375,2.5,2.625,2.75,2.875,3.0,3.125,3.375,3.625,3.875,4.0,4.25,4.5,4.75,5.0,5.25,5.5,5.75,6.0,6.25,6.5,7.0,7.5,8.0,8.5,9.0,9.5,10.0,10.5,11.0,11.5,12.0,12.5,13.0,13.5,14.0,14.5,15.0,15.5};
int main(){
  // day: untouched
  auto c=nap_short_exposure(200*1.0f,200,0,504,2,G,54,1.f); assert(c.t==200&&c.g==0);
  // night 2016 lines @1x -> <=504 at ~4x, EV kept
  c=nap_short_exposure(2016,2016,0,504,2,G,54,1.f); assert(c.t<=504&&c.g==26); assert(std::abs(c.t*G[c.g]-2016)<5);
  // already near max gain: gain capped at 15.5, t shortened only as EV allows
  c=nap_short_exposure(2016*14.f,2016,51,504,2,G,54,1.f); assert(c.g==54&&c.t>504&&std::abs(c.t*G[c.g]-2016*14.f)<10);
  // gain mult limit 4x: from 2x (idx13) -> max 8x (idx39)
  c=nap_short_exposure(2016*2.f,2016,13,100,2,G,54,1.f); assert(c.g==39&&c.t==504);
  NapSeGate gt; int on=0; for(int i=0;i<200;i++) on+=gt.step(true); assert(on<=NAP_SE_MAX_ACTIVE_FRAMES*2);
  puts("ok"); }
