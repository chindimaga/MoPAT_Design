#include <stdio.h>
#include <math.h>
#include <limits.h>


long calc_dist(int x, int x1, int y, int y1){
  return (x-x1)*(x-x1)+(y-y1)*(y-y1);
}

int main(){
  float Lf;
  float k= 0.5;
  float Lfc= 0.02;
  float Kp= 0.3;
  float dt= 0.1;
  float L = 2.9;
  int bot_x;
  int bot_y;
  float bot_yaw;
  float bot_v;
  float time=0.0;
  int target_ind = 0;
  int old_nearest_point_index = 0;
  float alpha,delta;
  int tx,ty;

  // target course

  int path_x[] = {53,53,52,51,50,49,48,47,46,45,44,43,43,43,43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3};
  int path_y[] = {50,49,48,47,46,45,44,43,42,41,40,39,38,37,36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 21, 22, 23, 24, 25, 26, 27, 28, 28, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
  float target_speed = 10.0/3.6;

  // initial state
  bot_x=53;
  bot_y=50;
  bot_yaw = 0.0;
  bot_v = 0.0;
  int last_index = (sizeof (path_x) / sizeof (path_x[0]))-1;
  int len;

  // search target lastIndex
  // printf("%d\n", 1);
  int i;
  long distance = LONG_MAX;
  long d1;
  long d2;
  int ind;
  for(i=0;i<=last_index;i++){
    d1=calc_dist(path_x[i], bot_x, path_y[i], bot_y);
    if(d1<distance){
      distance=d1;
      ind=i;
    }
  }
  // printf("%d\n", ind);
  old_nearest_point_index= ind;
  L=0.0;
  Lf = k * bot_v + Lfc;
  while(Lf>L && ind+1 < (sizeof path_x / sizeof path_x[0])){
    L = calc_dist(path_x[ind], bot_x, path_y[ind], bot_y);
    ind +=1;
  }

  target_ind = ind;

  float a;
  // printf("%d\t%d\n", last_index,target_ind);
//feedback condition can be added
  while(last_index > target_ind){

    // PIDControl
    a = Kp * (target_speed - bot_v);


    // pure_pursuit_control

    // search index
    ind = old_nearest_point_index;
    d1 = calc_dist(path_x[ind],bot_x,path_y[ind],bot_y);
    while(1){
      len = (sizeof path_x / sizeof path_x[0]);
      if(ind+1<len) ind++;
      d2 = calc_dist(path_x[ind],bot_x,path_y[ind],bot_y);
      if (d1 < d2) break;
      d1=d2;
    }
    old_nearest_point_index= ind;
    L=0.0;
    Lf = k * bot_v + Lfc;
    while(Lf>L && ind+1 < (sizeof path_x / sizeof path_x[0])){
      L = calc_dist(path_x[ind], bot_x, path_y[ind], bot_y);
      ind +=1;
    }
    //

    if (target_ind >= ind) ind = target_ind;

    if (ind >= len) ind = len-1;
    tx = path_x[ind];
    ty = path_y[ind];

    alpha = atan2(ty - bot_y, tx - bot_x) - bot_yaw;

    Lf = k * bot_v + Lfc;

    delta = atan2(2.0 * L * sin(alpha) / Lf,1.0);

    target_ind= ind;

    // update

    bot_x += bot_v * cos(bot_yaw) * dt;
    bot_y += bot_v * sin(bot_yaw) * dt;
    bot_yaw += bot_v / L * tan(delta) * dt;
    bot_v += a * dt;
    time += dt;
    printf("x: %d y: %d yaw: %f v: %f\n",bot_x,bot_y, bot_yaw, bot_v );
  }
  printf("%d",1);
  return 0;

}
