# CarND-Path-Planning-Project

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The simulator will provide the car's localiztion and sensor fusion data. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway.
 Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
 Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

[//]: # (Image References)
[image1]: ./img/img02.png
[image11]:./examples/car_ycrcb.png
[image2]: ./examples/hog_result.png
[image3]: ./examples/imgsub.png
[image4]: ./examples/examples.png
[image51]: ./examples/heat1.png
[image52]: ./examples/heat2.png
[image53]: ./examples/heat3.png
[image6]: ./examples/label.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.mp4

---

####The car is able to drive at least 4.32 miles without incident.

The top right screen, as shown in the below image of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. 

![alt text][image1]

####The car drives according to the speed limit.
In order to control the speed limit, the `ref_vel` was defined.
 At each cycle, the last two points in the path are used to identify the speed and continue from that value up to the maximun velocity allowed, thus avoiding sudden peeks in the rate of change of velocity.
 
```cpp
374                 double ref_vel = 0.0; 
...
461                     ref_vel = (dist / TIME_STEP) - ACCEL_VAL;
```
####Max Acceleration and Jerk are not Exceeded.
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
When calculating the length of each segment, the reference velocity is increased by `.224`, that is the maximun change in velocity. 

```cpp
554        ref_vel = ref_vel < MAX_SPEED ? ref_vel + ACCEL_VAL : MAX_SPEED;

```
###Car does not have collisions.
The car must not come into contact with any of the other cars on the road.
The following function will return the minimun distance from our car to the next car in the same lane in order to avoid collisions.

```cpp
240 vector<double> checkProximity(vector<vector<double> > *sensor_fusion,
241      double car_s, int car_lane) {
242     double best_dist = 999999;
243     double best_bk_dist = 99999;
244     double best_speed = 0;
245     int lane;
246     double dist;
247 
248     for (int i = 0; i < (*sensor_fusion).size(); i++) {
249         lane = getLane((*sensor_fusion)[i][6]);
250         if (lane == -1) continue;
251         // check only cars in the same lane
252         if (car_lane == lane) {
253             dist = (*sensor_fusion)[i][5] - car_s;
254             if (dist >= 0) {
255                 if (best_dist > dist) {
256                     best_dist = dist;
257                     best_speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
258                             (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);
259                 }
260             } else {
261                 dist = abs(dist);
262                 if (best_bk_dist > dist) {
263                     best_bk_dist = dist;
264                 }
265             }
266         }
267     }
268     return {best_dist, best_speed, best_bk_dist};
269 }

```

####The car stays in its lane, except for the time between changing lanes.
Whit the se of the spline library it was possible to interpolate segments of the path achieving correct velocity, acceleration and jerk.

```cpp
512                 tk::spline s;
513                 s.set_points(ptx, pty);

```


###The car is able to change lanes.
The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

With the use of the  `findBestLane()` function which returns the best lane according to a cost function having into account the distance to other cars in the same lane and other lanes aswell as its velocity. The program choose what is the best lane to stay at any given time during the trip.


```cpp
182 int findBestLane(vector<vector<double> > *sensor_fusion, double car_s,
183         double car_lane) {
184     vector<double> lanevalues = {0, 0, 0};
185     double dist;
186     int lane;
187     double speed;
188 
189     for (int i = 0; i < (*sensor_fusion).size(); i++) {
190         dist = (*sensor_fusion)[i][5] - car_s;
191         lane = getLane((*sensor_fusion)[i][6]);
192         speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
193                 (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);
194 
195         // cout << "Sensor value: " << (*sensor_fusion)[i][6] << " Sensor lane: " <<
196         // lane << endl;
197 
198         if (lane == -1) continue;
199 
200         if (dist > 0) {
201             lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
202         } else {
203             dist = abs(dist);
204             if (dist <= 10) {
205                 dist = 10;
206                 lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
207             }
208         }
209     }


```




