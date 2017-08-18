#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define TIME_STEP 0.02
#define MAX_SPEED  22.0
#define MPH_TO_MTS 0.44704
#define LATENCY 60e-3
#define MAX_SPACE 0.4  // mts
#define NUMBER_LANES 3
#define ACCEL_VAL 0.10
#define MIN_DIST 30
#define MIN_MIN_DIST 10

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double euclidean_distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x,
                    vector<double> maps_y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = euclidean_distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x,
                 vector<double> maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         vector<double> maps_x, vector<double> maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = euclidean_distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = euclidean_distance(center_x, center_y, x_x, x_y);
  double centerToRef = euclidean_distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s +=
        euclidean_distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += euclidean_distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

int getLane(double car_d) {
  int currentLane = -1;

  if (car_d > 0 && car_d <= 4)
    currentLane = 0;
  else if (car_d > 4 && car_d <= 8)
    currentLane = 1;
  else if (car_d > 8 && car_d <= 12)
    currentLane = 2;

  return currentLane;
}


/* ! \fn int findBestLane(vector<vector<double> > *sensor_fusion, double car_s,
                 double car_lane) {
     \brief Return best lane based in the proximity of other cards
     \param sensor_fusion vector with sensor fusion data
     \param car_s s position of ego car
     \param car_lane lane of current car position
 */
int findBestLane(vector<vector<double> > *sensor_fusion, double car_s,
                 double car_lane) {
  vector<double> lanevalues = {0, 0, 0};
  double dist;
  int lane;
  double speed;

  for (int i = 0; i < (*sensor_fusion).size(); i++) {
    dist = (*sensor_fusion)[i][5] - car_s;
    lane = getLane((*sensor_fusion)[i][6]);
    speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
                 (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);

    // cout << "Sensor value: " << (*sensor_fusion)[i][6] << " Sensor lane: " <<
    // lane << endl;

    if (lane == -1) continue;

    if (dist > 0) {
      lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
    } else {
      dist = abs(dist);
      if (dist <= 10) {
        dist = 10;
        lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
      }
    }
  }

  // double lane_weight = (lanevalues[0] + lanevalues[1] + lanevalues[2]) / 3.0;
  //
  // if (car_lane == 0) {
  //   lanevalues[1] += lane_weight;
  //   lanevalues[2] += lane_weight + lane_weight;
  // } else if (car_lane == 1) {
  //   lanevalues[0] += lane_weight;
  //   lanevalues[2] += lane_weight;
  // } else {
  //   lanevalues[1] += lane_weight;
  //   lanevalues[0] += lane_weight + lane_weight;
  // }

  cout << "lane values: " << lanevalues[0] << ", " << lanevalues[1] << ", "
       << lanevalues[2] << endl;

  vector<double>::iterator result =
      min_element(begin(lanevalues), end(lanevalues));
  return std::distance(begin(lanevalues), result);
}

/* ! \fn vector<double> checkProximity(vector<vector<double> > * sensor_fusion,
                                        double car_s, int car_lane)
     \brief Return how close is and the speed of the car in front of us
     \param sensor_fusion vector with sensor fusion data
     \param car_s s position of ego car
     \param car_lane lane of current car position
 */

vector<double> checkProximity(vector<vector<double> > *sensor_fusion,
                              double car_s, int car_lane) {
  double best_dist = 999999;
  double best_bk_dist = 99999;
  double best_speed = 0;
  int lane;
  double dist;

  for (int i = 0; i < (*sensor_fusion).size(); i++) {
    lane = getLane((*sensor_fusion)[i][6]);
    if (lane == -1) continue;
    // check only cars in the same lane
    if (car_lane == lane) {
      dist = (*sensor_fusion)[i][5] - car_s;
      if (dist >= 0) {
        if (best_dist > dist) {
          best_dist = dist;
          best_speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
                            (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);
        }
      } else {
        dist = abs(dist);
        if (best_bk_dist > dist) {
          best_bk_dist = dist;
        }
      }
    }
  }
  return {best_dist, best_speed, best_bk_dist};
}

/*********************
 * main routine
 *********************/

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          vector<vector<double> > sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds

          /* CODE PROJECT START */

          int pre_path_size = previous_path_x.size();
          vector<double> ptx, pty;
          double ref_car_yaw = deg2rad(car_yaw);
          double end_path_x;
          double end_path_y;
          double end_path_yaw;
          int end_path_lane;
          int lane;
          bool slowdown = false;
          bool switch_lane = false;
          double front_car_ref_vel;

          car_speed = car_speed * MPH_TO_MTS;  //  convert to meter per sec
          double ref_vel = 0.0;                // MAX_SPEED;

          // if size of previous path is less than 2
          // then use current car state to generate points
          if (pre_path_size < 2) {
            double prev_car_x = car_x - cos(ref_car_yaw);
            double prev_car_y = car_y - sin(ref_car_yaw);

            end_path_x = car_x;
            end_path_y = car_y;
            end_path_s = car_s;
            end_path_d = car_d;
            end_path_yaw = ref_car_yaw;

            ptx.push_back(prev_car_x);
            ptx.push_back(car_x);

            pty.push_back(prev_car_y);
            pty.push_back(car_y);

          } else {
            // identify current lane
            lane = getLane(car_d);

            vector<double> prx = checkProximity(&sensor_fusion, car_s, lane);

            int best_lane = findBestLane(&sensor_fusion, car_s, lane);
            cout << "***Best lane: " << best_lane << endl;

            if (best_lane == lane && prx[0] < MIN_DIST) {
              cout << "too close, slowdown now !!!\n";
              slowdown = true;
              if (prx[0] <= MIN_MIN_DIST) {
                front_car_ref_vel = prx[1] - ACCEL_VAL;
              } else {
                front_car_ref_vel = prx[1];
              }

            } else {
              if (abs(lane - best_lane) != 2) {
                lane = best_lane;
                slowdown = false;
              } else {
                  // check proximity of next lane
                  vector<double> next_prx = checkProximity(&sensor_fusion, car_s, 1);

                  if (next_prx[0] >= MIN_DIST && next_prx[2] >= MIN_DIST) {
                    lane = 1;
                  } else {

                if (prx[0] < MIN_DIST) {
                  cout << "too close, slowdown now due to double shift !!!\n";
                  slowdown = true;

                  if (prx[0] <= MIN_MIN_DIST) {
                    front_car_ref_vel = prx[1] - ACCEL_VAL;
                  } else {
                    front_car_ref_vel = prx[1];
                  }

                } else {
                  slowdown = false;
                }
              }
            }
            }
            // then use previous path last two points
            end_path_x = previous_path_x[pre_path_size - 1];
            end_path_y = previous_path_y[pre_path_size - 1];

            double prev_end_path_x = previous_path_x[pre_path_size - 2];
            double prev_end_path_y = previous_path_y[pre_path_size - 2];

            end_path_yaw = atan2(end_path_y - prev_end_path_y,
                                 end_path_x - prev_end_path_x);

            ptx.push_back(prev_end_path_x);
            ptx.push_back(end_path_x);

            pty.push_back(prev_end_path_y);
            pty.push_back(end_path_y);

            // define reference velocity
            double dist = sqrt((end_path_x - prev_end_path_x) *
                                   (end_path_x - prev_end_path_x) +
                               (end_path_y - prev_end_path_y) *
                                   (end_path_y - prev_end_path_y));
            ref_vel = (dist / TIME_STEP) - ACCEL_VAL;
          }
          // identify current lane and end lane

          // end_path_lane = getLane(end_path_d);
          // generate additional points

          vector<double> wp0 =
              getXY(end_path_s + 30, 2 + 4 * lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> wp1 =
              getXY(end_path_s + 60, 2 + 4 * lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> wp2 =
              getXY(end_path_s + 90, 2 + 4 * lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> wp3 =
              getXY(end_path_s + 120, 2 + 4 * lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          // add new points to points
          ptx.push_back(wp0[0]);
          ptx.push_back(wp1[0]);
          ptx.push_back(wp2[0]);
          ptx.push_back(wp3[0]);

          pty.push_back(wp0[1]);
          pty.push_back(wp1[1]);
          pty.push_back(wp2[1]);
          pty.push_back(wp3[1]);

          // add previuos path points to next values
          for (int i = 0; i < pre_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // rotate points to CAR coordinates
          //
          for (int i = 0; i < ptx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptx[i] - end_path_x;
            double shift_y = pty[i] - end_path_y;

            ptx[i] = (shift_x * cos(0 - end_path_yaw) -
                      shift_y * sin(0 - end_path_yaw));
            pty[i] = (shift_x * sin(0 - end_path_yaw) +
                      shift_y * cos(0 - end_path_yaw));
          }

          // create spline
          tk::spline s;
          s.set_points(ptx, pty);

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          //  ref_vel = car_speed;

          for (int i = 0; i < 80 - pre_path_size; i++) {
            double N = target_dist / (TIME_STEP * ref_vel);
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);

            x_add_on = x_point;

            // rotate back to global cooridinates

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(end_path_yaw) - y_ref * sin(end_path_yaw));

            y_point = (x_ref * sin(end_path_yaw) + y_ref * cos(end_path_yaw));

            x_point += end_path_x;
            y_point += end_path_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

            if (slowdown == true) {
              // ref_vel = ref_vel - ACCEL_VAL;
              if (ref_vel > front_car_ref_vel) {
                ref_vel -= ACCEL_VAL;
              } else {
                ref_vel += ACCEL_VAL;
              }

            } else {
              ref_vel = ref_vel < MAX_SPEED ? ref_vel + ACCEL_VAL : MAX_SPEED;
            }
          }

          /* CODE PROJECT END */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
