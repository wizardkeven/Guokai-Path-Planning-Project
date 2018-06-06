To calculate a possible end point when changing lane, we can depart from calculating the shortest lane change time dt. We assume at one point we want to change lane
 to either left lane or right lane if either exist. Also suppose the ego vehicle is running in the center of its current lane. As the lane is
 4m wide, we take 2m as an averaged vehicle width, so it means the distance from the edge side of the ego vehicle to lane lines of both side is
 about 1m. Of course the lateral distance for changing lane should also include the d value of the possible colliding vehicle in this lane. But 
 for a safe lane changing, I decide to not take this d value into consideration. Besides we can not only think about the just right non-collision
 situation, we should also consider a safty-zone for the possible lane changing action. Here I take a buffer distance 6 meters from the possible 
 colliding vehicle both ahead and back side. So in all, a safe lane changing should be that the target point of lane changing trajectory should 
 be at least 6 meters away from any vehicle in this lane at the future timestamp delta_t. Then using this point, we can generate a polynomial 
 trajectory with spline. Before all these happened, we must have a reasonalbe predictions for vehicles received from the sensor fusion. Then
 the important point is to get a proper delta_t as the ideal target point reaching timestamp. 
 
 Remember the maximum jerk should not pass 10 m/s_3(here we take 7 for comfort consideration), with the formular to calculate distance with acceleration, s_t+1 = s_t + v * t + a * t*t / 2,
 assuming the initial velocity and initial lateral acceleration are all 0, lateral distance is 1m(reasoned before),  we can get possible delta_t is 
 about 0.53 s. With this starting time value, we can then calculate the shortest distance to get to the possible lane changing target point,
 s_lane_change = s_ego_car + 0.53 * v_longitude + distant_bufferred  = s_ego_car + 0.53 * v_longitude + 6. Based on this value, we can calculate 
 if there would be a possible collision on this lane. If not, we can start to change lane; otherwise, we have to keep lane and wait for this 
 vehicle passing by.
