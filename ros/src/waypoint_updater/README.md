# TEAM MEMBERS
Steve Frazee - stevefrazee123@gmail.com

## Waypoint Updater (partial)
The goal of the first part of the waypoint updater is to publish a list of waypoints directly in front of the car for the car to follow. We are first given a list of waypoints that wraps around the entire track. From here we need to find the waypoint that is closest to the car and in front of the car. Once we have that, we can publish a list starting from that waypoint and include N waypoints after that. This task was broken down into 2 steps: first find the closest waypoint and second determine whether or not that waypoint is in front of or behind the car.

### Find the closest waypoint
Given the [x,y] coordinates of our car's location, we need to find the closest [x,y] waypoint. One way to do this would be a brute force search of the list of waypoints we are given. The advantage of this is that the list of waypoints is already given to us so we won't need to construct a new data structure. The disadvantage is that the list contains 10902 waypoints. Since a brute force search scales in O(N) time, we will have to perform 10000+ distance comparisons near the end of the list. Another option would be to create a KDTree from the list of waypoints. The advantage of this is that a nearest neighbor lookup scales in O(logN) time. The disadvantage is we have to construct the KDTree which takes O(NlogN) time. Since we only have to construct the KDTree once and we have to perform lookups many times per second, it makes much more sense to use a KDTree and take the penalty of construction while reaping the benefits of a much faster nearest neighbor search time.

Using the scipy.spatial library the KDTree is easily constructed once in the waypoints_cb function:
```
    def waypoints_cb(self, lane):
        ## called once on startup
        # save base waypoints as lane object
        self.base_waypoints = lane
        # create list of [x,y] waypoint positions to initialize kd_tree
        if not self.kd_tree:
            kd_tree_pts = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]
            self.kd_tree = KDTree(kd_tree_pts)
```

Now that we have a KDTree constructed we can find the closest waypoint to the car's current location by querying the tree:
```
    closest_waypoint_idx = self.kd_tree.query(self.current_pose)[1]

```

### Checking if the closest waypoint is in front or behind the car
These waypoints are eventually going to be sent to the drive by wire node to create control commands so the car can follow the waypoints. We don't want the car to try and follow a waypoint that is behind the car, so we now need to make sure that the closest waypoint we found is actually in front of the car. The method I chose to accomplish this is to compare the distance between the nearest neighbor and the previous waypoint and the distance between the car and the previous waypoint. If the car is closer to the previous waypoint than the nearest neighbor, then the nearest neighbor must be in front of the car. If the nearest neighbor is closer, then it is behind the car. In this case we can just take the next waypoint in the list to get the waypoint that is ahead of the car.

Using the euclidean function from the scipy.spatial.distance library, the distances between the car and previous waypoint and the nearest neighbor and previous waypoint are compared:
```
    def get_closest_waypoint_idx(self):
        # get index of waypoint closest to the car
        closest_waypoint_idx = self.kd_tree.query(self.current_pose)[1]
        # check if point is behind or ahead of vehicle by comparing distance to previous point
        previous_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.y]
        current_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.y]
        car_dist = euclidean(self.current_pose, previous_waypoint)
        waypoint_dist = euclidean(current_waypoint, previous_waypoint)
        # if the car is further away from the previous waypoint than the closest waypoint, then the closest waypoint is behind the car and we should take the next waypoint in the list
        if car_dist > waypoint_dist:
            closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.base_waypoints.waypoints)
        return closest_waypoint_idx
```
