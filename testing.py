from geometry_msgs.msg import Twist
import rclpy
from gpiozero import LED
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import atexit                     # Detect KeyboardInterrupt
from math import isnan
from math import isinf
from collections import deque
import numpy as np
try:
    import smbus2 as smbus        # (Has better PEC-support)
except ImportError:
    # Try old system-binding as reserver
    import smbus
import time

from datetime import datetime
import pandas as pd

class Turtlebot3ObstacleDetection(Node):
    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('\n##############################################')
        print('##################### MOB ####################')
        print('############# MOVE THE FUCK AWAY #############')
        print('##############################################\n')
    # -----------------------------------------------------------------------
    # Initialize
    # -----------------------------------------------------------------------
        self.scan_ranges = [] # Prepare to store incoming LaserScan data and track if we've received any
        self.has_scan_received = False
        # Set up QoS, Publishers, Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback) # Create a timer that calls `self.timer_callback` every 0.2 seconds.
        #RGB
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x44, 0x01, 0x05)
        self.last_color = None
        self.last_color_time = 0.0
        # LED
        self.victim_led = LED(17)
        self.led_on = False
        self.led_counter = 0
        
    # -----------------------------------------------------------------------
    # Logging
    # -----------------------------------------------------------------------
        self.metric_linear = [] # Store linear speeds in an array
        self.metric_angular = [] # Store angular speeds in an array
        self.metric_collisions = 0 # Store collision count
        self.metric_victims_found = 0 # Store victim count
        self.collisioncounter = 0 # Counter used in collisiontracking
        self.collisioncounter2 = 0 # Counter used in collisiontracking for reversing
        self.collision_timer = 100 # Amount of cycles used for ignoring collisions after a collision
        print("Robot initialized. Metrics will be displayed when shutting down.") # Print to terminal
        atexit.register(self._print_final_metrics) # Register shutdown print

        self.metric_log = []
        self.start_time = time.time()
        self.run_duration = 121  # seconds
        self.last_log_time = self.start_time
        self.latest_lin_speed = 0.0

    # -----------------------------------------------------------------------
    # Variables
    # -----------------------------------------------------------------------
        # Distances
        self.distance_critical = 0.15 # Distance before something is considered in critical range
        self.distance_start_turning = 0.33 # Distance to start turning
        self.distance_is_trapped = 0.20 # Distance to determine wether trapped
        self.distance_sidescore = 0.3 # Distance for sidescores

        # Speeds
        self.speed_linear_max = 0.20 # Max linear speed
        self.speed_angular_max = 2.0 # Max angular speed

        # Factors
        self.is_blocked_percentage = 0.5 # Percentage to be blocked for returning true
        self.side_score_value_norm_factor = 0.8 # Side score factor for normalized value
        self.side_score_value_avg_factor = 0.2 # Side score factor for average value
        self.speed_factor = 1.1 # Multiply speed

        # Thresholds
        self.compare_score_pairs_threshold = 0.3 # Threshold before something is a good pair
        self.compare_score_pairs_hysteresis = 0.05 # Buffer value of score pairs
        self.speed_factor_threshold = 1 # Max angular speed for speed_factor to be applied

        # Segments: adjust these to get the exact indices you want!
        self.total_segments = 32 # total segments when splitting readings

        # Filter ranges
        self.min_range = 0.15 # remove any reading below
        self.max_range = 3.5 # remove any reading above

        # LED
        self.led_cycles = 12 # number of cycles led will be on

        # History
        self.history = { # used as readings for invalid readings
            'sl0': deque(maxlen=5),
            'sl1': deque(maxlen=5),
            'sl2': deque(maxlen=5),
            'sl3': deque(maxlen=5),
            'sl4': deque(maxlen=5),
            'sr0': deque(maxlen=5),
            'sr1': deque(maxlen=5),
            'sr2': deque(maxlen=5),
            'sr3': deque(maxlen=5),
            'sr4': deque(maxlen=5),
            'front': deque(maxlen=5),
            'front_left': deque(maxlen=5),
            'front_right': deque(maxlen=5),

        }
        self.dominating_color = None
        
        # Segments
        self.segments = {
            # main groups
            'front'       : [0, 1, -1, -2],
            'front_left'  : [2, 3, 4, 5],
            'front_right' : [-3, -4, -5, -6],

            # fine slices
            'sl0' : [0, 1],
            'sl1' : [1, 2, 3],
            'sl2' : [3, 4, 5],
            'sl3' : [5, 6, 7],
            'sl4' : [7, 8],
            'sr0' : [-1, -2],
            'sr1' : [-2, -3, -4],
            'sr2' : [-4, -5, -6],
            'sr3' : [-6, -7, -8],
            'sr4' : [-8, -9],

            # collision zone (everything directly in front)
            'collision': [0, 1, 2, 3, 4, -1, -2, -3, -4, -5],
        }
    # -----------------------------------------------------------------------
    # States
    # -----------------------------------------------------------------------
        self.collision_recent = False # Wether a recent collision has occured
        self.prev_direction = 'FORWARD' # Used to store previous direction, initialized to forward

# -----------------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def scan_callback(self, msg): # Call back for range scans
        self.scan_ranges = msg.ranges
        self.has_scan_received = True
    def cmd_vel_raw_callback(self, msg): # Call back
        self.tele_twist = msg
    def timer_callback(self): # Call back for every time cycle
        if time.time() - self.start_time > self.run_duration:
            print("⏱️ 1 minute has passed. Stopping robot and saving metrics...")
            self.stop_robot()
            self.export_metrics_to_excel()
            rclpy.shutdown()
            return
        
        if self.has_scan_received:
            self.detect_obstacle()
            self.log_metrics_each_second()

    # -----------------------------------------------------------------------
    # Functions
    # -----------------------------------------------------------------------

    def filter_ranges(self ,segments): # Filter out data outside of range
        if not segments: # If array is empty, return as is
            return segments
        sectors = [[float(v) # filter outside of range, nan, and inf
                    for v in seg
                    if self.min_range <= v <= self.max_range and not isnan(v) and not isinf(v)]
                for seg in segments]
        return sectors

    def split_segments(self, data): # Split LiDAR readings into equal segments
        # --- slice -------------------------------------------------------------
        segments = np.array_split(data, self.total_segments) # split data into array of arrays
        sectors = self.filter_ranges(segments) # filter out unwanted data

        distances = {} # init dictionary
        for segment_name, indecies in self.segments.items(): # create segments from dictionary in _init_
            current = []
            for index in indecies:
                current.extend(sectors[index]) # combines all indecis in group to readings
            distances[segment_name] = current # returns combined group to current group

        return distances # returns array of groups

    def dynamic_lin_speed(self, distance):
        # Check if elements are valid
        valid = [r for r in distance if not isnan(r) and not isinf(r)]
        # If no valid data, use history
        if not valid:
            valid = self.history['front']
        
        min_dist = min(valid) # find smallest distance

        if min_dist <= self.distance_critical: # stop for critical distance
            return 0.0
        elif min_dist >= self.distance_start_turning: # no need for turning, go max speed
            return self.speed_linear_max
    
        # Linear interpolation, computes ratio for linear speed
        ratio = (min_dist - self.distance_critical) / (self.distance_start_turning - self.distance_critical)
        return ratio * self.speed_linear_max # return max speed multiplied by ratio

    def dynamic_ang_speed(self, distance):
        # Check if elements are valid
        valid = [r for r in distance if not isnan(r) and not isinf(r)]
        # If no valid data, return 0.0
        if not valid:
            valid = self.history['front']
        # If min valid data is less than critical distance, max speed
        elif min(valid) <= self.distance_critical: # below critical, turn max speed
            return self.speed_angular_max
        elif min(valid) >= self.distance_start_turning: # if min distance is above turning; dont turn
            return 0.0

        # Calculate ratio when between critical and turning distance
        else: # linear interpolation
            ratio = np.clip((self.distance_start_turning - min(valid)) / (self.distance_start_turning - self.distance_critical ), 0.0, 1.0)
            return ratio * self.speed_angular_max

    def is_blocked(self, front, safe_distance): # Helper function, used in is_trapped()
        # Compute amount of elements that have to be blocked
        threshold = len(front) * self.is_blocked_percentage # 50%
        # Count the number of elements less than the safe distance
        count = sum(1 for r in front if r < safe_distance)
        # Check if the count is greater than or equal to the threshold
        return count >= threshold

    def is_trapped(self, dist):
        # Segments from dictionary
        front = dist['front']
        front_left = dist['front_left']
        front_right = dist['front_right']

        # Check if the front is blocked
        if self.is_blocked(front, self.distance_is_trapped):
            # Check if the front left and front right are blocked
            front_left_blocked = self.is_blocked(front_left, self.distance_is_trapped)
            front_right_blocked = self.is_blocked(front_right, self.distance_is_trapped)

            # If all is blocked, it is trapped
            if front_left_blocked and front_right_blocked:
                return True
        return False # else not trapped

    def compare_score_pairs(self, dist):
        pairs = [('sl0', 'sr0'), ('sl1', 'sr1'), ('sl2', 'sr2'), ('sl3', 'sr3'), ('sl4', 'sr4')] # Store pairs in array

        for left, right in pairs:
            l_score = self.side_score_value(dist[left], left) # left score is computed
            r_score = self.side_score_value(dist[right], right) # right score is computed

            # If both are good
            if l_score > self.compare_score_pairs_threshold and r_score > self.compare_score_pairs_threshold:
                if abs(l_score - r_score) < self.compare_score_pairs_hysteresis:
                    return self.prev_direction  # Keep direction if below hystersis
                return 'LEFT' if l_score > r_score else 'RIGHT' # else return highest

            elif l_score > self.compare_score_pairs_threshold:
                return 'LEFT' # left is better; return left

            elif r_score > self.compare_score_pairs_threshold:
                return 'RIGHT' # right is better; return right

        return 'LEFT' # fallback; go left

    def turning_logic(self, dist):    
        if self.is_trapped(dist): # If trapped, must u-turn
            self.set_speed(dist['front'], 'TRAPPED')
            return

        direction = self.compare_score_pairs(dist) # Determine best side
        self.prev_direction = direction  # Save best side
        self.set_speed(dist['front'], direction) # turn towards best side

    def side_score_value(self, dist, name):
        valid = [r for r in dist if not isnan(r) and not isinf(r)] # Check wether valid
        if not valid and name in self.history and self.history[name]:
            valid = self.history[name] # Use history if invalid

        if valid: # Compute score for side
            count = sum(1 for r in valid if r >= 0.4)
            norm_count = count / len(valid)
            avg = sum(valid) / len(valid)
            score = (self.side_score_value_norm_factor * norm_count) + (self.side_score_value_avg_factor * avg)

            if dist and name in self.history: # Append to history if valid data to add
                self.history[name].append(score)

            return score # Return computed score

        return 0.0 # Fallback; return 0.0 if nothing is computed

    def set_speed(self, front, direction):
        # Calculate the linear speed
        lin_speed = self.dynamic_lin_speed(front)
        # Calculate the angular speed
        ang_speed = self.dynamic_ang_speed(front)
        twist = Twist()
        # Set the linear speed
        twist.linear.x = lin_speed
        
        # Set speed depending on input parameter
        if direction == 'FORWARD':
            lin_speed = lin_speed
        elif direction == 'LEFT':
            twist.angular.z = ang_speed
        elif direction == 'RIGHT':
            twist.angular.z = -ang_speed
        elif direction == 'TRAPPED': # Forced u-turn; publish and sleep
            twist.linear.x = 0.0
            twist.angular.z = 2.4
            self.metrics(abs(twist.linear.x), abs(twist.angular.z))
            self.cmd_vel_pub.publish(twist)
            print("TRAPPED")  
            time.sleep(0.3)
            return

        if twist.angular.z < self.speed_factor_threshold: # Check wether to multiply
            twist.linear.x = min(self.speed_linear_max, twist.linear.x * self.speed_factor) # Mutliple for extra action B-)
            twist.angular.z = twist.angular.z * self.speed_factor # Mutliple for extra action B-)

        self.latest_lin_speed = abs(twist.linear.x)

        self.metrics(abs(twist.linear.x), abs(twist.angular.z)) # Append speeds to array for metrics

        self.cmd_vel_pub.publish(twist) # Publish speed

    # -----------------------------------------------------------------------
    # Metrics
    # -----------------------------------------------------------------------
    def metrics(self, linear: float, angular: float):
        self.metric_linear.append(linear) # Append linear speed to array
        self.metric_angular.append(angular) # Append angular speed to array
    
    def check_collision(self, dist):
        if self.recent_collision(): # If recent collision, dont check for collisions
            return
        
        col_dist  = dist['collision']
        
        # counter incrementer
        if min(col_dist) < 0.16:
            self.collisioncounter += 1 # går op hvis noget er tæt på
        else:
            self.collisioncounter = max(0, self.collisioncounter - 1) # går ned hvis ikke der er

        # detect collision
        if self.collisioncounter >= 50:
            self.metric_collisions += 1 # count collision
            print(f"\033[91mRED DETECTED: Collision\033[0m") # print collision
            self.collisioncounter = 0 # reset counter
            self.collision_recent = True # Set recent collision to true
    
    def recent_collision(self):
        if self.collision_recent == True: # If recentl collision, count down until 0
            if self.collision_timer > 0:
                self.collision_timer = max(0, self.collision_timer - 1)
                return True
            else: # When 0 set recent collision to False and reset timer
                self.collision_timer = 100
                self.collision_recent = False
                return False
    
    def _read_rgb_raw(self): # comptues 16 bit rgb, returns 0-255 rgb
        # Read high byte and low byte for all channels
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)

        # left-shift highbyte, merge with lowbyte = 16-bit representation
        green16bit = (data[1] << 8) | data[0]
        red16bit = (data[3] << 8) | data[2]
        blue16bit = (data[5] << 8) | data[4]

        # 16-bit max is 65535; compute quotient and multiply by 255 for 0-255 scale
        red = (red16bit / 65535) * 255 
        green = (green16bit / 65535) * 255 
        blue = (blue16bit / 65535) * 255 

        # return channels
        return red, green, blue

    def getAndUpdateColour(self):
        red, green, blue = self._read_rgb_raw() # compute rgb
        total_sum = red + green + blue # total sum from all channels

        if total_sum == 0: # prevents division by 0
            norm_red = norm_green = norm_blue = 0
        else: # normalize
            norm_red = (red / total_sum)
            norm_green = (green / total_sum)
            norm_blue = (blue / total_sum) 

        # print(
        #     f"Red: {norm_red:.2f}, Green: {norm_green:.2f}, Blue: {norm_blue:.2f}"
        # )

        # RED detection
        if 0.44 <= norm_red:
            dominating_color = "red"
        else:
            dominating_color = "floor" # Gulv detected

        now = self.get_clock().now().nanoseconds / 1e9  # Sekunder

        # Hvis vi har en farve:
        if dominating_color:
            # Hvis farven har ændret sig ELLER der er gået mere end 2 sekunder siden sidste log
            if (dominating_color == "red") and (now - self.last_color_time > 2.0):
                print(f"\033[91mRED DETECTED: Victim has been located\033[0m")
                self.metric_victims_found += 1
                # LED blink logic
                self.victim_led.on()
                self.led_on = True

                self.last_color = dominating_color
                self.last_color_time = now

            if dominating_color != "red" and self.led_on == True: # Turn off LED after led_cycles
                    self.led_counter += 1
                    if self.led_counter == self.led_cycles:
                        self.victim_led.off()
                        self.led_counter = 0

    def _print_final_metrics(self):
        if len(self.metric_linear) == 0:
            print("No movement data collected.")
            return

        avg_lin  = sum(self.metric_linear)  / len(self.metric_linear)
        avg_ang  = sum(self.metric_angular) / len(self.metric_angular)

        print("\n\n=== FINAL METRICS ===")
        print(f"Total collisions: {self.metric_collisions}")
        print(f"Victims found: {self.metric_victims_found}")
        print(f"Average linear speed:  {avg_lin:.3f} m/s")
        print(f"Average angular speed: {avg_ang:.3f} rad/s")
        print("======================\n\n")

    def detect_obstacle(self): # Compute navigation
        dist = self.split_segments(self.scan_ranges)
        self.turning_logic(dist) 
        self.getAndUpdateColour()
        self.check_collision(dist)

    def log_metrics_each_second(self):
        now = time.time()
        if now - self.last_log_time >= 1.0:
            elapsed_time = int(now - self.start_time)
            if elapsed_time <= 121:
                self.metric_log.append({
                    "second": elapsed_time,
                    "collisions": self.metric_collisions,
                    "victims": self.metric_victims_found,
                    "linear_speed": self.latest_lin_speed
                })
                self.last_log_time = now
            elif elapsed_time > 121:
                self.export_metrics_to_excel()
                self.timer.cancel()  

    
    def export_metrics_to_excel(self):
        df = pd.DataFrame(self.metric_log)
        df.to_excel("turtlebot3_metrics.xlsx", index=False)
        print("Metrics exported to turtlebot3_metrics.xlsx")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("Robot stopped after 2 minutes.")


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
