import math

#################### RACING LINE ######################

# Optimal racing line for the Spain track
# Each row: [x,y,speed,timeFromPreviousPoint]
racing_track = [[3.06664, 0.69989, 4.0, 0.03654],
	[3.21372, 0.69357, 4.0, 0.0368],
	[3.36169, 0.6893, 4.0, 0.03701],
	[3.51032, 0.68657, 4.0, 0.03716],
	[3.6594, 0.68496, 4.0, 0.03727],
	[3.8088, 0.68412, 4.0, 0.03735],
	[3.9584, 0.68379, 4.0, 0.0374],
	[4.10793, 0.68414, 4.0, 0.03738],
	[4.25712, 0.68535, 4.0, 0.0373],
	[4.40585, 0.68761, 4.0, 0.03719],
	[4.55396, 0.69115, 4.0, 0.03704],
	[4.70133, 0.69619, 4.0, 0.03686],
	[4.84783, 0.70293, 4.0, 0.03666],
	[4.99331, 0.71158, 4.0, 0.03644],
	[5.13763, 0.72237, 4.0, 0.03618],
	[5.28066, 0.73548, 3.77051, 0.03809],
	[5.42227, 0.75106, 3.4085, 0.0418],
	[5.56233, 0.76926, 3.0652, 0.04608],
	[5.70059, 0.79043, 2.73898, 0.05107],
	[5.83677, 0.81492, 2.42648, 0.05702],
	[5.97044, 0.84325, 2.14982, 0.06356],
	[6.10109, 0.87602, 1.90057, 0.07087],
	[6.22807, 0.91394, 1.6532, 0.08016],
	[6.35051, 0.95783, 1.6532, 0.07868],
	[6.46729, 1.00867, 1.6532, 0.07704],
	[6.57689, 1.06758, 1.6532, 0.07526],
	[6.67731, 1.1357, 1.5, 0.0809],
	[6.76588, 1.21406, 1.5, 0.07884],
	[6.83839, 1.3035, 1.5, 0.07675],
	[6.8965, 1.40041, 1.5, 0.07534],
	[6.94112, 1.50274, 1.5, 0.07442],
	[6.96947, 1.60974, 1.5, 0.0738],
	[6.97707, 1.71948, 1.62547, 0.06767],
	[6.96702, 1.82873, 1.64355, 0.06675],
	[6.94149, 1.93565, 1.64355, 0.06688],
	[6.90175, 2.03894, 1.64355, 0.06734],
	[6.84699, 2.13674, 1.64355, 0.0682],
	[6.77574, 2.22619, 1.81638, 0.06296],
	[6.69117, 2.307, 2.02166, 0.05786],
	[6.5958, 2.37958, 2.25647, 0.05312],
	[6.49161, 2.44467, 2.57378, 0.04773],
	[6.38049, 2.50335, 2.88958, 0.04349],
	[6.26371, 2.5565, 3.28713, 0.03903],
	[6.14243, 2.60505, 3.84335, 0.03399],
	[6.01777, 2.65004, 4.0, 0.03313],
	[5.89082, 2.69257, 4.0, 0.03347],
	[5.76272, 2.73384, 4.0, 0.03365],
	[5.63017, 2.77782, 4.0, 0.03491],
	[5.49811, 2.82317, 4.0, 0.03491],
	[5.36667, 2.87018, 4.0, 0.0349],
	[5.23602, 2.9192, 4.0, 0.03489],
	[5.10632, 2.97055, 4.0, 0.03487],
	[4.97777, 3.02458, 4.0, 0.03486],
	[4.85051, 3.08159, 4.0, 0.03486],
	[4.72465, 3.14171, 4.0, 0.03487],
	[4.60022, 3.20493, 4.0, 0.03489],
	[4.47719, 3.27112, 4.0, 0.03493],
	[4.35549, 3.34005, 4.0, 0.03497],
	[4.23502, 3.41139, 4.0, 0.035],
	[4.11568, 3.48475, 4.0, 0.03502],
	[3.99733, 3.55968, 4.0, 0.03502],
	[3.87982, 3.63569, 3.99168, 0.03506],
	[3.76284, 3.71231, 3.40158, 0.04111],
	[3.64732, 3.78753, 2.96325, 0.04652],
	[3.53132, 3.86145, 2.58054, 0.05331],
	[3.41449, 3.93319, 2.58054, 0.05313],
	[3.29649, 4.00174, 2.58054, 0.05288],
	[3.17696, 4.06601, 2.58054, 0.05259],
	[3.05548, 4.12441, 2.58054, 0.05223],
	[2.93169, 4.17515, 2.58054, 0.05184],
	[2.80549, 4.21581, 2.84648, 0.04658],
	[2.67785, 4.24822, 2.95493, 0.04456],
	[2.5493, 4.27301, 2.85411, 0.04587],
	[2.42021, 4.29067, 2.71843, 0.04793],
	[2.29093, 4.30153, 2.46766, 0.05258],
	[2.16175, 4.30562, 2.25095, 0.05742],
	[2.03303, 4.30283, 2.01526, 0.06389],
	[1.90519, 4.29292, 1.77238, 0.07234],
	[1.7788, 4.27535, 1.64, 0.0778],
	[1.65459, 4.24957, 1.64, 0.07736],
	[1.53376, 4.21418, 1.64, 0.07677],
	[1.41797, 4.16786, 1.64, 0.07604],
	[1.30974, 4.10893, 1.64, 0.07514],
	[1.21287, 4.03538, 1.64, 0.07416],
	[1.13093, 3.94692, 1.69002, 0.07135],
	[1.06435, 3.84609, 1.87443, 0.06446],
	[1.01121, 3.73603, 2.06299, 0.05924],
	[0.96999, 3.61869, 2.25217, 0.05522],
	[0.93956, 3.49541, 2.45838, 0.05165],
	[0.91891, 3.36729, 2.68098, 0.04841],
	[0.90708, 3.23527, 2.89759, 0.04574],
	[0.90334, 3.10018, 3.15381, 0.04285],
	[0.90681, 2.9629, 3.34051, 0.04111],
	[0.91698, 2.82419, 3.49559, 0.03979],
	[0.93341, 2.68483, 3.62072, 0.03876],
	[0.95571, 2.54557, 3.61788, 0.03898],
	[0.98342, 2.40706, 3.43355, 0.04114],
	[1.01626, 2.26986, 3.20539, 0.04401],
	[1.05392, 2.13444, 2.92442, 0.04806],
	[1.09624, 2.00121, 2.65809, 0.05259],
	[1.14311, 1.87057, 2.39254, 0.05801],
	[1.19482, 1.7431, 2.12719, 0.06467],
	[1.25158, 1.61938, 1.8624, 0.07309],
	[1.31382, 1.50015, 1.8624, 0.07222],
	[1.38221, 1.38643, 1.8624, 0.07125],
	[1.45757, 1.27943, 1.8624, 0.07027],
	[1.54096, 1.18072, 1.8624, 0.06938],
	[1.63386, 1.09253, 1.8624, 0.06878],
	[1.7384, 1.01844, 2.19396, 0.0584],
	[1.85098, 0.955, 2.43202, 0.05313],
	[1.97002, 0.90067, 2.63525, 0.04966],
	[2.09459, 0.85453, 2.85162, 0.04658],
	[2.2239, 0.81579, 3.08727, 0.04372],
	[2.35729, 0.78373, 3.34554, 0.04101],
	[2.49419, 0.75767, 3.63682, 0.03832],
	[2.63406, 0.73695, 3.99096, 0.03543],
	[2.77639, 0.72086, 4.0, 0.03581],
	[2.92074, 0.70874, 4.0, 0.03621]]

################## HELPER FUNCTIONS ###################

def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

def closest_2_racing_points_index(racing_coords, car_coords):

    # Calculate all distances to racing points
    distances = []
    for i in range(len(racing_coords)):
        distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                 y1=racing_coords[i][1], y2=car_coords[1])
        distances.append(distance)

    # Get index of the closest racing point
    closest_index = distances.index(min(distances))

    # Get index of the second closest racing point
    distances_no_closest = distances.copy()
    distances_no_closest[closest_index] = 999
    second_closest_index = distances_no_closest.index(
        min(distances_no_closest))

    return [closest_index, second_closest_index]

def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
    
    # Calculate the distances between 2 closest racing points
    a = abs(dist_2_points(x1=closest_coords[0],
                          x2=second_closest_coords[0],
                          y1=closest_coords[1],
                          y2=second_closest_coords[1]))

    # Distances between car and closest and second closest racing point
    b = abs(dist_2_points(x1=car_coords[0],
                          x2=closest_coords[0],
                          y1=car_coords[1],
                          y2=closest_coords[1]))
    c = abs(dist_2_points(x1=car_coords[0],
                          x2=second_closest_coords[0],
                          y1=car_coords[1],
                          y2=second_closest_coords[1]))

    # Calculate distance between car and racing line (goes through 2 closest racing points)
    # try-except in case a=0 (rare bug in DeepRacer)
    try:
        distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                       (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
    except:
        distance = b

    return distance

# Calculate which one of the closest racing points is the next one and which one the previous one
def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

    # Virtually set the car more into the heading direction
    heading_vector = [math.cos(math.radians(
        heading)), math.sin(math.radians(heading))]
    new_car_coords = [car_coords[0]+heading_vector[0],
                      car_coords[1]+heading_vector[1]]

    # Calculate distance from new car coords to 2 closest racing points
    distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                x2=closest_coords[0],
                                                y1=new_car_coords[1],
                                                y2=closest_coords[1])
    distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                       x2=second_closest_coords[0],
                                                       y1=new_car_coords[1],
                                                       y2=second_closest_coords[1])

    if distance_closest_coords_new <= distance_second_closest_coords_new:
        next_point_coords = closest_coords
        prev_point_coords = second_closest_coords
    else:
        next_point_coords = second_closest_coords
        prev_point_coords = closest_coords

    return [next_point_coords, prev_point_coords]

def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

    # Calculate the direction of the center line based on the closest waypoints
    next_point, prev_point = next_prev_racing_point(closest_coords,
                                                    second_closest_coords,
                                                    car_coords,
                                                    heading)

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    return direction_diff

# Gives back indexes that lie between start and end index of a cyclical list 
# (start index is included, end index is not)
def indexes_cyclical(start, end, array_len):

    if end < start:
        end += array_len

    return [index % array_len for index in range(start, end)]

# Calculate how long car would take for entire lap, if it continued like it did until now
def project_time(first_index, closest_index, step_count, times_list):

    # Calculate how much time has passed since start
    current_actual_time = (step_count-1) / 15

    # Calculate which indexes were already passed
    indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

    # Calculate how much time should have passed if car would have followed optimals
    current_expected_time = sum([times_list[i] for i in indexes_traveled])

    # Calculate how long one entire lap takes if car follows optimals
    total_expected_time = sum(times_list)

    # Calculate how long car would take for entire lap, if it continued like it did until now
    try:
        projected_time = (current_actual_time/current_expected_time) * total_expected_time
    except:
        projected_time = 9999

    return projected_time
            
class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):
        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 30
        FASTEST_TIME = 18
        times_list = [row[3] for row in racing_track]
        projected_time = project_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 30  # seconds (time that is easily done by model)
        FASTEST_TIME = 18  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)