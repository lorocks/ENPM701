from queue import PriorityQueue
import numpy as np
import cv2
import time
import os
import math

import random


# Function to calculate Euclidean distance
def heuristic(node, goal):
    return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5

# Function to create the grid space
def createGrid(height, width, bounding_location, padding = 0, wall_padding = 0, scale = 1):
  image = np.full((height, width, 3), 255, dtype=np.uint8)

  # Add padding to walls
  if wall_padding > 0:
    image = setWallPadding(image, wall_padding, 125)

  # Add padding and obstacles using half planes and semi algebraic equations
  for obstacle in bounding_location:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    obstacle = obstacle.astype(np.int32)
    if len(obstacle) == 1:
        center_x = obstacle[0][0]
        center_y = obstacle[0][1]
        radius = obstacle[0][2]

        x_small = center_x - radius
        x_big = center_x + radius
        y_small = center_y - radius
        y_big = center_y + radius

        x_small_pad = x_small
        x_big_pad = x_big
        y_small_pad = y_small
        y_big_pad = y_big

        if x_small - padding > 0:
            x_small_pad -= padding
        else:
            x_small_pad = 0
        if x_big + padding < width:
            x_big_pad += padding
        else:
            x_big_pad = width
        if y_small - padding > 0:
            y_small_pad -= padding
        else:
            y_small_pad = 0
        if y_big + padding < height:
            y_big_pad += padding
        else:
            y_big_pad = height

        for x in range(x_small_pad, x_big_pad):
            for y in range(y_small_pad, y_big_pad):
                if heuristic((x, y), (center_x, center_y)) <= radius + padding + 1:
                    image[y, x] = (125, 125, 125)

        for x in range(x_small, x_big):
            for y in range(y_small, y_big):
                if heuristic((x, y), (center_x, center_y)) <= radius + 1:
                    image[y, x] = (0, 0, 0)

    else:
        x_small = x_big = y_small = y_big = -1
        for point in obstacle:
            if x_small > point[0] or x_small == -1:
                x_small = point[0]
            if x_big < point[0] or x_big == -1:
                x_big = point[0]
            if y_small > point[1] or y_small == -1:
                y_small = point[1]
            if y_big < point[1] or y_big == -1:
                y_big = point[1]

            x_small_pad = x_small
            x_big_pad = x_big
            y_small_pad = y_small
            y_big_pad = y_big

            if x_small - padding > 0:
                x_small_pad -= padding
            else:
                x_small_pad = 0
            if x_big + padding < width:
                x_big_pad += padding
            else:
                x_big_pad = width
            if y_small - padding > 0:
                y_small_pad -= padding
            else:
                y_small_pad = 0
            if y_big + padding < height:
                y_big_pad += padding
            else:
                y_big_pad = height

            image[y_small_pad:y_big_pad, x_small_pad:x_big_pad] = (125, 125, 125)
            image[y_small:y_big, x_small:x_big] = (0, 0, 0)


  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  grid = np.full((height, width), 0)
  grid[gray == 125] = -12
  grid[gray == 180] = -15
  grid[gray == 0] = -11
  return grid, gray


# Function to add padding for walls
def setWallPadding(image, padding, value):
  height, width, _ = image.shape
  points = [
      (0, 0), (padding, height),
      (0, 0), (width, padding),
      (0, height - padding), (width, height),
      (width - padding, 0), (width, height),
  ]
  for i in range(0, len(points), 2):
    image[points[i][1]:points[i+1][1], points[i][0]:points[i+1][0]] = (value, value, value)
  return image


# Generate x, y from a 2D Guassian distribution
def generateGuassianRandom(x, y, angle, rho, sigma):
  x1 = np.random.normal()
  x2 = np.random.normal()

  x3 = rho*x1 + (1 - (rho**2))**0.5 * x2

  point1 = sigma * x1
  point2 = sigma * x3

  rand_ = np.array([point1, point2]).reshape((-1, 1))
  rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
  rand = np.dot(rot, rand_) + np.array([x, y]).reshape(-1, 1)

  return rand[0], rand[1]



# Main
def runPlanner(step_size):

    scale = 30

    height = 64 * scale
    width = 93 * scale

    obstacle_bounding_boxes = [
        [(24, height), (24, 29), (25, 29), (25, height)],
        [(88, height), (88, 29), (89, 29), (89, height)],
        [(56, 35), (56, 0), (57, 0), (57, 35)],
        ]

    step_size = int(step_size * scale) # 15

    starting_x = 0 * scale
    starting_y = 32 * scale

    goal_x = 82 * scale
    goal_y = 15 * scale
    
    print("Building Obstacle Space")
    start = time.time()

    grid_f, useless = createGrid(height, width, obstacle_bounding_boxes, 10 * scale, 0, scale)
    grid_b = grid_f.copy()
    backtrack_grid_f = np.full(grid_f.shape, -1)
    backtrack_grid_b = np.full(grid_b.shape, -1)

    print("Obstacle map created")
    end = time.time()
    print(end - start)

    valid = False
    while not valid:
        try:
            if grid_f[starting_y, starting_x] == 0:
                grid_f[starting_y, starting_x] = 1
                valid = True
            else:
                print("\nStarting position invalid, obstacle exists\n")
                return False
        except:
            print("\nStarting position invalid\n")
            return False


    valid = False
    while not valid:
        try:
            if grid_b[goal_y, goal_x] == 0:
                grid_b[goal_y, goal_x] = 1
                valid = True
            else:
                print("\nGoal position invalid, obstacle exists\n")
                return False
        except:
            print("\nGoal position invalid, obstacle exists\n")
            return False


    # Forward variables
    far_x_f = starting_x
    near_x_f = starting_x

    far_y_f = starting_y
    near_y_f = starting_y


    # Backward variables
    far_x_b = goal_x
    near_x_b = goal_x

    far_y_b = goal_y
    near_y_b = goal_y

    # Random point generation
    rho = 0.5
    sigma = 0.25 * heuristic((starting_x, starting_y), (goal_x, goal_y))
    a1 = math.atan2(goal_y - starting_y, goal_x - starting_x)
    a2 = math.radians(45)
    a3 = a1 - a2

    bias1 = 0.6 # 0.6
    bias2 = 0.9

    print("Starting algorithm")
    start = time.time()
    while True:
    # Get random points
        ##### Forward RRT Start
        p = random.uniform(0, 1)

        if p <= bias1:
            accept = False
            while not accept:
                x_temp, y_temp = generateGuassianRandom(goal_x, goal_y, a3, rho, sigma)

                if x_temp > 0 and y_temp > 0 and y_temp < height and x_temp < width:
                    accept = True
                    x_rand = x_temp[0]
                    y_rand = y_temp[0]


        elif p >= bias2:
            x_rand = goal_x
            y_rand = goal_y
        else:
            x_rand = random.uniform(0, width-1)
            y_rand = random.uniform(0, height-1)

        x_round = round(x_rand)
        y_round = round(y_rand)

        
        added = False
        # print("Not within obstacle")
        range_queue = PriorityQueue()

        if x_round > far_x_f:
            max_x = far_x_f + 1
            min_x = max(0, far_x_f - (2 * step_size) - 1)
        elif x_round < near_x_f:
            min_x = near_x_f - 1
            max_x = min(width, near_x_f + (2 * step_size) + 1)
        else:
            # min_x = x_round - (3 * step_size)
            # max_x = x_round + (3 * step_size)
            min_x = near_x_f
            max_x = far_x_f + 1

        # print("Creating queue")

        for node in np.argwhere(grid_f[near_y_f:far_y_f + 1, min_x:max_x] > 0):
            node[0] += near_y_f
            node[1] += min_x
        # for node in np.argwhere(grid > 0):
            tru_dist = heuristic((node[1], node[0]), (x_rand, y_rand))
            range_queue.put((tru_dist, node[0], node[1]))

        # print("Query queue")
        true_dist, node_y, node_x = range_queue.get()

        # Circle eq
        if true_dist <=15:
            # print("15 lesser")
            x1 = round((node_x + (2 * x_rand)) / 3)
            y1 = round((node_y + (2 * y_rand)) / 3)

            x2 = round((x_rand + (2 * node_x)) / 3)
            y2 = round((y_rand + (2 * node_y)) / 3)

            if grid_f[y1, x1] >= 0:
                if grid_f[y2, x2] >= 0:
                    if grid_f[y_round, x_round] == 0 or grid_f[y_round, x_round] > grid_f[node_y, node_x] + true_dist:
                        grid_f[y_round, x_round] = grid_f[node_y, node_x] + true_dist
                        backtrack_grid_f[y_round, x_round] = node_x + (node_y * width)
                        added = True
                        x_actual = x_round
                        y_actual = y_round
                        # print("added")
                        if x_round > far_x_f:
                            far_x_f = x_round
                        elif x_round < near_x_f:
                            near_x_f = x_round

        else:
            # print("sadge1")
            m = (y_rand - node_y)/(x_rand - node_x)

            x_actual_1 = (step_size/((1 + (m**2))**0.5)) + node_x
            y_actual_1 = (m * x_actual_1) - (m * node_x) + node_y

            x_actual_2 = -(step_size/((1 + (m**2))**0.5)) + node_x
            y_actual_2 = (m * x_actual_2) - (m * node_x) + node_y

            if heuristic((x_actual_1, y_actual_1), (x_rand, y_rand)) > heuristic((x_actual_2, y_actual_2), (x_rand, y_rand)):
                x_actual = round(x_actual_2)
                y_actual = round(y_actual_2)
            else:
                x_actual = round(x_actual_1)
                y_actual = round(y_actual_1)

            if x_actual < 0 or x_actual > width or y_actual < 0 or y_actual > height:
                continue

            x1 = round((node_x + (2 * x_actual)) / 3)
            y1 = round((node_y + (2 * y_actual)) / 3)

            x2 = round((x_actual + (2 * node_x)) / 3)
            y2 = round((y_actual + (2 * node_y)) / 3)

            if grid_f[y1, x1] >= 0:
                if grid_f[y2, x2] >= 0:
                    if grid_f[y_actual, x_actual] == 0 or grid_f[y_actual, x_actual] > grid_f[node_y, node_x] + true_dist:
                        grid_f[y_actual, x_actual] = grid_f[node_y, node_x] + true_dist
                        backtrack_grid_f[y_actual, x_actual] = node_x + (node_y * width)
                        added = True
                        # print("added")

                        if x_actual > far_x_f:
                            far_x_f = x_actual
                        elif x_actual < near_x_f:
                            near_x_f = x_actual

                        if y_actual > far_y_f:
                            far_y_f = y_actual
                        elif y_actual < near_y_f:
                            near_y_f = y_actual

        # Check Continuity
        y_min_check = max(0, y_actual - step_size)
        y_max_check = min(height, y_actual + step_size)
        x_min_check = max(0, x_actual - step_size)
        x_max_check = min(width, x_actual + step_size)
        near_points = np.argwhere(grid_b[y_min_check:y_max_check, x_min_check:x_max_check] > 0)

        if added:
            if len(near_points) != 0:
                print("Linking found forward")
                index_f = (x_actual, y_actual)
                index_b = (near_points[0][1] + x_min_check, near_points[0][0] + y_min_check)
                break

            # Recheck branches
            nodes = np.argwhere(grid_f[y_min_check :y_max_check , x_min_check :x_max_check ] > 0)
            for i, node_i in enumerate(nodes):
                for node_j in nodes[i:]:
                    dist_h = heuristic((node_i[1], node_i[0]), (node_j[1], node_j[0]))
                    if dist_h <= 15 and grid_f[node_j[0] + y_min_check, node_j[1] + x_min_check] < grid_f[node_i[0] + y_min_check, node_i[1] + x_min_check] + dist_h:
                        grid_f[node_j[0] + y_min_check,node_j[1] + x_min_check] = grid_f[node_i[0] + y_min_check, node_i[1] + x_min_check] + dist_h
                        backtrack_grid_f[node_j[0] + y_min_check, node_j[1] + x_min_check] = node_i[1] + x_min_check  + ((node_i[0] + y_min_check)  * width)
                    else:
                        break

        ##### End Forward RRT


        ##### Backward RRT Start
        p = random.uniform(0, 1)

        if p <= bias1:
            accept = False
            while not accept:
                x_temp, y_temp = generateGuassianRandom(starting_x, starting_y, a3, rho, sigma)

                if x_temp > 0 and y_temp > 0 and y_temp < height and x_temp < width:
                    accept = True
                    x_rand = x_temp[0]
                    y_rand = y_temp[0]

        elif p >= bias2:
            x_rand = starting_x
            y_rand = starting_y
        else:
            x_rand = random.uniform(0, width - 1)
            y_rand = random.uniform(0, height - 1)

        x_round = round(x_rand)
        y_round = round(y_rand)

        added = False
        # print("Not within obstacle")
        range_queue = PriorityQueue()

        if x_round > far_x_b:
            max_x = far_x_b + 1
            min_x = max(0, far_x_b - (2 * step_size) - 1)
        elif x_round < near_x_b:
            min_x = near_x_b - 1
            max_x = min(width, near_x_b + (2 * step_size) + 1)
        else:
            # min_x = x_round - (3 * step_size)
            # max_x = x_round + (3 * step_size)
            min_x = near_x_b
            max_x = far_x_b + 1


        # print("Creating queue")

        for node in np.argwhere(grid_b[near_y_b:far_y_b+1, min_x:max_x] > 0):
            node[0] += near_y_b
            node[1] += min_x
            tru_dist = heuristic((node[1], node[0]), (x_rand, y_rand))
            range_queue.put((tru_dist, node[0], node[1]))

        # print("Query queue")
        # est_cost, true_dist, node_y, node_x = range_queue.get()
        true_dist, node_y, node_x = range_queue.get()

        # Circle eq
        if true_dist <=15:
            # print("15 lesser")
            x1 = round((node_x + (2 * x_rand)) / 3)
            y1 = round((node_y + (2 * y_rand)) / 3)

            x2 = round((x_rand + (2 * node_x)) / 3)
            y2 = round((y_rand + (2 * node_y)) / 3)


            if grid_b[y1, x1] >= 0:
                if grid_b[y2, x2] >= 0:
                    if grid_b[y_round, x_round] == 0 or grid_b[y_round, x_round] > grid_b[node_y, node_x] + true_dist:
                        grid_b[y_round, x_round] = grid_b[node_y, node_x] + true_dist
                        backtrack_grid_b[y_round, x_round] = node_x + (node_y * width)
                        added = True
                        x_actual = x_round
                        y_actual = y_round
                        # print("added")

                        if x_round > far_x_b:
                            far_x_b = x_round
                        elif x_round < near_x_b:
                            near_x_b = x_round

                        if y_round > far_y_b:
                            far_y_b = y_round
                        elif y_round < near_y_b:
                            near_y_b = y_round

        else:
            # print("sadge1")
            m = (y_rand - node_y)/(x_rand - node_x)

            x_actual_1 = (step_size/((1 + (m**2))**0.5)) + node_x
            y_actual_1 = (m * x_actual_1) - (m * node_x) + node_y

            x_actual_2 = -(step_size/((1 + (m**2))**0.5)) + node_x
            y_actual_2 = (m * x_actual_2) - (m * node_x) + node_y

            if heuristic((x_actual_1, y_actual_1), (x_rand, y_rand)) > heuristic((x_actual_2, y_actual_2), (x_rand, y_rand)):
                x_actual = round(x_actual_2)
                y_actual = round(y_actual_2)
            else:
                x_actual = round(x_actual_1)
                y_actual = round(y_actual_1)

            if x_actual < 0 or x_actual > width or y_actual < 0 or y_actual > height:
                continue

            x1 = round((node_x + (2 * x_actual)) / 3)
            y1 = round((node_y + (2 * y_actual)) / 3)

            x2 = round((x_actual + (2 * node_x)) / 3)
            y2 = round((y_actual + (2 * node_y)) / 3)

            if grid_b[y1, x1] >= 0:
                if grid_b[y2, x2] >= 0:
                    if grid_b[y_actual, x_actual] == 0 or grid_b[y_actual, x_actual] > grid_b[node_y, node_x] + true_dist:
                        grid_b[y_actual, x_actual] = grid_b[node_y, node_x] + true_dist
                        backtrack_grid_b[y_actual, x_actual] = node_x + (node_y * width)
                        added = True
                        # print("added")
                        if x_actual > far_x_b:
                            far_x_b = x_actual
                        elif x_actual < near_x_b:
                            near_x_b = x_actual

                        if y_actual > far_y_b:
                            far_y_b = y_actual
                        elif y_actual < near_y_b:
                            near_y_b = y_actual

        # Check Continuity
        y_min_check = max(0, y_actual - step_size)
        y_max_check = min(height, y_actual + step_size)
        x_min_check = max(0, x_actual - step_size)
        x_max_check = min(width, x_actual + step_size)
        near_points = np.argwhere(grid_f[y_min_check:y_max_check, x_min_check:x_max_check] > 0)

        if added:
            if len(near_points) != 0:
                print("Linking found backtrack")
                index_b = (x_actual, y_actual)
                index_f = (near_points[0][1] + x_min_check, near_points[0][0] + y_min_check)
                break


            # Recheck branches
            nodes = np.argwhere(grid_b[y_min_check :y_max_check , x_min_check :x_max_check ] > 0)
            for i, node_i in enumerate(nodes):
                for node_j in nodes[i:]:
                    dist_h = heuristic((node_i[1], node_i[0]), (node_j[1], node_j[0]))
                    if dist_h <= 15 and grid_b[node_j[0] + y_min_check, node_j[1] + x_min_check] < grid_b[node_i[0] + y_min_check,node_i[1] + x_min_check] + dist_h:
                        grid_b[node_j[0] + y_min_check, node_j[1] + x_min_check] = grid_b[node_i[0] + y_min_check, node_i[1] + x_min_check] + dist_h
                        backtrack_grid_b[node_j[0] + y_min_check, node_j[1] + x_min_check] = node_i[1] + x_min_check  + ((node_i[0] + y_min_check)  * width)
                    else:
                        break


    ##### End Backward RRT

    print(f"Algorithm Execution time: {time.time() - start}")


    # Display
    grid, gray = createGrid(height, width, obstacle_bounding_boxes, 10 * scale, 0, scale)

    image = np.full((height, width, 3), (224, 224, 224))
    image[gray == 125] = (125, 125, 125)
    image[gray == 0] = (0, 0, 0)
    image = np.ascontiguousarray(image, dtype=np.uint8)

    for node in np.argwhere(backtrack_grid_f >= 0):
        x_pos = int(backtrack_grid_f[node[0], node[1]] % width)
        y_pos = int((backtrack_grid_f[node[0], node[1]] - (backtrack_grid_f[node[0], node[1]] % width))/width)
        # print(heuristic(tuple(node)))
        cv2.line(image, (node[1], node[0]), (x_pos, y_pos), (125, 255, 125), 2)

    for node in np.argwhere(backtrack_grid_b >= 0):
        x_pos = int(backtrack_grid_b[node[0], node[1]] % width)
        y_pos = int((backtrack_grid_b[node[0], node[1]] - (backtrack_grid_b[node[0], node[1]] % width))/width)
        # print(heuristic(tuple(node)))
        cv2.line(image, (node[1], node[0]), (x_pos, y_pos), (125, 255, 125), 2)

    # cv2.circle(image, (goal_x, goal_y), step_size, (255, 0, 0), 1)

    for node in np.argwhere(backtrack_grid_f >= 0):
        cv2.circle(image, (node[1], node[0]), 2, (0, 0, 255), -1)

    for node in np.argwhere(backtrack_grid_b >= 0):
        cv2.circle(image, (node[1], node[0]), 2, (0, 0, 255), -1)


    # Backtracking
    path = []
    costs_f = []

    index = backtrack_grid_f[index_f[1], index_f[0]]
    index_p = index_f
    while index > 0:
        path.append(index_p)
        costs_f.append(grid_f[index_p[1], index_p[0]])
        index = backtrack_grid_f[index_p[1], index_p[0]]
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)
        index_p = (x_pos, y_pos)
    path.append((starting_x, starting_y))
    costs_f.append(0)
    path.reverse()
    costs_f.reverse()


    max_cost_f = max(costs_f)
    costs_b = []
    index = backtrack_grid_b[index_b[1], index_b[0]]
    index_p = index_b
    while index > 0:
        path.append(index_p)
        costs_b.append(grid_b[index_p[1], index_p[0]] + max_cost_f)
        index = backtrack_grid_b[index_p[1], index_p[0]]
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)
        index_p = (x_pos, y_pos)
    costs_b.reverse()

    costs = np.concatenate((np.array(costs_f), np.array(costs_b)))

    last = -1
    for point in path:
        if last == -1:
            last = point
        else:
            cv2.line(image, last, point, (240, 32, 160), 2)
            last = point

    print("Image of final path will be displayed after 2 seconds, enter any key to exit image and continue application")
    time.sleep(2)
    
    cv2.imshow("Path", cv2.resize(cv2.flip(image, 0), (1200, 400)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Optimize path
    print("Optimizing Path")
    actual_path = []

    i = 0

    actual_path.append(i)

    while i < len(path[:-1]):
        fail = False
        for j in range(len(path[i:])):

            image_check = gray.copy()
            cv2.line(image_check, path[i], path[j+i], 255, 1)

            if np.count_nonzero(np.subtract(gray, image_check)) != 0:
                actual_path.append(j+i-1)
                fail = True
                i = j + i - 1
                break

            if fail:
                break

        i += 1

    actual_path.append(i)

    path_o = []

    path_o.append(path[actual_path[0]])
    for i in actual_path[1:-1]:
        # path_o.append(path[i-1])
        path_o.append(path[i])
        # path_o.append(path[i+1])

    path_o.append(path[actual_path[-1]])



    # Plot Optimal Path
    last = -1
    for point in path_o:
        if last == -1:
            last = point
        else:
            cv2.line(image, last, point, (255, 0, 0), 2)
            last = point


    # Display Optimized path
    print("Optimized path will be overlapped on initial image, display in 2 seconds, enter any key to exit image and continue application")
    time.sleep(2)

    cv2.imshow("Path", cv2.resize(cv2.flip(image, 0), (1200, 400)))
    cv2.waitKey(0)

    cv2.destroyAllWindows()
    
    return  np.array(path_o)/scale