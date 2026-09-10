### /Jonathan Burger
### 09/10/2026
### Program to control the mazigator robot through a 6x6 grid with two obstacles. 
### used A* as path finding algorithm and sensors utilized by Webots infrared proximity sensors.


from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())


left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_encoder = left_motor.getPositionSensor()
right_encoder = right_motor.getPositionSensor()

left_encoder.enable(timestep)
right_encoder.enable(timestep)

### Webots infrared proximity sensors declaration and implmenetation
sensors = []

for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    sensors.append(sensor)

### A* PATHFINDING Algorithm implementation

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):

    open_set = [start]
    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:

        current = min(
            open_set,
            key=lambda node: f_score.get(node, float("inf"))
        )

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()

            return path

        open_set.remove(current)

        neighbors = [
            (current[0] + 1, current[1]),
            (current[0] - 1, current[1]),
            (current[0], current[1] + 1),
            (current[0], current[1] - 1)
        ]

        for neighbor in neighbors:

            if (
                neighbor[0] < 0
                or neighbor[0] >= len(grid)
                or neighbor[1] < 0
                or neighbor[1] >= len(grid[0])
            ):
                continue

            if grid[neighbor[0]][neighbor[1]] == 1:
                continue

            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f_score[neighbor] = (
                    tentative_g
                    + heuristic(neighbor, goal)
                )

                if neighbor not in open_set:
                    open_set.append(neighbor)

    return None


### GRID ASSIGNMENT

grid = [
    [0, 0, 0, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0]
]

start = (0, 0)
goal = (5, 6)

path = astar(grid, start, goal)

print("A* PATH:", path)


### Wheel motor coordinates 

CELL_SIZE = 0.1

WHEEL_RADIUS = 0.0205

AXLE_LENGTH = 0.052

SPEED = 2.0


def stop_robot():

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def move_forward(distance):

    start_left = left_encoder.getValue()
    start_right = right_encoder.getValue()

    left_motor.setVelocity(SPEED)
    right_motor.setVelocity(SPEED)

    while robot.step(timestep) != -1:

        sensor_values = [sensor.getValue() for sensor in sensors]
        max_sensor = max(sensor_values)
               
        ### Indicate that there is a obstacle ahead       
        if max_sensor > 150:

            print(
                "IR OBSTACLE DETECTED!",
                "Max SENSOR =", round(max_sensor, 1)
            )

            stop_robot()

            return False

        left_distance = abs(
            left_encoder.getValue() - start_left
        ) * WHEEL_RADIUS

        right_distance = abs(
            right_encoder.getValue() - start_right
        ) * WHEEL_RADIUS

        average_distance = (
            left_distance + right_distance
        ) / 2.0

        if average_distance >= distance:
            break

    stop_robot()

    return True

### needed the ability to turn right and left.

def turn_left_90():

    left_motor.setVelocity(-SPEED)
    right_motor.setVelocity(SPEED)

    # Turn for approximately 90 degrees
    for _ in range(31):
        if robot.step(timestep) == -1:
            return

    stop_robot()


def turn_right_90():

    left_motor.setVelocity(SPEED)
    right_motor.setVelocity(-SPEED)

    for _ in range(31):
        if robot.step(timestep) == -1:
            return

    stop_robot()
### once path is determine, proceed with that path
if path:

    print("A* found a valid route with", len(path), "cells.")
    print("Starting A* navigation...")
    
    navigation_interrupted = False
    

    current_direction = (1, 0)

    for i in range(1, len(path)):

        current = path[i - 1]
        next_cell = path[i]

        dx = next_cell[0] - current[0]
        dz = next_cell[1] - current[1]

        new_direction = (dx, dz)

        print(
            "Moving from",
            current,
            "to",
            next_cell
        )

### if direction change is needed
        if new_direction != current_direction:

            ### turn left based on grid coordinates
            if current_direction == (1, 0) and new_direction == (0, 1):

                turn_left_90()

            ### turn right based on grid coordinates
            elif current_direction == (1, 0) and new_direction == (0, -1):

                turn_right_90()

            ### turn right based on grid coordinates
            elif current_direction == (0, 1) and new_direction == (1, 0):

                turn_right_90()

            ### turn right twice to go backwards based on grid coordinates

            elif current_direction == (0, 1) and new_direction == (-1, 0):

                turn_right_90()
                turn_right_90()

            ### turn right based on grid coordinates
            elif current_direction == (-1, 0) and new_direction == (0, 1):

                turn_right_90()

            ### turn left based on grid coordinates
            elif current_direction == (-1, 0) and new_direction == (0, -1):

                turn_left_90()

            ### turn left based on grid coordinates
            elif current_direction == (0, -1) and new_direction == (1, 0):

                turn_left_90()

            ### turn right based on grid coordinates
            elif current_direction == (0, -1) and new_direction == (-1, 0):

                turn_right_90()

            current_direction = new_direction
           
           
        ### print correct verbiage if unable to proceed.   
        if not move_forward(CELL_SIZE):
            print("Navigation stopped by IR sensor.")
            navigation_interrupted = True
            break

    stop_robot()

    if navigation_interrupted:
        print("A* navigation interrupted by obstacle.")
    else:
        print("A* navigation complete!")

else:

    stop_robot()

    print("Navigation stopped because no path exists.")


# ---------------------------------------------------------
# END SIMULATION LOOP
# ---------------------------------------------------------

while robot.step(timestep) != -1:

    pass