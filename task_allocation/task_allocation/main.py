from nav2_commander import *
from genetic_algorithm import *



def main():
    rclpy.init()
    nav = BasicNavigator()

    picking_stations = [(0.0,-2.0), (-1.0,-2.0), (1.0,-2.0)]
    task_shelves_coordinates =  [(2.0, 0.6), (2.0, 0.0), (2.0, -0.6),
                                (0.2, -0.4), (0.2, -0.6), (-2.0, 0.6)]
    tasks_num = len(task_shelves_coordinates)
    task_shelves_poses = make_pose_stamps(task_shelves_coordinates, nav)
    
    robots_num = 2
    robots_dict = make_robots_dict("pose.csv")
    robots_coordinates = robots_dict.values()
    robots_poses = make_pose_stamps(robots_coordinates, nav)

    pop_size = 100
    MaxEpoc = 10000
    crossover_rate = 0.95
    elitist_precentage = 20
    distance_strategy = euclidean_distance
    nav = None

    robot_tasks = genetic_alg(
        pop_size=pop_size,
        MaxEpoc=MaxEpoc,
        crossover_rate=crossover_rate,
        num_robots=robots_num, 
        num_tasks=tasks_num, 
        elitist_precentage=elitist_precentage,
        robots_coordinates=robots_coordinates, 
        robots_poses=robots_poses, 
        task_shelves_coordinates=task_shelves_coordinates, 
        task_shelves_poses=task_shelves_poses, 
        distance_strag=distance_strategy,
        nav=nav)
    '''
    nav_list = []
    robots_waypoints = []

    print("creating nav2 ojects.....")
    for i, robot in enumerate(robot_tasks):
        robot_name = 'robot' + str(i + 1)
        nav_list.append(BasicNavigator(namespace=robot_name))
    print("starting NAV2.....")
    for nav in nav_list:
        nav.waitUntilNav2Active()
    print("creating waypoints.....")
    for robot in robot_tasks:
        waypoints = []
        for task in robot:
            waypoints.append(create_pose_stamped(nav, task_shelves_coordinates[task[0] - 1][0], task_shelves_coordinates[task[0] - 1][1], 0.0))
            waypoints.append(create_pose_stamped(nav, picking_stations[task[1]][0], picking_stations[task[1]][1], 0.0))
        robots_waypoints.append(waypoints)
    print("Starting Navigation.....")
    for i, nav in enumerate(nav_list):
        nav.followWaypoints(robots_waypoints[i])
    
    while not all(nav.isTaskComplete() for nav in nav_list):
        pass
    #         feedback = nav.getFeedback()
    #         # print(feedback)

    # --- Get the result ---
    for nav in nav_list:
        print(nav.getResult())
    
    '''

    rclpy.shutdown()


if __name__ == '__main__':
    main()
