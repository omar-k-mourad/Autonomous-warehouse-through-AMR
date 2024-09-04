from nav2_commander import *
from genetic_algorithm import *
from set_cover_greedy import *
from clean_data import *
from warehouseDBFunctions import *
from param_config import *
import ast
import time

def get_shelves_to_pick():
    ordered_products = get_ordered_products()
    if ordered_products:
        warehouse, shelves_locations = get_warehouse(), get_shelves_locations()
        ordered_products = unique_ordered_products(ordered_products)
        shelves_to_pick = min_shelves_greedy(warehouse, ordered_products, shelves_locations)
        task_shelves_coordinates = [ast.literal_eval(item) for item in shelves_to_pick]
        return task_shelves_coordinates
    return []

def main():
    print("Task allocation server starting.....")
    rclpy.init()

    # Controller for the costmap layer to be able to compute paths with a clear costmap
    local_robot_layer_controller = RobotLayerController('/robot1/local_costmap/local_costmap/set_parameters')
    global_robot_layer_controller = RobotLayerController('/robot1/global_costmap/global_costmap/set_parameters')

    # set the number of robots and create nav2 object for each robot
    robots_num = 2
    
    nav_list = []
    print("creating nav2 ojects.....")
    for i in range(robots_num):
        robot_name = 'robot' + str(i + 1)
        nav_list.append(BasicNavigator(namespace=robot_name))

    print("starting NAV2.....")
    for nav in nav_list:
        nav.waitUntilNav2Active()

    # min tasls before starting task allocaiton
    min_tasks = robots_num
    # waiting time before retriving again from queue
    waiting_time = 5

    tasks_queue = []
    # for testing to add shelves directly tasks_queue = [(0,6), (3,6), (0,0), (3,0)]
    #tasks_queue = [(1.6, -1.6), (1.6, 1.6)]

    try:
        while True:
            print("starting new iteration.....")
            #tasks_queue.extend(get_shelves_to_pick())
            print("Tasks: ", tasks_queue)
            
            #wait until collecting min task(8.0, 8.0)
            while len(tasks_queue) < min_tasks:
                print("waiting for tasks....")
                time.sleep(waiting_time)
                tasks_queue.extend(get_shelves_to_pick())
                print("Tasks: ", tasks_queue)


            # setting input parameters for Genetic algorithm
            task_shelves_coordinates = tasks_queue
            tasks_num = len(task_shelves_coordinates)
            task_shelves_poses = make_pose_stamps(task_shelves_coordinates, nav)
            robots_dict = make_robots_dict("pose.csv")
            robots_coordinates = robots_dict.values()
            robots_poses = make_pose_stamps(robots_coordinates, nav)
            picking_stations_coordinates = [(4.0, -2.0), (-4.0, 2.0), (4.0, 2.0), (-4.0, -2.0)] #picking_stations_coordinates = getSlotsLocations('picking')
            picking_stations_poses = make_pose_stamps(picking_stations_coordinates, nav)
            pop_size = 100
            max_epochs = 1000
            crossover_rate = 0.95
            elitist_percentage = 20
            distance_strategy = nav_distance
            nav = nav_list[0]
            
            #disable robots layer before computing costs
            costmap_controller(local_robot_layer_controller, global_robot_layer_controller, False)
            
            nav_list[0].clearAllCostmaps()

            robot_tasks = genetic_alg(
                pop_size=pop_size,
                max_epochs=max_epochs,
                crossover_rate=crossover_rate,
                num_robots=robots_num, 
                num_tasks=tasks_num, 
                elitist_percentage=elitist_percentage,
                robots_coordinates=robots_coordinates, 
                robots_poses=robots_poses, 
                task_shelves_coordinates=task_shelves_coordinates, 
                task_shelves_poses=task_shelves_poses, 
                distance_strategy=distance_strategy,
                picking_stations_coordinates = picking_stations_coordinates,
                picking_stations_poses=picking_stations_poses,
                nav=nav)
            
            #enable robots layer before navigation
            costmap_controller(local_robot_layer_controller, global_robot_layer_controller, True)
            
            #creating waypoints
            robots_waypoints = []

            print("creating waypoints.....")
            for robot in robot_tasks:
                waypoints = []
                for task in robot:
                    # go to shelf
                    waypoints.append(task_shelves_poses[task[0] - 1])
                    # go to picking station
                    waypoints.append(picking_stations_poses[task[1]])
                    # return to shelf
                    waypoints.append(task_shelves_poses[task[0] - 1])
                robots_waypoints.append(waypoints)
            print("Starting Navigation.....")
            navigation_start = time.time()
            for i, nav in enumerate(nav_list):
                nav.followWaypoints(robots_waypoints[i])
            
            while not all(nav.isTaskComplete() for nav in nav_list):
                pass
            #         feedback = nav.getFeedback()
            #         # print(feedback)

            # --- Get the result ---
            for nav in nav_list:
                print(nav.getResult())
                #handle completed tasks
            tasks_queue.clear()
            navigation_end = time.time()
            print("Navigation time:", navigation_end - navigation_start, "s")
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()