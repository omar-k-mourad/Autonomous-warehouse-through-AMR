from nav2_commander import *
from genetic_algorithm import *
from set_cover_greedy import *
from clean_data import *
from warehouseDBFunctions import *
import ast
import time
import boto3

def get_shelves_to_pick(dynamodb, sqs_client, queue_url):
    warehouse, ordered_products, shelves_locations = get_warehouse(dynamodb), get_ordered_products(sqs_client, queue_url), get_shelves_locations(dynamodb)
    ordered_products = unique_ordered_products(ordered_products)
    shelves_to_pick = min_shelves_greedy(warehouse, ordered_products, shelves_locations)
    task_shelves_coordinates = [ast.literal_eval(item) for item in shelves_to_pick]
    return task_shelves_coordinates

def main():
    print("Task allocation server starting.....")
    rclpy.init()
    nav = BasicNavigator(namespace='robot1')

    # Initialize the DynamoDB resource
    dynamodb = boto3.resource('dynamodb')

    # Initialize SQS client
    sqs_client = boto3.client('sqs')

    # SQS OrderProductsQueue URL
    queue_url = "https://sqs.eu-north-1.amazonaws.com/381491978736/OrderProductsQueue"
    tasks = get_shelves_to_pick(dynamodb, sqs_client, queue_url)
    print("Tasks: ", tasks)

    min_tasks = 2
    waiting_time = 10

    tasks_queue = []
    try:
        while True:
            print("starting new iteration.....")

            tasks = get_shelves_to_pick(dynamodb, sqs_client, queue_url)
            tasks_queue.extend(tasks)
            
            #wait until collecting min task
            while len(tasks_queue) < min_tasks:
                print("waiting for tasks....")
                time.sleep(waiting_time)
                tasks = get_shelves_to_pick(dynamodb, sqs_client, queue_url)
                tasks_queue.append(tasks)


            # setting input parameters for Genetic algorithm
            task_shelves_coordinates = tasks_queue
            tasks_num = len(task_shelves_coordinates)
            task_shelves_poses = make_pose_stamps(task_shelves_coordinates, nav)
            robots_num = 1
            robots_dict = make_robots_dict("pose.csv")
            robots_coordinates = robots_dict.values()
            robots_poses = make_pose_stamps(robots_coordinates, nav)
            picking_stations_coordinates = [(0.0,-2.0), (-1.0,-2.0), (1.0,-2.0)]
            picking_stations_poses = make_pose_stamps(picking_stations_coordinates, nav)
            pop_size = 100
            MaxEpoc = 10000
            crossover_rate = 0.95
            elitist_precentage = 20
            distance_strategy = nav_distance
            nav = nav

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
                picking_stations_coordinates = picking_stations_coordinates,
                picking_stations_poses=picking_stations_poses,
                nav=nav)
            
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
                    # go to shelf
                    waypoints.append(task_shelves_poses[task[0] - 1])
                    # go to picking station
                    waypoints.append(picking_stations_poses[task[1]])
                    # return to shelf
                    waypoints.append(task_shelves_poses[task[0] - 1])
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
                #handle completed tasks
                tasks_queue.clear()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
