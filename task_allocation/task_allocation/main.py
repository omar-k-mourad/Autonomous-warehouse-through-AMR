from nav2_commander import *
from genetic_algorithm import *
from fetching_orderProducts_with_shelfIDs import *
from set_cover_greedy import *
from clean_data import *
import ast



def main():
    rclpy.init()
    nav = BasicNavigator()
    # Initialize the DynamoDB resource
    dynamodb = boto3.resource('dynamodb')

    # Initialize SQS client
    sqs_client = boto3.client('sqs')

    # SQS OrderProductsQueue URL
    queue_url = "https://sqs.eu-north-1.amazonaws.com/381491978736/OrderProductsQueue"

    order_products = fetching_order_products_with_shelf_IDs(dynamodb, sqs_client, queue_url)
    print("order products:", order_products)
    reduced_order_products_shelves = extract_product_and_shelf(order_products)
    print("rduced", reduced_order_products_shelves)
    order_items, shelfIDs, warehouse = Transform_response(reduced_order_products_shelves)
    print("TRansform", warehouse, order_items, shelfIDs)
    shelves_to_pick = min_shelves_greedy(warehouse, order_items, shelfIDs, dynamodb)
    task_shelves_coordinates = [ast.literal_eval(item) for item in shelves_to_pick]
    print(task_shelves_coordinates)

    """ 
    tasks_num = len(task_shelves_coordinates)
    task_shelves_poses = make_pose_stamps(task_shelves_coordinates, nav)
    
    
    robots_num = 3
    robots_dict = make_robots_dict("pose.csv")
    robots_coordinates = robots_dict.values()
    robots_poses = make_pose_stamps(robots_coordinates, nav)

    pop_size = 100
    MaxEpoc = 10000
    crossover_rate = 0.95
    elitist_precentage = 20
    distance_strategy = nav_distance
    nav = nav

    genetic_alg(
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
    """
    rclpy.shutdown()


if __name__ == '__main__':
    main()
