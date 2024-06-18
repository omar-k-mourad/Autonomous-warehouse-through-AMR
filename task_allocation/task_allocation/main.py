from nav2_commander import *
from genetic_algorithm import *



def main():
    rclpy.init()
    nav = BasicNavigator()

    task_shelves_coordinates =  [(2.0, 0.6), (2.0, 0.0), (2.0, -0.6),
                                (0.2, -0.4), (0.2, -0.6), (-2.0, 0.6)]
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
    distance_strategy = euclidean_distance
    nav = None

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

    rclpy.shutdown()


if __name__ == '__main__':
    main()
