#### AWS configuration

- setup aws on linux
  
      sudo apt install awscli
  
- configure aws
      aws configure (use access key and secret access key provided from the AWS IAM and the region the services is used in)
  
- install requriments
  
        pip3 install -r requirements.txt
  
#### Running Task Server

    cd task_allocation/task_allocation
    python3 main.py

#### Set_cover_greedy

Formalting the problem into an set cover problem where, The Set Cover Problem is a classical problem in computer science where you're given a set of items to cover and a collection of sets,
each containing some of those items. The objective is to find the smallest subset of the collection such that the union of all the sets in that subset covers all the items.

In our case, the items are stored in multiple shelves, and we want to minimize the number of shelves you need to visit to retrieve a given set of items.
This can be modeled as a set cover problem where the sets represent the shelves and the items represent the elements that need to be covered.

The greedy algorithm is feasiable but not optimal, it's faster than the combination method that has to compute every possible combination and choose the minimum.

#### Genetic Algorithm for task allocation in warehouse ####

Genetic Algorithm is a search method inspired from natural evolution and has been successfully applied to a variety of NP-hard combinato-rial optimization problems. It is iterative,
population-based algorithm and follows the principle of survival of the fittest to search solutions through the operations of selection, crossover, and mutation. To apply GA to solve the warehouse
scheduling problem, the focus is to establish an effective chromosome representation and design suitable selection, crossover, and mutation operators to improve the solution quality.

## References

- Dou, Jiajia, Chunlin Chen, and Pei Yang. "Genetic Scheduling and Reinforcement Learning in Multirobot Systems for Intelligent Warehouses." *Journal of Robotics*, vol. 2015, Article ID 597956, 2015. DOI: [10.1155/2015/597956](https://doi.org/10.1155/2015/597956)
