#### DynamoDB and AWS configuration ####

- setup aws on linux
    sudo apt install awscli
- configure aws
    aws configure (use access key and secret access key provided from the AWS IAM and the region the services is used in)
- setup the envirnoment
    in terminal create an envirnoment :
        python3 -m venv .venv
    activate envirnoment:
        source ./.venv/bin/activate
    test the env (running the file):
        python3 config.py
    installing boto3 to connect to aws:
        create a requirments.txt file and add boto3
        pip3 install -r requirments.txt -t ./.venv/lib/python(write ur version)/site-packages/

#### set_cover_greedy ####

    Formalting the problem into an set cover problem where, The Set Cover Problem is a classical problem in computer science where you're given a set of items to cover and a collection of sets,
each containing some of those items. The objective is to find the smallest subset of the collection such that the union of all the sets in that subset covers all the items.

    In our case, the items are stored in multiple shelves, and we want to minimize the number of shelves you need to visit to retrieve a given set of items.
This can be modeled as a set cover problem where the sets represent the shelves and the items represent the elements that need to be covered.

The greedy algorithm is feasiable but not optimal, it's faster than the combination method that has to compute every possible combination and choose the minimum.