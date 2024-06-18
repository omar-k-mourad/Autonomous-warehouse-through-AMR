from boto3 import resource
from boto3.dynamodb.conditions import Attr, Key
from datetime import datetime

# connecting machine to database table

demo_table = resource('dynamodb').Table('Order')

#############################  insert record #############################

def insert():
    print(f'demo_insert')
    response = demo_table.put_item(
        Item={
                'order_id': 'ord-01', # parition key
                'customer_id' : 'cus-5',  # sort key
                'status': 'pending',
                'created_date' : datetime.now().isoformat()
            }
        )
    print(f'Insert response: {response}') 

insert()