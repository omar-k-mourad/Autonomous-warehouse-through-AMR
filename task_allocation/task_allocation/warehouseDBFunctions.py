import boto3
import json
client = boto3.client('dynamodb')
def addAMR(AMRId, curr_Loc = '(0,0)', Battery = 'Full', IsFree = True):
    client.put_item(
        TableName='Warehouse',
        Item={
        'PK': { 'S':'AMR'},
        'SK': { 'S': f'AMR#{AMRId}' },
        'Type': { 'S': 'amr' },
        'AMRId': {'S': AMRId},
        'curr_Loc':{'S': curr_Loc},
        'Battery':{'S': Battery },
        'IsFree':{'BOOL': IsFree}
        }
    )
def addSlot(SlotId, slotType, Loc, IsFree = True):
    client.put_item(
        TableName='Warehouse',
        Item={
        'PK': { 'S':'SLOT'},
        'SK': { 'S': f'SLOT#{SlotId}' },
        'Type': { 'S': slotType },
        'SlotId': {'S': SlotId},
        'Loc':{'S': Loc},
        'IsFree':{'BOOL': IsFree}
        }
    )

def GetAllAMRs():
    AMRs = client.query(
        TableName='Warehouse',
        KeyConditionExpression='#pk = :pk',
        ExpressionAttributeNames={
            '#pk': 'PK'
        },
        ExpressionAttributeValues={
            ':pk': { 'S': 'AMR' }
        },
    )
    return AMRs

def GetAllSlots():
    Slots = client.query(
        TableName='Warehouse',
        KeyConditionExpression='#pk = :pk',
        ExpressionAttributeNames={
            '#pk': 'PK'
        },
        ExpressionAttributeValues={
            ':pk': { 'S': 'SLOT' }
        },
    )
    return Slots

def getSlotsLocations(stationType):
    # Query to get all slots of the specified type (picking or replenishment)
    response = client.query(
        TableName='Warehouse',
        IndexName='Type-index',  # Ensure this secondary index exists on 'Type' attribute
        KeyConditionExpression='#type = :type',
        ExpressionAttributeNames={
            '#type': 'Type'
        },
        ExpressionAttributeValues={
            ':type': {'S': stationType}
        }
    )

    # Extract locations from the response and convert them to tuples
    locations = [tuple(map(int, item['Loc']['S'].strip('()').split(','))) for item in response.get('Items', [])]
    return locations

def getItem(PK_value, SK_value):
    # Define the composite primary key of the item
    composite_key = {
        'PK': {'S': PK_value},  # Replace with actual partition key value
        'SK': {'S': SK_value}             # Replace with actual sort key value
    }

    # Retrieve the item from the table
    response = client.get_item(
        TableName='Warehouse',
        Key=composite_key
    )
    return response

def updateAMRStatus(AMRId, Status):
    # Define the composite primary key of the item
    composite_key = {
        'PK': {'S': "AMR"},  # Replace with actual partition key value
        'SK': {'S': f'AMR#{AMRId}'}             # Replace with actual sort key value
    }

    # Define the update expression and attribute values
    update_expression = 'SET IsFree = :val'
    expression_attribute_values = {
        ':val': {'BOOL': Status}  # Example modification to a boolean attribute
    }

    # Update the specific attribute of the item
    client.update_item(
        TableName='Warehouse',
        Key=composite_key,
        UpdateExpression=update_expression,
        ExpressionAttributeValues=expression_attribute_values
    )

def updateSlotStatus(SlotId, Status):
    # Define the composite primary key of the item
    composite_key = {
        'PK': {'S': "SLOT"},  # Replace with actual partition key value
        'SK': {'S': f'SLOT#{SlotId}'}             # Replace with actual sort key value
    }

    # Define the update expression and attribute values
    update_expression = 'SET IsFree = :val'
    expression_attribute_values = {
        ':val': {'BOOL': Status}  # Example modification to a boolean attribute
    }

    # Update the specific attribute of the item
    client.update_item(
        TableName='Warehouse',
        Key=composite_key,
        UpdateExpression=update_expression,
        ExpressionAttributeValues=expression_attribute_values
    )


#addAMR('1', '(0,0)', 'Full',True)
#addAMR('2', '(0,0)', 'Full',True)
#addSlot('1','picking','(0 , 1)',True)
#addSlot('2','picking','(0 , 1)', True)
#updateSlotStatus('SLOT#1', False)
#updateAMRStatus('AMR#2', False)

def get_shelves_locations(dynamodb):
    """
    Fetches the shelf locations from the Shelf table in DynamoDB.

    Args:
    dynamodb (boto3.resource): The DynamoDB resource.

    Returns:
    dict: A dictionary with shelf IDs as keys and shelf coordinates as values.
    """
    shelf_table = dynamodb.Table('Shelf-ei5ggzbrrjghbe3yjny255f35q-dev')
    shelf_table_response = shelf_table.scan()
    shelves_locations = {shelf['id']: shelf['shelf_coordinate'] for shelf in shelf_table_response['Items']}

    return shelves_locations

def get_warehouse(dynamodb):
    """
    Retrieves and constructs the warehouse inventory from the DynamoDB 'ShelfProducts' table.

    Args:
        dynamodb (boto3.resource): The DynamoDB resource.

    Returns:
        list of dict: A list of dictionaries representing the shelves and their items.
    """
    shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')
    shelf_products_table_response = shelf_products_table.scan()
    items = shelf_products_table_response['Items']
    warehouse = {}
    
    # Iterate through items and organize them by shelf_id
    for item in items:
        shelf_id = item['shelfID']
        product_id = item['productID']
        product_count = item['shelf_item_count']
        
        if shelf_id not in warehouse:
            warehouse[shelf_id] = {'shelf_id': shelf_id, 'items': {}}
        
        warehouse[shelf_id]['items'][product_id] = product_count
    
    # Convert the warehouse dictionary to a list
    warehouse_list = list(warehouse.values())

    return warehouse_list
    
def process_message(message):
    """
    Processes an SQS message to extract product information.

    Args:
    message (dict): The SQS message.

    Returns:
    dict: A dictionary with product ID and quantity.
    """
    data = json.loads(message['Body'])
    return {'product_id': data['product_id'], 'quantity': data['quantity']}

def get_ordered_products(sqs_client, queue_url):
    """
    Receives and processes messages from an SQS queue.

    Args:
    sqs_client (boto3.client): The SQS client.
    queue_url (str): The URL of the SQS queue.

    Returns:
    list: A list of dictionaries containing product IDs and quantities.
    """
    ordered_products = []
    
    while True:
        response = sqs_client.receive_message(
            QueueUrl=queue_url,
            MaxNumberOfMessages=1,  # Adjust batch size as needed
            WaitTimeSeconds=0
        )
        
        messages = response.get('Messages', [])
        
        if not messages:
            break
        
        for message in messages:
            ordered_product = process_message(message)
            ordered_products.append(ordered_product)
            
            # Delete the message from the queue
            sqs_client.delete_message(
                QueueUrl=queue_url,
                ReceiptHandle=message['ReceiptHandle']
            )
    
    return ordered_products
print(getSlotsLocations('picking'))