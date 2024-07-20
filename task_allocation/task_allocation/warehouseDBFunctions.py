import boto3
import json
import uuid
from boto3.dynamodb.conditions import Key
from datetime import datetime

# Initialize the DynamoDB resource and client
client = boto3.client('dynamodb')
dynamodb = boto3.resource('dynamodb')
# Initialize SQS client
sqs_client = boto3.client('sqs')
# SQS OrderProductsQueue URL
queue_url = "https://sqs.eu-north-1.amazonaws.com/381491978736/OrderProductsQueue"

shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')
product_table = dynamodb.Table('Product-ei5ggzbrrjghbe3yjny255f35q-dev')
shelf_table = dynamodb.Table('Shelf-ei5ggzbrrjghbe3yjny255f35q-dev')

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

def get_shelves_locations():
    """
    Fetches the shelf locations from the Shelf table in DynamoDB.

    Args:
    dynamodb (boto3.resource): The DynamoDB resource.

    Returns:
    dict: A dictionary with shelf IDs as keys and shelf coordinates as values.
    """
    shelf_table_response = shelf_table.scan()
    shelves_locations = {shelf['id']: shelf['shelf_coordinate'] for shelf in shelf_table_response['Items']}

    return shelves_locations

def get_warehouse():
    """
    Retrieves and constructs the warehouse inventory from the DynamoDB 'ShelfProducts' table.

    Args:
        dynamodb (boto3.resource): The DynamoDB resource.

    Returns:
        list of dict: A list of dictionaries representing the shelves and their items.
    """
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

def get_ordered_products():
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

def current_timestamp():
    # Return the current UTC time in ISO 8601 format with 'Z' at the end
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'

def manage_product_on_shelf(action, shelfID, productID, quantity):
    """
    Manages products on shelves by either picking or storing items.

    Parameters:
    - action (str): The action to perform; either 'pick' or 'store'.
    - shelfID (str): The ID of the shelf where the product is located or being added.
    - productID (str): The ID of the product to be picked or stored.
    - quantity (int): The quantity of the product to be picked or added.

    Behavior:
    - If action is 'pick':
        - Queries the ShelfProducts table for the specified shelfID and productID.
        - If the product is found, it updates the `shelf_item_count` by subtracting the specified quantity.
        - If the updated `shelf_item_count` is zero or less, it deletes the record from the ShelfProducts table.
        - Updates the `ProductLefts` in the Product table by subtracting the quantity.
    
    - If action is 'store':
        - Queries the ShelfProducts table for the specified shelfID and productID.
        - If the product is found, it updates the `shelf_item_count` by adding the specified quantity.
        - If the product is not found, it creates a new record in the ShelfProducts table with a unique ID, setting `createdAt` and `updatedAt` to the current timestamp.
        - Updates the `ProductLefts` in the Product table by adding the quantity.

    Exceptions:
    - Catches and prints errors encountered during the process.
    """
    try:
        # Query the ShelfProducts table using the byShelf index
        response = shelf_products_table.query(
            IndexName='byShelf',
            KeyConditionExpression=Key('shelfID').eq(shelfID)
        )

        # Filter the results for the specific productID
        items = response.get('Items', [])
        shelf_product = next((item for item in items if item['productID'] == productID), None)

        if action == 'pick':
            if not shelf_product:
                print('Product not found on the shelf.')
                return

            # Calculate the updated quantity for ShelfProducts
            updated_shelf_quantity = shelf_product['shelf_item_count'] - quantity

            if updated_shelf_quantity > 0:
                # Update the quantity and updatedAt in ShelfProducts
                shelf_products_table.update_item(
                    Key={
                        'id': shelf_product['id']
                    },
                    UpdateExpression='SET shelf_item_count = :val1, updatedAt = :val2',
                    ExpressionAttributeValues={
                        ':val1': updated_shelf_quantity,
                        ':val2': current_timestamp()
                    }
                )
                print('Product quantity on shelf updated successfully.')
            else:
                # Delete the record in ShelfProducts if quantity is 0 or less
                shelf_products_table.delete_item(
                    Key={
                        'id': shelf_product['id']
                    }
                )
                print('Product removed from shelf due to zero quantity.')

            # Update the ProductLefts in the Product table
            product_table.update_item(
                Key={
                    'id': productID
                },
                UpdateExpression='SET ProductLefts = ProductLefts - :val3, updatedAt = :val4',
                ConditionExpression='ProductLefts >= :val3',  # Ensure ProductLefts does not go negative
                ExpressionAttributeValues={
                    ':val3': quantity,
                    ':val4': current_timestamp()
                }
            )
            print('ProductLefts updated successfully in Product table.')

        elif action == 'store':
            if shelf_product:
                # If the product is already on the shelf, update the quantity and updatedAt
                new_shelf_quantity = shelf_product['shelf_item_count'] + quantity
                shelf_products_table.update_item(
                    Key={
                        'id': shelf_product['id']
                    },
                    UpdateExpression='SET shelf_item_count = :val1, updatedAt = :val2',
                    ExpressionAttributeValues={
                        ':val1': new_shelf_quantity,
                        ':val2': current_timestamp()
                    }
                )
                print('Product quantity on shelf updated successfully.')
            else:
                # If the product is not on the shelf, create a new record with a generated id
                shelf_products_table.put_item(
                    Item={
                        'id': str(uuid.uuid4()),  # Generate a unique ID using UUID
                        'shelfID': shelfID,
                        'productID': productID,
                        'shelf_item_count': quantity,
                        'createdAt': current_timestamp(),
                        'updatedAt': current_timestamp(),
                        '__typename': 'ShelfProducts'
                    }
                )
                print('New product added to the shelf successfully.')

            # Update the ProductLefts in the Product table
            product_table.update_item(
                Key={
                    'id': productID
                },
                UpdateExpression='SET ProductLefts = ProductLefts + :val3, updatedAt = :val4',
                ExpressionAttributeValues={
                    ':val3': quantity,
                    ':val4': current_timestamp()
                }
            )
            print('ProductLefts updated successfully in Product table.')

    except Exception as e:
        print('Error managing product:', e)