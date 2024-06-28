import json

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

def receive_and_process_messages(sqs_client, queue_url):
    """
    Receives and processes messages from an SQS queue.

    Args:
    sqs_client (boto3.client): The SQS client.
    queue_url (str): The URL of the SQS queue.

    Returns:
    list: A list of dictionaries containing product IDs and quantities.
    """
    order_products = []
    
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
            order_product = process_message(message)
            order_products.append(order_product)
            
            # Delete the message from the queue
            sqs_client.delete_message(
                QueueUrl=queue_url,
                ReceiptHandle=message['ReceiptHandle']
            )
    
    return order_products

def fetch_order_products_with_shelf_ids(dynamodb, sqs_client, queue_url):
    """
    Fetches order products along with their associated shelf IDs.

    Args:
    dynamodb (boto3.resource): The DynamoDB resource.
    sqs_client (boto3.client): The SQS client.
    queue_url (str): The URL of the SQS queue.

    Returns:
    list: A list of dictionaries containing product IDs, quantities, and shelf IDs.
    """
    order_products = receive_and_process_messages(sqs_client, queue_url)
    shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')

    order_products_with_shelf_ids = []
    
    for order_product in order_products:
        product_id = order_product['product_id']
        
        # Query the 'ShelfProducts' table to find the shelf IDs for the product
        product_shelf_ids_response = shelf_products_table.query(
            IndexName='byProduct',
            KeyConditionExpression='productID = :pid',
            ExpressionAttributeValues={
                ':pid': product_id
            }
        )
        
        shelf_products = product_shelf_ids_response.get('Items', [])
        
        if shelf_products:
            for shelf_product in shelf_products:
                order_products_with_shelf_ids.append({
                    'productID': product_id,
                    'quantity': order_product['quantity'],
                    'shelfID': shelf_product['shelfID']
                })
    
    return order_products_with_shelf_ids






