# Assuming you have configured your AWS credentials and region
import boto3
import json

dynamodb = boto3.resource('dynamodb')

# Initialize SQS client
sqs_client = boto3.client('sqs')

# SQS OrderProductsQueue URL
queue_url = "https://sqs.eu-north-1.amazonaws.com/381491978736/OrderProductsQueue"
#Get shelves locations from Shelf table
def get_shelves_locations(dynamodb):
    shelf_table = dynamodb.Table('Shelf-ei5ggzbrrjghbe3yjny255f35q-dev')
    shelf_table_response = shelf_table.scan()
    shelves_locations = {shelf['id']: shelf['shelf_coordinate'] for shelf in shelf_table_response['Items']}

    return shelves_locations

def process_message(message, order_products):
    data = json.loads(message['Body'])
    order_products.append({'product_id': data['product_id'], 'quantity': data['quantity']})

def receive_and_process_messages(sqs_client, queue_url):
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
            process_message(message, order_products)
            # Delete the message from the queue
            sqs_client.delete_message(
                QueueUrl=queue_url,
                ReceiptHandle=message['ReceiptHandle']
            )
    return order_products

# Function to fetch order products with associated shelf IDs
def fetching_order_products_with_shelf_IDs(dynamodb, sqs_client, queue_url):
    #order_products = receive_and_process_messages(sqs_client, queue_url)
    order_products = [{'product_id': '7381daac-236f-42b1-8b45-7348d90c68ba', 'quantity': '1'}, {'product_id': '3aaa19ac-592d-4a5a-8c43-062bbbcb3f5e', 'quantity': '1'}, {'product_id': '9fb10680-365f-4cef-ba2f-22b5dd12b662', 'quantity': '1'}, {'product_id': 'c0c6ab69-62d1-4944-83fe-c14b95c85313', 'quantity': '1'}, {'product_id': 'c0c6ab69-62d1-4944-83fe-c14b95c85313', 'quantity': '7'}]
    # Get the 'ShelfProducts' table
    shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')

    order_products_with_shelf_IDs = []
    # Iterate over each order product
    for order_product in order_products:
        # Get the product ID for the current order product
        product_id = order_product['product_id']
        
        # Query the 'ShelfProducts' table to find the shelf IDs for the product
        product_shelfIDs_response = shelf_products_table.query(
            IndexName='byProduct',
            KeyConditionExpression='productID = :pid',
            ExpressionAttributeValues={
                ':pid': product_id
            }
        )
        
        # Extract the list of items from the shelf response
        shelf_products = product_shelfIDs_response['Items']
        # If a shelf ID is found, assign it to the current order product; otherwise, set it to None
        if shelf_products:
            for shelf_product in shelf_products:
                order_products_with_shelf_IDs.append({'productID': product_id, 'quantity':order_product['quantity'],'shelfID': shelf_product['shelfID']})
    return  order_products_with_shelf_IDs





