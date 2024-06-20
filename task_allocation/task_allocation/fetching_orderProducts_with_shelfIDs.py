# Assuming you have configured your AWS credentials and region
import boto3
import json

# Initialize the DynamoDB resource
dynamodb = boto3.resource('dynamodb')

# Initialize SQS client
sqs_client = boto3.client('sqs')

# SQS OrderProductsQueue URL
queue_url = "https://sqs.eu-north-1.amazonaws.com/381491978736/OrderProductsQueue"

order_products = []
def process_message(message):
    data = json.loads(message['Body'])
    order_products.append({'product_id': data['product_id'], 'quantity': data['quantity']})

def receive_and_process_messages():
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
            process_message(message)
            # Delete the message from the queue
            sqs_client.delete_message(
                QueueUrl=queue_url,
                ReceiptHandle=message['ReceiptHandle']
            )

# Function to fetch order products with associated shelf IDs
def get_order_products_with_shelf_id():
    """
    This function get the products from orders and get the respective shelves and store it in a list.
    
    Args:
        NAN.

    Returns:
        A List of Dictionaries containing:
            - {TypeName, order_item_count, orderID, updateAt, createdAt, id, productID, shelfID}
    """

    # Get the 'ShelfProducts' table
    shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')

    # Iterate over each order product
    for order_product in order_products:
        # Get the product ID for the current order product
        product_id = order_product['product_id']
        
        # Query the 'ShelfProducts' table to find the shelf ID for the product
        shelf_response = shelf_products_table.query(
            IndexName='byProduct',
            KeyConditionExpression='productID = :pid',
            ExpressionAttributeValues={
                ':pid': product_id
            }
        )
        
        # Extract the list of items from the shelf response
        shelf_products = shelf_response['Items']
        
        # If a shelf ID is found, assign it to the current order product; otherwise, set it to None
        if shelf_products:
            order_product['shelfID'] = shelf_products[0]['shelfID']
        else:
            order_product['shelfID'] = None


receive_and_process_messages()
get_order_products_with_shelf_id()
print(order_products)

