# Assuming you have configured your AWS credentials and region
import boto3

# Initialize the DynamoDB resource
dynamodb = boto3.resource('dynamodb')

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
    # Get the 'OrderProducts' table
    table = dynamodb.Table('OrderProducts-ei5ggzbrrjghbe3yjny255f35q-dev')
    
    # Scan the table to retrieve all items
    response = table.scan()
    
    # Extract the list of items from the response
    order_products = response['Items']

    # Get the 'ShelfProducts' table
    shelf_products_table = dynamodb.Table('ShelfProducts-ei5ggzbrrjghbe3yjny255f35q-dev')

    # Iterate over each order product
    for order_product in order_products:
        # Get the product ID for the current order product
        product_id = order_product['productID']
        
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

    # Return the list of order products with associated shelf IDs
    return order_products


op = get_order_products_with_shelf_id()
print(type(op))
print(op)

"""

OUTPUT

<class 'list'>
[{'__typename': 'OrderProducts', 'order_item_count': Decimal('5'), 'orderID': '9883ef29-7410-41a6-a2d5-16950c1d44a6',
'updatedAt': '2024-04-20T17:45:55.874Z', 'createdAt': '2024-04-20T17:45:55.874Z', 'id': '8789aeeb-87a7-4653-b7d5-c6feaaa4bb90',
'productID': '7381daac-236f-42b1-8b45-7348d90c68ba', 'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'}, {'__typename': 'OrderProducts',
'order_item_count': Decimal('3'), 'orderID': '9883ef29-7410-41a6-a2d5-16950c1d44a6', 'updatedAt': '2024-04-20T17:48:16.537Z', 
'createdAt': '2024-04-20T17:48:16.537Z', 'id': '9135fb1d-7655-4138-855d-31f25a9a75a2', 'productID': '3aaa19ac-592d-4a5a-8c43-062bbbcb3f5e',
 'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'}]

"""