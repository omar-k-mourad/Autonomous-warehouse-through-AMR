
def reduce_response(response, desired_keys):
    """
    This function reduces the keys of the response to the desired keys.

    Args:
        response: list of dictionaries.
        desired_keys: list of keys to reduce to.

    Returns:
        reduced_response : list of dictionaries after reducing the response.
    """

    """
    ## input sample ##

    order_self_response = [
                            {'__typename': 'OrderProducts', 'order_item_count': '5', 'orderID': '9883ef29-7410-41a6-a2d5-16950c1d44a6',
                             'updatedAt': '2024-04-20T17:45:55.874Z', 'createdAt': '2024-04-20T17:45:55.874Z', 
                             'id': '8789aeeb-87a7-4653-b7d5-c6feaaa4bb90',
                             'productID': '7381daac-236f-42b1-8b45-7348d90c68ba', 'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'},

                            {'__typename': 'OrderProducts','order_item_count': '3', 'orderID': '9883ef29-7410-41a6-a2d5-16950c1d44a6',
                             'updatedAt': '2024-04-20T17:48:16.537Z', 'createdAt': '2024-04-20T17:48:16.537Z',
                             'id': '9135fb1d-7655-4138-855d-31f25a9a75a2', 'productID': '3aaa19ac-592d-4a5a-8c43-062bbbcb3f5e',
                             'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'}
                        ]

    ## output sample ##

    [{'productID': '7381daac-236f-42b1-8b45-7348d90c68ba', 'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'},
     {'productID': '3aaa19ac-592d-4a5a-8c43-062bbbcb3f5e', 'shelfID': '069d030a-a9c4-4116-88d5-f2bfd09456e9'}]

    """

    reduced_response = []

    for item in response:
        item = {key: value for key, value in item.items() if key in desired_keys}
        reduced_response.append(item)

    return reduced_response

def extract_product_and_shelf(data):
    result = []
    for item in data:
        product_and_shelf = {
            'product_id': item['product_id'],
            'shelfID': item['shelfID']
        }
        result.append(product_and_shelf)
    return result

def Transform_response(response):
  """
  This function takes a list of dictionaries (products) with 'productID' and 'shelfID' keys
  and returns
      1. unique_products : list of Unique product IDs
      2. shelves : list of shelfs ID 
      3. list with the respective item on the shelf (same index)
  """
    
  """
  ## input sample ##

  products = [
    {'productID': '1', 'shelfID': '2'},
    {'productID': '3', 'shelfID': '2'},
    {'productID': '1', 'shelfID': '4'},
    {'productID': '2', 'shelfID': '2'}  # Duplicate productID
    ]
  ## output sample ##

    ['2', '1', '3']
    ['2', '4']
    [['1', '3', '2'], ['1']]

  """
  unique_products = set()  # Use a set for efficient removal of duplicates
  shelves = {}

  for product in response:
    product_id = product["product_id"]
    shelf_id = product["shelfID"]

    unique_products.add(product_id)

    # Initialize shelf if not present
    if shelf_id not in shelves:
      shelves[shelf_id] = []

    shelves[shelf_id].append(product_id)

  return list(unique_products), list(shelves), list(shelves.values())
