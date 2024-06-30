from warehouseDBFunctions import get_shelves_locations
def unique_ordered_products(ordered_products):
    """
    Aggregates the quantities for each unique product_id in the ordered products.

    Args:
        ordered_products (list of dict): A list of dictionaries, each containing a product_id and quantity.

    Returns:
        list of dict: A list of dictionaries with unique product_id and summed quantities.
    """
    product_quantities = {}

    # Iterate through the ordered products and aggregate quantities
    for product in ordered_products:
        product_id = product['product_id']
        quantity = product['quantity']
        if product_id in product_quantities:
            product_quantities[product_id] += quantity
        else:
            product_quantities[product_id] = quantity

    # Convert the aggregated quantities back into a list of dictionaries
    unique_ordered_products = [{'product_id': pid, 'quantity': qty} for pid, qty in product_quantities.items()]

    return unique_ordered_products

def min_shelves_greedy(warehouse, order_items, shelves_locations):
    """
    Uses a greedy approach to find the minimum number of shelves required to pick 
    all items in the order and returns the list of shelf coordinates to visit.

    Args:
        warehouse (list of dict): A list of dictionaries, where each dictionary contains
                                  a shelf_id and items (a dictionary of product IDs and quantities).
        order_items (list of dict): A list of dictionaries, each containing the product ID and the required quantity.
        dynamodb (boto3.resource): The DynamoDB resource.

    Returns:
        list: A list of coordinates for the shelves to pick items from.
    """
    # Convert order items to a dictionary for efficient lookup
    order_items_dict = {item['product_id']: item['quantity'] for item in order_items}
    
    # Track visited shelves and the remaining quantities needed for each item
    visited_shelves = set()
    remaining_items = order_items_dict.copy()

    # Iterate until all items are covered
    while remaining_items:
        # Find the shelf that covers the most uncovered items with sufficient quantities
        max_covered = 0
        best_shelf = None
        best_shelf_items = None

        for shelf in warehouse:
            shelf_id = shelf['shelf_id']
            shelf_items = shelf['items']
            if shelf_id not in visited_shelves:
                # Calculate the number of items that can be covered by this shelf
                covered_items = sum(
                    min(shelf_items.get(product_id, 0), quantity)
                    for product_id, quantity in remaining_items.items()
                )
                if covered_items > max_covered:
                    max_covered = covered_items
                    best_shelf = shelf_id
                    best_shelf_items = shelf_items

        # If no shelf can cover any remaining items, break the loop to prevent infinite loop
        if best_shelf is None:
            print("No more shelves can cover remaining items.")
            break

        # Add the best shelf to visited shelves and update remaining items
        visited_shelves.add(best_shelf)
        items_to_remove = []
        for product_id, quantity in remaining_items.items():
            if product_id in best_shelf_items:
                remaining_items[product_id] -= min(best_shelf_items[product_id], quantity)
                if remaining_items[product_id] <= 0:
                    items_to_remove.append(product_id)
        # Remove items that are fully covered
        for product_id in items_to_remove:
            del remaining_items[product_id]

    # Get coordinates for the selected shelves
    shelves_to_pick = [shelves_locations[shelf_id] for shelf_id in visited_shelves]

    return shelves_to_pick


"""

# Example warehouse inventory (each shelf's items and quantities)
warehouse = [
    {'shelf_id': 'shelf0', 'items': {'item1': 5, 'item2': 2}},  # Shelf 0
    {'shelf_id': 'shelf1', 'items': {'item1': 1, 'item3': 3}},  # Shelf 1
    {'shelf_id': 'shelf2', 'items': {'item2': 7, 'item4': 6}},  # Shelf 2
    {'shelf_id': 'shelf3', 'items': {'item3': 4, 'item4': 1}},  # Shelf 3
]

# Example order items (items and their required quantities)
ordered_items = [
    {'product_id': 'item1', 'quantity': 4},
    {'product_id': 'item2', 'quantity': 6},
    {'product_id': 'item3', 'quantity': 2},
]

# Mock DynamoDB resource
dynamodb = None  # Replace with actual DynamoDB resource in real usage

# Mock get_shelves_locations function
def get_shelves_locations(dynamodb):
    return {
        'shelf0': (0, 0),
        'shelf1': (1, 0),
        'shelf2': (0, 1),
        'shelf3': (1, 1),
    }

# Function call
shelves_to_pick = min_shelves_greedy(warehouse, ordered_items, dynamodb)

# Output
print(shelves_to_pick)

"""