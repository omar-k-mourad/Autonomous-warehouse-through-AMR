
def min_shelves_greedy(warehouse, order_items, shelfIDs):
  """
  This function uses a greedy approach to find the minimum number of shelves 
  required to pick all items in the order, and returns the list of shelves to visit.

  Args:
      warehouse: A list of lists, where each sub-list represents the items on a shelf.
      order_items: A list of items the customer ordered.

  Returns:
      extracted_shelfs : list of shelfs id to pick
  """
  # Create a set of all items in the order.
  all_items = set(order_items)
  remaining_items = set(order_items)

  # Track visited shelves.
  visited_shelves = set()

  # Iterate until all items are covered.
  while remaining_items:
    # Find the shelf that covers the most uncovered items.
    max_covered = 0
    best_shelf = None
    for shelf_id, shelf_items in enumerate(warehouse):
      if shelf_id not in visited_shelves:
        covered_items = len(remaining_items.intersection(set(shelf_items)))
        if covered_items > max_covered:
          max_covered = covered_items
          best_shelf = shelf_id

    # Add the best shelf to visited shelves and update remaining items.
    if best_shelf is not None:
      visited_shelves.add(best_shelf)
      remaining_items -= set(warehouse[best_shelf])

  extracted_shelfs = [shelfIDs[i] for i in list(visited_shelves)]

  return extracted_shelfs

# Example usage
warehouse = [['1', '3', '2'], ['1']]
order_items = ['2', '1', '3']
shelfIDS = ['2', '4']
    
shelves_to_pick = min_shelves_greedy(warehouse, order_items, shelfIDS)

print(f"Shelves to pick: {shelves_to_pick}")

