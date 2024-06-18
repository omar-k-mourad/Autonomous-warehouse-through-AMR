import boto3
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

updateSlotStatus('SLOT#1', False)
updateAMRStatus('AMR#2', False)