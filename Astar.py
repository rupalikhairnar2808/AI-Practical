import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Estimated cost from current node to goal
        self.f = 0  # Total cost (g + h)

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def heuristic(node_position, goal_position):
    # Heuristic function: Manhattan distance
    return abs(node_position[0] - goal_position[0]) + abs(node_position[1] - goal_position[1])

def astar_search(grid, start, goal):
    # Create start and goal nodes
    start_node = Node(start)
    goal_node = Node(goal)
    
    # Initialize open and closed lists
    open_list = []
    closed_list = set()
    
    # Add start node to open list and initialize priority queue
    heapq.heappush(open_list, start_node)
    
    while open_list:
        # Get the node with the lowest f value from the open list
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)
        
        # If we reached the goal, reconstruct the path and return it
        if current_node == goal_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        
        # Generate children (neighboring nodes)
        children = []
        for new_position in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  # 4 possible movements (up, down, right, left)
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1])
            
            # Make sure the move is within grid bounds and not an obstacle
            if (0 <= node_position[0] < len(grid)) and (0 <= node_position[1] < len(grid[0])):
                if grid[node_position[0]][node_position[1]] != 1:  # 1 represents an obstacle
                    children.append(Node(node_position, current_node))
        
        # Process each child
        for child in children:
            if child.position in closed_list:
                continue
            
            # Calculate g, h, and f values
            child.g = current_node.g + 1  # Assume cost of each step is 1
            child.h = heuristic(child.position, goal_node.position)
            child.f = child.g + child.h
            
            # If the child is already in the open list with a higher f value, skip it
            if any(open_node == child and open_node.f <= child.f for open_node in open_list):
                continue
            
            # Add the child to the open list
            heapq.heappush(open_list, child)
    
    return None  # If the goal cannot be reached

def main():
    # Get grid size from the user
    rows = int(input("Enter the number of rows in the grid: "))
    cols = int(input("Enter the number of columns in the grid: "))
    
    # Initialize the grid
    grid = []
    print("Enter the grid (0 for open space, 1 for obstacles):")
    for _ in range(rows):
        grid.append(list(map(int, input().strip().split())))
    
    # Get the start and goal points from the user
    print("Enter the starting point (row and column):")
    start = tuple(map(int, input().strip().split()))
    
    print("Enter the goal point (row and column):")
    goal = tuple(map(int, input().strip().split()))
    
    # Perform A* search
    path = astar_search(grid, start, goal)
    
    # Output the result
    if path:
        print("Path found:")
        for position in path:
            print(position)
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
