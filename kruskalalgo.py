class DisjointSet:
    def __init__(self, n):
        # Initialize the parent array and rank array for the disjoint set
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, u):
        # Find the root of the set containing u with path compression
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        # Union the sets containing u and v using union by rank
        root_u = self.find(u)
        root_v = self.find(v)
        
        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1
            return True
        return False

def kruskal_mst(edges, n):
    # Sort edges by weight
    edges.sort(key=lambda edge: edge[2])
    
    # Initialize the disjoint set
    disjoint_set = DisjointSet(n)
    
    # List to store the edges in the minimal spanning tree (MST)
    mst = []
    
    # Process each edge
    for edge in edges:
        u, v, weight = edge
        
        # Use union-find to check if the edge forms a cycle
        if disjoint_set.union(u, v):
            # If the edge does not form a cycle, add it to the MST
            mst.append(edge)
        
        # If the MST has n-1 edges, stop (MST complete)
        if len(mst) == n - 1:
            break
    
    return mst

def main():
    # Get the number of vertices and edges from the user
    n = int(input("Enter the number of vertices in the graph: "))
    m = int(input("Enter the number of edges in the graph: "))
    
    # Initialize the list of edges
    edges = []
    
    # Get each edge from the user
    print("Enter each edge in the format 'u v weight':")
    for _ in range(m):
        u, v, weight = map(int, input().strip().split())
        edges.append((u, v, weight))
    
    # Perform Kruskal's algorithm to find the MST
    mst = kruskal_mst(edges, n)
    
    # Output the result
    print("\nMinimal Spanning Tree:")
    total_weight = 0
    for u, v, weight in mst:
        print(f"Edge ({u}, {v}) with weight {weight}")
        total_weight += weight
    print(f"\nTotal weight of the MST: {total_weight}")

if __name__ == "__main__":
    main()
