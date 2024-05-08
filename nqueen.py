class Queen:
    
    def __init__(self, N):
        self.N= N
        #chessboard
        #NxN matrix with all elements 0
        self.board = [[0]*N for _ in range(N)]
        
    def disp_board(self):
        for row in self.board:
            print()
            for col in row:
                if col==1:
                    print(u"\U0001F451", end=' ')
                else:
                    print(u"\u274C", end=' ')
        print(end= '\n')
        
    def is_attack(self, i, j):
        #checking if there is a queen in row or column
        for k in range(0,self.N):
            """
            In slicing, if 'k' is used in first '[]' (slicing index) 
            then it traverses row and if it is used in second slicing 
            index dimension, then it traverses columns
            """
            if self.board[i][k]==1 or self.board[k][j]==1:
                return True
        #checking diagonals
        for k in range(0,self.N):
            for l in range(0,self.N):
                if (k+l==i+j) or (k-l==i-j): 
                    # k+l checks left to right diagonal and 
                    # k-l checks right to left diagonal
                    if self.board[k][l]==1:
                        return True
        return False

    def N_queen(self, n):
        #if n is 0, solution found
        if n==0: # n is the number of queens yet to be placed
            return True

        print('----','\n','Current State:')
        self.disp_board()

        for i in range(0,self.N):
            for j in range(0,self.N):
                '''checking if we can place a queen here or not
                queen will not be placed if the place is being attacked
                or already occupied'''
                if (not(self.is_attack(i,j))) and (self.board[i][j]!=1):
                    self.board[i][j] = 1
                    #recursion
                    #wether we can put the next queen with this arrangment or not
                    if self.N_queen(n-1)==True:
                        return True
                    self.board[i][j] = 0

        return False
    
# input for number of queens
N = int(input("Enter the number of queens: ")) 
Q= Queen(N) # constructor for object initialization
Q.N_queen(N) # calling main method for NQueens
print('Final State:')
Q.disp_board()