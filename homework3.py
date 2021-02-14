from queue import Queue
from collections import defaultdict 
from math import sqrt
from queue import PriorityQueue

def isValidPos(x, y):
    return (0 <= x < H) and (0 <= y < W)

def isPathAllowed(x, y, d):
    nd = abs(searchMatrix[x][y]) if (searchMatrix[x][y] < 0) else 0
    d = abs(d) if d < 0 else 0
    return  abs(nd-d) <= maxWagonHeight 

def printPath(pathFound, path, fp):
    if pathFound:
        n = len(path)
        for i in range(n):
            x, y = path[i]
            fp.write(str(x)+','+str(y))
            if i != n-1:
                fp.write(' ')
    else:
        fp.write('FAIL')


def BFS(W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, fp):
    dirs = [
        lambda x, y: (x, y - 1),  # down
        lambda x, y: (x, y + 1),  # up
        lambda x, y: (x - 1, y),  # left
        lambda x, y: (x + 1, y,), # right
        lambda x, y: (x - 1, y - 1),  # down left
        lambda x, y: (x + 1, y - 1),  # up left
        lambda x, y: (x - 1, y + 1),  # down right
        lambda x, y: (x + 1, y + 1)  # up right
    ]
    q = Queue()
    q.put((startX, startY, searchMatrix[startX][startY], [(startY, startX)]))
    visited = set()
    visited.add((startX, startY))
    pathFound = False

    while not q.empty():
        x, y, d, path = q.get()
        if [y, x] == setlingSite:
            pathFound = True
            break
        for dir in dirs:
            nx, ny = dir(x, y)
            if isValidPos(nx, ny) and (nx, ny) not in visited and isPathAllowed(nx, ny, d):
                visited.add((nx, ny))
                q.put((nx, ny, searchMatrix[nx][ny], path + [(ny, nx)]))

    printPath(pathFound, path, fp)

def UCS(W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, fp):
    dirs = [
        lambda x, y, c: (x, y - 1, c + 10),  # down
        lambda x, y, c: (x, y + 1, c + 10),  # up
        lambda x, y, c: (x - 1, y, c + 10),  # left
        lambda x, y, c: (x + 1, y, c + 10),  # right
        lambda x, y, c: (x - 1, y - 1, c + 14),  # down left
        lambda x, y, c: (x + 1, y - 1, c + 14),  # up left
        lambda x, y, c: (x - 1, y + 1, c + 14),  # down right
        lambda x, y, c: (x + 1, y + 1, c + 14)  # up right
    ]
    visited = set()
    q = PriorityQueue()
    q.put((startX, startY, searchMatrix[startX][startY], 0, [(startY, startX)]))
    pathFound = False

    while not q.empty():
        x, y, d, c, path = q.get()
        if [y, x] == setlingSite:
            pathFound = True
            break
        visited.add((x, y))
        for dir in dirs:
            nx, ny, nc = dir(x, y, c)
            if isValidPos(nx, ny) and isPathAllowed(nx, ny, d) and (nx, ny) not in visited:
                q.put((nx, ny, searchMatrix[nx][ny], nc,  path + [(ny, nx)]))

    printPath(pathFound, path, fp)

def aStarSearch( W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, fp):
    #Diagonal Distance Formula, with D1 = 10 and D2 = 14
    # Use heuristic to estimate the shortest distance goal state from current state
    def heuristic(ix, iy):
        x, y = setlingSite
        dx = abs(x - ix)
        dy = abs(y - iy)
        heuristic = 10*(dx+dy) + (14-2*10)*min(dx, dy)
        #heuristic = max(abs(x - ix), abs(y - iy))*10
        return heuristic

    def heightDifferenceCost(h1, h2):
        h1 = abs(h1) if h1 < 0 else 0
        h2 = abs(h2) if h2 < 0 else 0
        return abs(h1-h2)

    def mudddinessCost(m):
        return m if m >= 0 else 0

    dirs = [
        lambda x, y, c: (x, y - 1, c + 10),  # down
        lambda x, y, c: (x, y + 1, c + 10),  # up
        lambda x, y, c: (x - 1, y, c + 10),  # left
        lambda x, y, c: (x + 1, y, c + 10),  # right
        lambda x, y, c: (x - 1, y - 1, c + 14),  # down left
        lambda x, y, c: (x + 1, y - 1, c + 14),  # up left
        lambda x, y, c: (x - 1, y + 1, c + 14),  # down right
        lambda x, y, c: (x + 1, y + 1, c + 14)  # up right
    ]

    visited = set()
    q = PriorityQueue()
    g = 0
    h = heuristic(startX, startY)
    f = g + h
    q.put((f, startX, startY, searchMatrix[startX][startY], g, h, [(startY, startX)]))
    pathFound = False

    while not q.empty():
        f, x, y, d, g, h, path = q.get()
        if [y, x] == setlingSite:
            pathFound = True
            break
        visited.add((x, y))
        for dir in dirs:
            nx, ny, ng = dir(x, y, g)
            if isValidPos(nx, ny) and isPathAllowed(nx, ny, d) and (nx, ny) not in visited:
                ng = ng + heightDifferenceCost(searchMatrix[x][y], searchMatrix[nx][ny]) + mudddinessCost(searchMatrix[nx][ny])
                nh = heuristic(nx, ny)
                nf = ng + nh
                q.put((nf, nx, ny, searchMatrix[nx][ny], ng, nh, path + [(ny, nx)]))
        
    printPath(pathFound, path, fp)

f = open('input.txt', 'r')
algo = f.readline().strip()
#print('Algo:',algo)
W, H = [int(x) for x in f.readline().split()]
#print('Row*Col:', W, H)
startY, startX =  (int(x) for x in f.readline().split())
#print('Starting point:',startX, startY)
maxWagonHeight = int(f.readline().rstrip('\n'))
#print('Max Wagon Height:', maxWagonHeight)
numsetlingSites = int(f.readline().rstrip('\n'))
#print('Number of setling Sites:', numsetlingSites)
setlingSites = [[int(x) for x in f.readline().split(' ')] for i in range(numsetlingSites)]
#print('Setling Sites:',setlingSites)
searchMatrix = [[int(x) for x in f.readline().split()] for i in range(H)]
#print('Search Matrix:',searchMatrix)
f.close()


f = open("output.txt", "w")
outputLines = numsetlingSites

if algo == 'BFS':
    for setlingSite in setlingSites:
        BFS(W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, f)
        outputLines -=1 
        if outputLines > 0:
            f.write('\n')
elif algo == 'UCS':
    for setlingSite in setlingSites:
        UCS(W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, f)
        outputLines -=1 
        if outputLines > 0:
            f.write('\n')
else:
    for setlingSite in setlingSites:
        aStarSearch(W, H, startX, startY, maxWagonHeight, numsetlingSites, setlingSite, searchMatrix, f)
        outputLines -=1 
        if outputLines > 0:
            f.write('\n')
f.close()