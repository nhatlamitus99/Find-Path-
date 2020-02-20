import cv2
import numpy as np

#Khởi tạo các tham số
m = 0  #col
n = 0  #row
start = (0,0) 
goal = (0,0)		
list_diemdon=[]  				# Danh sách các điểm đón
list_chuongngaivat = []			# Danh sách các chướng ngại vật



# Đọc file
def readfile():

	list_po=[]
	list_barriers=[]
	boxSize = 27
	inFile = open("input.txt","r")
	n, m = list(map(int, inFile.readline().split(',')))
	line = list(map(int, inFile.readline().split(',')))
	Sx=line[0]
	Sy=line[1]
	Gx=line[2]
	Gy=line[3]
	Get=line[4:]
	start=(Sx, Sy)
	goal=(Gx, Gy)
	Pi = line[4:]
	nPo = list(map(int, inFile.readline().split()))[0]
	Po = [[]] * nPo
	res = []
	for i in range(nPo):
		Po[i] = list(map(int, inFile.readline().split(',')))
		j=0
		while j<len(Po[i]):
		
			x=Po[i][j]
			y=Po[i][j+1]
			j=j+2
			list_barriers.append((x,y))
		res.append(list_barriers)
		list_barriers=[]
	nGet=0
	while nGet<len(Get):
		x=Get[nGet]
		y=Get[nGet+1]
		nGet=nGet+2				
		list_diemdon.append((x,y))
			
	return res, m, n, start, goal

list_cnv, m, n, start, goal = readfile()


# Thuật toán A*
def astar(maze, start, end):
       
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
   
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    
    while len(open_list) > 0:

        
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        
        open_list.pop(current_index)
        closed_list.append(current_node)
       
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] 

        
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (1, -1), (1, 1), (-1, 1), (-1, -1)]: 
            
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue
            
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            
            new_node = Node(current_node, node_position)
            
            children.append(new_node)
        
        for child in children:

           
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            
            open_list.append(child)

def box(img,p,color):

    cv2.rectangle(img,(21*p[0]+1,21*p[1]+1),(21*p[0]+20,21*p[1]+20),color,-1)

def box2(img,p,color,m):

    box(img,(p[0],m-p[1]),color)
    
def drawBackground(m,n,img):

    for i in range(n+1):
        cv2.line(img,(21*i+21,0),(21*i+21,(m+2)*21),(150,150,150),1)
        box(img,(i,0),(100,100,100))
        box(img,(i,m),(100,100,100))
    for i in range(m+1):

        cv2.line(img,(0,21*i+21),(21*(n+2),21*i+21),(150,150,150),1)
        box(img,(0,i),(100,100,100))
        box(img,(n,i),(100,100,100))

class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class AStarGraph(object):
 	
	def __init__(self, list_chuongngaivat):
		self.barriers = list_chuongngaivat
 
 	# Hàm ước lượng Heristic
	def heuristic(self, start, goal):  					
		
		dx = abs(start[0] - goal[0])
		dy = abs(start[1] - goal[1])
		return dx**2+dy**2
 
	def get_vertex_neighbours(self, pos, size_x, size_y):
		n = []
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			if x2 < 0 or x2 > size_x or y2 < 0 or y2 > size_y:
				continue
			n.append((x2, y2))
		return n
 
	def move_cost(self, a, b):
		for barrier in self.barriers:
			if b in barrier:
				return 1000

		return 1 
 

 # Thuật toán A*
def AStarSearch(start, end, graph):  			
 
	G = {} 
	F = {} 

	G[start] = 0 
	F[start] = graph.heuristic(start, end)
 
	closedVertices = set()
	openVertices = set([start])
	cameFrom = {}
 
	while len(openVertices) > 0:

		current = None
		currentFscore = None
		for pos in openVertices:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos

		if current == end:

			path = [current]
			while current in cameFrom:
				current = cameFrom[current]
				path.append(current)
			path.reverse()
			return path, F[end], end
 

		openVertices.remove(current)
		closedVertices.add(current)
 

		for neighbour in graph.get_vertex_neighbours(current, m, n):
			if neighbour in closedVertices: 
				continue 
			candidateG = G[current] + graph.move_cost(current, neighbour)
 
			if neighbour not in openVertices:
				openVertices.add(neighbour) 
			elif candidateG >= G[neighbour]:
				continue 
 

			cameFrom[neighbour] = current
			G[neighbour] = candidateG
			H = graph.heuristic(neighbour, end)
			F[neighbour] = G[neighbour] + H
 
	raise RuntimeError("Can not find a path !!!")

def create_barriers(list_barriers):

	graph=AStarGraph(list_chuongngaivat)

	for i in range(len(list_barriers)):
		for j in range(len(list_barriers[i])):
			if (j<len(list_barriers[i])):
				temp1, temp2, temp3 = AStarSearch(list_barriers[i][j-1], list_barriers[i][j], graph)
				list_chuongngaivat.extend(temp1)		

create_barriers(list_cnv)

# Tìm thứ tự ưu tiên đi qua các điểm đón nào trước
def FindPriority(start, end, list, graph):   	

	j=0
	index=0
	dis=0
	closed_path=[end]
	location=0
	sum=0
	while len(list)!=1:

		i=0
		min1 = 100
		pos=0
		while i<len(list):	

			path, dis, endpos=AStarSearch(closed_path[j], list[i], graph)

			if dis<min1:
				Des=endpos
				pos=i
				min1=dis
			i=i+1
			
		list.pop(pos)
		closed_path.append(Des)
		j=j+1
		sum=sum+min1

	closed_path.append(list[0])
	closed_path.append(start)

	return closed_path, sum

maze = np.arange(m*n).reshape(m, n)

for i in range(m):
	for j in range(n):
		maze[i][j]=0

for k in list_chuongngaivat:
	maze[k[0]-1][k[1]-1]=1



# Tìm đường đi từ Start đến Goal đi qua các điểm đón
def FindPath(start, end, list, maze):				
  						
	graph = AStarGraph(list_chuongngaivat)
	dis=0
	array, dis = FindPriority(start, end, list, graph)
	res=[]
	for index in range(len(array)):

		if index!=0:

			a=array[index-1]
			b=array[index]
			temp1 = astar(maze, a, b)
			temp1.pop(len(temp1)-1)
			res.extend(temp1)
			for k in temp1:
				maze[k[0]-1][k[1]-1]=1
			

	res.append(start)
	return res, dis
	raise RuntimeError("Can not find a path !!!")
	


list_point=[]
for i in list_diemdon:		
	list_point.append(i)




res, dis=FindPath(start, goal, list_diemdon, maze)

list_res=[]

for i in list_chuongngaivat: 

	j=list(i)
	j[0]=j[0]-1
	j[1]=j[1]-1
	i=tuple(j)
	list_res.append(i)

for k in list_chuongngaivat:
	maze[k[0]-1][k[1]-1]=1


def count(res):

	count=0
	i=0
	while i<len(res)-1:

		if(abs(res[i][0]-res[i+1][0])==1 and abs(res[i][1]-res[i+1][1])==1):
			count=count+1.5
		else:
			count=count+1
		i=i+1
	return count



f = open("output.txt",'w')
f.write("Price: ")
f.write(str(count(res)))
f.write("\nPath:\n")
res.reverse()

for i in res:
	f.write("(")
	f.write(str(i[0]))
	f.write(",")
	f.write(str(i[1]))
	f.write(") ")
f.close()


img=np.zeros(((m+1)*21,(n+1)*21,3), np.uint8)
drawBackground(m,n,img)
dup=[]
for i in  list_point:
	dup.append(i)


for i in list_res:
	box2(img,i,(0,255,0),m)
	    



for i in res:
	box2(img,i,(0,0,255),m)
for i in dup:
	box2(img,i,(255,0,0),m)

cv2.imshow("Graph Path from Start to Goal passed points ",img)
cv2.waitKey(0)
cv2.destroyAllWindows()

