import scipy.io
import numpy as np
INF_MAX = 1024
adj_mat = scipy.io.loadmat('adjacencyMatrix.mat')

def BellmanFordAlgo(adjMat, src):
    graph = {}
    for i in range(len(adjMat)):
        for j in range (len(adjMat[1])):
            if(adjMat[i][j] != 0):
                graph[i,j] = adjMat[i][j]
    # print(graph)
    
    nodes = []
    for key in graph:
        for ele in key:
            nodes.append(ele)

    nodes = list(set(nodes))

    prev_node = []
    distance = []
    visited = []

    for i in range(len(nodes)):
        distance.append(INF_MAX)
        prev_node.append('null')
        visited.append(-1)

    distance[src] = 0
    flag = True
    while(flag):
        flag = False
        for edge in graph:
            if(distance[edge[1]] > distance[edge[0]] + graph[edge]):
                distance[edge[1]] = distance[edge[0]] + graph[edge]
                prev_node[edge[1]] = edge[0]
                flag = True
    output = []
    for i in range(len(nodes)):
        output.append([nodes[i], distance[i], prev_node[i]])
 
    return output

# print(BellmanFordAlgo(adj_mat['A'],0))
# print(BellmanFordAlgo(adj_mat['A'],1))
# print("BellmanFord Output ---->>>>", BellmanFordAlgo(adj_mat['A'],2))
# print(BellmanFordAlgo(adj_mat['A'],3))
# print(BellmanFordAlgo(adj_mat['A'],4))
# print(BellmanFordAlgo(adj_mat['A'],5))


############ PART B ###############

def globalEfficiency(adjMat):

    sum_of_paths = 0.0
    for i in range(len(adjMat)):
        shortest_path = BellmanFordAlgo(adjMat,i)
        for path in shortest_path:
            if(path[0] != i  and path[1] != INF_MAX):
                sum_of_paths += 1/path[1]

    harmonic_mean = (len(adjMat) * (len(adjMat)-1)) * ( 1 / sum_of_paths)
    return (1/harmonic_mean)


# print("Global Efficiency  ---->>>>", globalEfficiency(adj_mat['A']))


def diffisionEfficiency(adjMat):
    #First step defining the U matrix --> U = W x (1/S)
    graph = {}
    S = np.zeros((len(adjMat),len(adjMat[1])))
    # for i in range(len(adjMat)):
    #     for j in range (len(adjMat[1])):
    #         if(adjMat[i][j] != 0):
    #             graph[i,j] = adjMat[i][j]
    #             S[i][i] += adjMat[i][j]
    for i in range(len(adjMat)):
        S[i][i] = np.sum(adjMat[i,:])
    U = np.dot(adjMat, np.linalg.inv(S))
    sum_of_Xij = 0
    for i in range(len(adjMat)):
        for j in range(len(adjMat[1])):
            if(i != j):
                Uj = U.copy()
                Uj[j,:] = 0
                one = np.eye(len(adjMat))
                sub = one - Uj
                try:
                    inv_sub = np.linalg.inv(sub)
                except:
                    inv_sub = np.full((len(adjMat), len(adjMat)), np.inf)
                
                Xij = np.sum(inv_sub[:,i])

                if(Xij!=np.inf and Xij!=0):
                    sum_of_Xij += 1/Xij
    Ediff = (1 / (len(adjMat)*(len(adjMat)-1))) * sum_of_Xij

    return Ediff

print(diffisionEfficiency(adj_mat['A']))


brain_mat_1 = scipy.io.loadmat('adjacencyMatrix.mat')