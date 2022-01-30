import scipy.io

INF_MAX = 1024

adj_mat = scipy.io.loadmat('adjacencyMatrix.mat')
print(adj_mat['A'])

def globalEfficiency(adjMat):
    return
 

print(globalEfficiency(adj_mat['A']))
