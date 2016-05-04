import sys
# shapely is python library for geometry
from shapely.geometry import box, LineString, Point, LinearRing
# for fast kdtree
import scipy.spatial
# for fast math in python
import numpy as np
import random
# for plotting 
import matplotlib.pyplot as plt
from matplotlib import collections as mc

# half of robot base dimensions, for dilation purposes
W = .6/2
H = .4/2
# step size for algorithm, change to change topology of graph
step = .5

# obstacles dilated to create a proper c-space in R^2 (sliding box method)
obs_1 = box(2.5-1-W,
            2.5-1-H,
            2.5+1+W,
            2.5+1+H)

obs_2 = box(5.-.375-W,
            1-.375-H,
            5.+.375+W,
            1.+.375+H)

obs_3 = box(3-.375-W,
            5-.375-H,
            3+.375+W,
            5.+.375+H)

obs_4 = box(0-.25-W,
            3-.25-H,
            0+.25+W,
            3+.25+H)

# base of white contraption
obs_5 = box(.6-.2-W,
            0-.2-H,
            .6+.2+W,
            0+.2+H)

obstacles = [obs_1, obs_2, obs_3, obs_4, obs_5]

def _in_collision(q):
    for obstacle in obstacles:
        if obstacle.intersects(q) or obstacle.distance(q) <= .05:
            return True
    return False

# generate uniformly random config in dilated C-space
def qrand():
    xrand = random.uniform(-1+W, 6-W)
    yrand = random.uniform(-1+H, 6-H)
    # cartesian product of uniform rand vars is uniformly random!
    return [xrand, yrand]

# these algorithms are straight from book pseudocode
def buildRRT(q0, n):
    V = [q0]
    E = []
    T = (V, E)
    while len(V) <= n:
        extendRRT(T, qrand(), step)
    return T

def extendRRT(T, qrand, step_size):
    # find nearest neighbor in current tree
    V,E = T
    kdt = scipy.spatial.KDTree(V)
    _, idx = kdt.query(np.array(qrand))
    qnear = V[idx]
    # find config step size away from qnear in direction of qrand
    dir_vec = (qrand[0]-qnear[0], qrand[1]-qnear[1])
    vec_len = np.sqrt(dir_vec[0]**2+dir_vec[1]**2)
    # add step to qnear to get qnew
    dir_vec = (step_size*(dir_vec[0]/vec_len), step_size*(dir_vec[1]/vec_len))
    qnew = (qnear[0]+dir_vec[0], qnear[1]+dir_vec[1])
    # check for collisions
    if _in_collision(LineString([qnew, qnear])):
        return None
    # else
    V.append(qnew)
    E.append((qnear, qnew))
    return qnew

# nonstandard merge - ignore step size when connecting to T2
def mergeRRT(T1, T2, l):
    for i in range(l):
        qnew1 = extendRRT(T1, qrand(), step)
        if qnew1 is not None:
            kdt = scipy.spatial.KDTree(T2[0])
            _, idx = kdt.query(np.array(qnew1))
            closest = T2[0][idx]
            line = LineString([qnew1, closest])
            if not _in_collision(line):
                mergedTree = (list(set(T1[0]).union(set(T2[0]))),
                              list(set(T1[1]).union(set(T2[1]))))
                mergedTree[1].append((qnew1, closest))
                return mergedTree
    return None

# standard djikstra's shortest path, terminates after finding target
def shortest_path_djk(merged, source, dest):
    Q = []
    dist = {}
    prev = {}
    for v in merged[0]:
        dist[v] = np.inf
        prev[v] = None
        Q.append(v)
    dist[source] = 0
    while(len(Q) != 0):
        filtered_dist = {k:v for k,v in dist.iteritems() if k in Q}
        u = min(filtered_dist, key=filtered_dist.get)
        if u == dest:
            break
        Q.remove(u)
        # find neighbors
        incident_edges = filter(lambda x: u in x, merged[1])
        neighbors = map(lambda edge: edge[-1*edge.index(u)+1],incident_edges)
        for v in neighbors:
            alt = dist[u] + np.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
    S = []
    u = dest
    while prev[u] is not None:
        S.append(u)
        u = prev[u]
    S.append(source)
    S.reverse()
    return S

# matplotlib shyte
def plot_RRT(merged, shortest_path):
    V = merged[0]
    plt.scatter(*zip(*V))
    sp_X = map(lambda pt: pt[0], shortest_path)
    sp_Y = map(lambda pt: pt[1], shortest_path)
    plt.plot(sp_X, sp_Y, 'ro')
    lines = map(lambda x: list(x), merged[1])
    lc = mc.LineCollection(lines)
    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        fig = plt.figure(1, figsize=(5,5), dpi=90)
        ax = fig.add_subplot(111)
        ax.add_collection(lc)
        ax.grid()
        ax.plot(x, y, color='#6699cc', alpha=0.7,
            linewidth=1, solid_capstyle='round', zorder=2)
        ax.set_title('Polygon')
    plt.show()

# sample, and what gets run of you run this file
if __name__ == '__main__':
    T2 = buildRRT((5.,5.), 25)
    T1 = buildRRT((0.,0.), 25)
    merged = mergeRRT(T1, T2, 25)
    path = shortest_path_djk(merged, (0.,0.), (5.,5.))
    plot_RRT(merged, path)
