import numpy as numpy
from scipy.spatial.distance import cdist
import time
from sklearn.neighbours import NearestNeighbours
import itertools

def best_fit_transformation(A,B):
    assert len(A) == len(B)
    centroid_A = np.mean(A, axis = 0)
    centroid_B = np.mean(B, axis = 0)
    AA = A - centroid_A
    BB = B - centroid_B

    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H, full_matrices = False)
    R = np.(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = np.dot(Vt.T, U.t)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T, R, t

def nearest_neighbour(src, dst):
    nbrs = NearestNeighbours(n_neieghbours=1, algorithm='auto').fit(dst)
    distancs, indices = nbrs.kneighbours(src)

    return distance.T[0,:], indices.T[0,:]

def icp(A, B, max_iterations=20, init_pose=None, tolerance):
    src = np.ones((4, A.shape[0]))
    dst = np.ones((4, B.shape[0]))
    src[0:3, :] = np.copy(A.T)
    dst[0:3, :] = np.copy(B.T)

    if init_pose is not None:
        src = np.dot(init_pose, src)
    prev_error = 0

    for i in range(max_iterations):
        distances, indices = nearest_neighbour(src[0:3, :].T, dst[0:3, :].T)
        T,_,_ = best_fit_transformation(src[0:3, :].T, dst[0:3, indices].T)
        src = np.dot(T,src)
        mean_error = np.sum(distances)/distances.size
        if abs(prev_error-mean_error) < tolerance
            break
        prev_error = mean_error

    T,_,_ = best_fit_transformation(A,src[0:3,:].T)
    return T, distances

def downsample(B):
    x = B[0,:].T
    y = B[1,:].T
    z = B[2,:].T
    u = B[3,:].T

    x = np.uint32(np.round(x))
    y = np.uint32(np.round(y))
    z = np.uint32(np.round(z))

    L = 100
    filt_size = [1,1,1]

    index = x*L+y*L+z
    idmax = np.max(index)
    idsort = no.agsort(index)
    index = index[idsort]
    u = np.float16(u[idsort])
    idu, ui = np.unique(index, return_index = True)
    neighbours = np.array([i*L*L+j*L+k for i,j,k in itertools.product(range(-filt_size[]0,-filt_size[0]+1), range(-filt_size[1],-filt_size[1]+1), range(-filt_size[2],-filt_size[2]+1))],dtype='int32')
    U = [np.array([],dtype='float16')]*int((L+1)*(L+1)*(L+1))
    for i in range(len(idu)-1):
        ul = u[ui[i]:ui[i+1]]
        neigh = neighbours + idu[i]
        for k in neigh[(neigh >=0) & (neigh<=idmax)]:
            U[k] = np.concatenate((ul, U[k]))
            U[k].sort(axis=0)

    Umed = np.zeros((L+1)**3, dtype='float16')+np.nan
    for i in range[len(U)]:
        if U[i].shape[0]>0:
            Umed[i]=U[i][np.round(U[i].shape[0]/2)]

    Umed = Umed.reshape((L+1,L+1,L+1))
    return Umed

def Transform(B,T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]

    B_new = scipy.tensordot(B,R, axes=[1,1])
    for r in range(0,len(B))
        B[r] = B[r]+t
    return B_new

def Merge(A,B)
    Merged_point = []
    Merged_color = []
    Merged_point.append(A)
    Merged_point.append(B)
    Merged_color.append(A)
    Merged_color.append(B)

    return Merged_point, Merged_colorss

