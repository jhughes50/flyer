import numpy as np
import cv2
from skimage.morphology import skeletonize
from itertools import combinations


def image2graph (src : np.ndarray) -> tuple:
   
    # 1-padding to prevent boundary nodes
    src = cv2.copyMakeBorder(src, 
                             top=1, bottom=1, 
                             left=1, right=1, 
                             borderType=cv2.BORDER_CONSTANT)
   
    # binarize the input image to avoid
    # possible issues with the provided
    # input
    src = np.where(src != 0, 1., 0.)
    src = np.int16(src)
   
    # kernel for vertex detection
    kernel = np.array([[-1, -1, -1],
                       [-1,  8, -1],
                       [-1, -1, -1]], dtype=np.int16)
    # apply the kernel via convolution
    vertices = cv2.filter2D(src, ddepth=cv2.CV_16S, 
                            kernel=kernel, anchor=(-1, -1),
                            delta=0., borderType=cv2.BORDER_DEFAULT,
                            )
   
    # threshold only the valid values
    # Note: the first condition is used to detect
    # the ramification points, while the second one
    # is used to identify the isolated vertices
    vertices = ((vertices < 6) & (vertices > 0)) | (vertices == 7)
   
    # convert it to uint8 data type and binary image
    np.choose(vertices, choices=[0, 255], out=vertices)
    vertices = np.uint8(vertices)
    #vertices = np.uint8(np.where(vertices, 255, 0))
   
    # detect the connected components of the vertices 
    # image, i.e. each connected component provides a
    # vertex.
    # Note: there could be issues to close vertices 
    # and therefore is more safety to consider the
    # centroids of each blob as putative node
    _, _, stat, nodes = cv2.connectedComponentsWithStats(vertices, 
                                                         connectivity=8)
   
    # order the connected components according to
    # their area
    idx = np.argsort(stat[:, -1])
   
    # convert the nodes to integer values, discarding
    # background component, i.e. the largest connected component
    nodes = nodes[idx[:-1]]
    np.round(nodes, 0, out=nodes)
    nodes = np.int32(nodes)
   
    # remove the nodes coordinates to the skeleton
    # according to 4-connectivity. In this way
    # the skeleton will be split into the series of
    # edges which connect them
    vy, vx = nodes[:, 0], nodes[:, 1]
    h, w, *_ = src.shape
    src[vx,                      vy  ] = 0.
    src[np.minimum(vx+1, h-1),   vy  ] = 0.
    src[vx,     np.minimum(vy+1, h-1)] = 0.
    src[np.maximum(vx-1, 0),   vy    ] = 0.
    src[vx,     np.maximum(vy-1, 0)  ] = 0.
   
    # re-convert to uint8 for connected component eval
    src = np.uint8(src)
   
    # detect the edges as connected components into
    # the splitted skeleton image
    ne, edges_lbl = cv2.connectedComponents(src, 
                                            connectivity=8)
   
    # create the edges lookup table
    # Note: discard the key 0 since it is related
    # to the background
    edges = {e : [] for e in range(1, ne)}
   
    # loop along the found nodes
    for (vxi, vyi) in zip(vx, vy):
        # extract the 5x5 roi around the node
      roi = edges_lbl[vxi - 2 : vxi + 3, vyi - 2 : vyi + 3]
      # get the set of labels found in the roi
      edge_lbl = np.unique(roi, axis=None)
      # append the list of related node
      # Note: the 0 component, i.e. background, must
      # be discarded
      for e in edge_lbl[1:]:
          # remove 1-padd from nodes coords
        edges[e].append((vyi-1, vxi-1))
   
    # remove 1-padding from nodes
    nodes -= 1
   
    # convert the lookup table to a list of tuples
    edges = sum([list(combinations(x, r=2)) 
                 for x in edges.values()], 
                [])
   
    return (nodes, edges), edges_lbl
   
def road_graph(mask):
    skeleton = skeletonize(mask) * 255
    #plt.imshow(im)
    #plt.imshow(skeleton)

    (V, E), e_lbl = image2graph(skeleton)
    return V,E
