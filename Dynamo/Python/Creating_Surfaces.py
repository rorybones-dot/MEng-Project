import sys
import clr
clr.AddReference('ProtoGeometry')
from Autodesk.DesignScript.Geometry import *


dataEnteringNode = IN


nodes_in = IN[0]
members = IN[1]

# flatten nested node lists 
def flatten(lst):
    out = []
    for item in lst:
        if isinstance(item, (list, tuple)):
            out.extend(flatten(item))
        else:
            out.append(item)
    return out

nodes = flatten(nodes_in)

def ensure_point(p):
    if hasattr(p, "X"):
        return p
    elif isinstance(p, (list, tuple)) and len(p) == 3:
        return Point.ByCoordinates(p[0], p[1], p[2])
    else:
        raise TypeError("Unsupported node type: {}".format(p))

nodes = [ensure_point(p) for p in nodes]

#  adjacency map 
adj = {i: set() for i in range(len(nodes))}

def key(pt):
    return (round(pt.X, 5), round(pt.Y, 5), round(pt.Z, 5))

node_lookup = {key(n): i for i, n in enumerate(nodes)}

if hasattr(members[0], "StartPoint"):
    for m in members:
        k1, k2 = key(m.StartPoint), key(m.EndPoint)
        if k1 in node_lookup and k2 in node_lookup:
            i1, i2 = node_lookup[k1], node_lookup[k2]
            adj[i1].add(i2)
            adj[i2].add(i1)
else:
    for pair in members:
        if len(pair) == 2:
            i1, i2 = pair
            if 0 <= i1 < len(nodes) and 0 <= i2 < len(nodes):
                adj[i1].add(i2)
                adj[i2].add(i1)

# distance function 
def dist(i, j):
    p1, p2 = nodes[i], nodes[j]
    return ((p1.X - p2.X)**2 + (p1.Y - p2.Y)**2 + (p1.Z - p2.Z)**2)**0.5

# detect triangles directly
triangles = set()
for a in adj:
    for b in adj[a]:
        if b <= a:
            continue
        for c in adj[a]:
            if c <= b:
                continue
            if c in adj[b]:
                triangles.add(tuple(sorted([a, b, c])))

# detect quads 
quads = set()
for a in adj:
    for b in adj[a]:
        for c in adj[b]:
            if c in [a, b]:
                continue
            for d in adj[c]:
                if d in [a, b, c]:
                    continue
                if (a in adj[d]) and (b in adj[c]) and not ((a in adj[c]) or (b in adj[d])):
                    quads.add(tuple(sorted([a, b, c, d])))

#  triangulate quads 
quad_tris = set()
for q in quads:
    a = list(q)[0]
    neighbors_a = [n for n in adj[a] if n in q]
    if len(neighbors_a) != 2:
        continue
    b, d = neighbors_a
    c_candidates = [n for n in adj[b] if n in q and n not in [a, d]]
    if not c_candidates:
        c_candidates = [n for n in adj[d] if n in q and n not in [a, b]]
    if not c_candidates:
        continue
    c = c_candidates[0]
    d1 = dist(a, c)
    d2 = dist(b, d)
    if d1 <= d2:
        quad_tris.add(tuple(sorted([a, b, c])))
        quad_tris.add(tuple(sorted([a, c, d])))
    else:
        quad_tris.add(tuple(sorted([a, b, d])))
        quad_tris.add(tuple(sorted([b, c, d])))

# pentagon detection
pentagons = set()

for a in range(len(nodes)):
    for b in adj[a]:
        if b <= a:  # avoid duplicates
            continue
        for c in adj[b]:
            if c in [a, b]:
                continue
            for d in adj[c]:
                if d in [a, b, c]:
                    continue
                for e in adj[d]:
                    if e in [a, b, c, d]:
                        continue
                    if a in adj[e]:
                        cycle = [a, b, c, d, e]
                        # check for diagonals
                        has_diagonal = False
                        for i in range(5):
                            ni = cycle[i]
                            for j in range(i + 2, i + 4):  # skip adjacent + self
                                nj = cycle[j % 5]
                                if nj in adj[ni]:
                                    has_diagonal = True
                                    break
                            if has_diagonal:
                                break
                        if not has_diagonal:
                            pentagons.add(tuple(sorted(cycle)))

# triangulate pentagons using centroid 
pent_tris = []

for p in pentagons:
    # order the perimeter using adjacency
    a = list(p)[0]
    neighbors_a = [n for n in adj[a] if n in p]
    if len(neighbors_a) != 2:
        continue
    b, e = neighbors_a
    ordered = [a, b]
    current = b
    prev = a
    while len(ordered) < 5:
        next_nodes = [n for n in adj[current] if n in p and n != prev]
        if not next_nodes:
            break
        next_node = next_nodes[0]
        ordered.append(next_node)
        prev = current
        current = next_node
    if len(ordered) != 5:
        continue

    # Compute centroid
    pts = [nodes[i] for i in ordered]
    cx = sum(pt.X for pt in pts) / 5.0
    cy = sum(pt.Y for pt in pts) / 5.0
    cz = sum(pt.Z for pt in pts) / 5.0
    centroid = Point.ByCoordinates(cx, cy, cz)

    # Create fan of triangles (edge + centroid)
    for i in range(5):
        a_idx = ordered[i]
        b_idx = ordered[(i + 1) % 5]
        pent_tris.append([nodes[a_idx], nodes[b_idx], centroid])

# combine all triangles 
all_tris = triangles.union(quad_tris)

# create surfaces 
surfaces = []

# triangles + quads
for tri in all_tris:
    pts = [nodes[i] for i in tri]
    try:
        srf = Surface.ByPerimeterPoints(pts)
        if srf:
            surfaces.append(srf)
    except:
        pass

# pentagons
for tri_pts in pent_tris:
    try:
        srf = Surface.ByPerimeterPoints(tri_pts)
        if srf:
            surfaces.append(srf)
    except:
        pass

OUT = surfaces

