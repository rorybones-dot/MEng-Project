import sys
import clr, math
clr.AddReference('ProtoGeometry')
from Autodesk.DesignScript.Geometry import Point, Line, PolyCurve, Geometry

# Inputs
srf = IN[0]
boundary = IN[1]
divX = IN[2]
divY = IN[3]
zBuffer = IN[4] if len(IN) > 4 and IN[4] else 10.0

def ensure_polycurve(b):
    if isinstance(b, list):
        return PolyCurve.ByJoinedCurves(b)
    return b

def flatten_curve_xy(curve, samples=50):

    pts = []
    try:
        for i in range(samples):
            t = float(i) / (samples - 1)
            p = curve.PointAtParameter(t)
            pts.append(Point.ByCoordinates(p.X, p.Y, 0))
    except:
        p1 = curve.StartPoint
        p2 = curve.EndPoint
        pts = [
            Point.ByCoordinates(p1.X, p1.Y, 0),
            Point.ByCoordinates(p2.X, p2.Y, 0)
        ]

    return PolyCurve.ByPoints(pts)


def point_in_poly(x, y, poly_pts):
    inside = False
    n = len(poly_pts)
    if n == 0: 
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly_pts[i]
        xj, yj = poly_pts[j]
        intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside

def nearest_index(val, arr):
    #Return index of nearest value in arr to val (arr is list of numbers)
    if not arr:
        return None
    return min(range(len(arr)), key=lambda i: abs(arr[i] - val))

# --- Main ---
if srf is None or boundary is None or divX is None or divY is None:
    OUT = ("Error: missing input(s)", None, None)
else:
    poly = ensure_polycurve(boundary)
    try:
        boundary_curves = list(poly.Curves())
    except:
        boundary_curves = [poly]

    # flatten all curves to XY
    flat_curves = [flatten_curve_xy(c) for c in boundary_curves]

    bb = poly.BoundingBox
    minX, minY, minZ = bb.MinPoint.X, bb.MinPoint.Y, bb.MinPoint.Z
    maxX, maxY, maxZ = bb.MaxPoint.X, bb.MaxPoint.Y, bb.MaxPoint.Z
    
    gridX = float(divX)   # spacing in X direction
    gridY = float(divY)   # spacing in Y direction
    
    if gridX <= 0:
        gridX = 1.0
    if gridY <= 0:
        gridY = 1.0

    # Grid spacing
    gx = float(gridX)
    gy = float(gridY)
    
    # Max extents from origin
    halfRangeX = max(abs(minX), abs(maxX))
    halfRangeY = max(abs(minY), abs(maxY))
    
    # Number of steps on each side
    nx = int(math.ceil(halfRangeX / gx))
    ny = int(math.ceil(halfRangeY / gy))
    
    # Build grid using integer indexing → no accumulated floating-point errors
    xs = [round(i * gx, 10) for i in range(-nx, nx + 1)]
    ys = [round(i * gy, 10) for i in range(-ny, ny + 1)]
    



    z_high, z_low = maxZ+abs(zBuffer), minZ-abs(zBuffer)

    # Build grid nodes 
    pts_grid = [[None for j in range(len(ys))] for i in range(len(xs))]

    # polygon sampling for inside test
    sample_pts = []
    for c in flat_curves:
        try:
            n = 200
            for t in [float(i)/(n-1) for i in range(n)]:
                p = c.PointAtParameter(t)
                sample_pts.append((p.X, p.Y))
        except:
            pass

    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            if not point_in_poly(x, y, sample_pts):
                continue
            p_up = Point.ByCoordinates(x, y, z_high)
            p_down = Point.ByCoordinates(x, y, z_low)
            vl = Line.ByStartPointEndPoint(p_up, p_down)
            try:
                inter = srf.Intersect(vl)
                if inter:
                    pts_grid[i][j] = max(inter, key=lambda pt: pt.Z)
            except:
                pass

    # Find boundary intersections per grid line 
    boundary_nodes = []   # list of tuples: (point_on_surface, 'vert' or 'horiz', index)
    # vertical gridlines intersections
    for xi, x in enumerate(xs):
        testLine = Line.ByStartPointEndPoint(Point.ByCoordinates(x, minY, 0), Point.ByCoordinates(x, maxY, 0))
        for c in flat_curves:
            try:
                ints = testLine.Intersect(c)
                if ints:
                    for p in ints:
                        p_up = Point.ByCoordinates(p.X, p.Y, z_high)
                        p_down = Point.ByCoordinates(p.X, p.Y, z_low)
                        vl = Line.ByStartPointEndPoint(p_up, p_down)
                        inter_srf = srf.Intersect(vl)
                        if inter_srf:
                            best = max(inter_srf, key=lambda pt: pt.Z)
                            boundary_nodes.append((best, 'vert', xi))
            except:
                pass

    # horizontal gridlines intersections
    for yj, y in enumerate(ys):
        testLine = Line.ByStartPointEndPoint(Point.ByCoordinates(minX, y, 0), Point.ByCoordinates(maxX, y, 0))
        for c in flat_curves:
            try:
                ints = testLine.Intersect(c)
                if ints:
                    for p in ints:
                        p_up = Point.ByCoordinates(p.X, p.Y, z_high)
                        p_down = Point.ByCoordinates(p.X, p.Y, z_low)
                        vl = Line.ByStartPointEndPoint(p_up, p_down)
                        inter_srf = srf.Intersect(vl)
                        if inter_srf:
                            best = max(inter_srf, key=lambda pt: pt.Z)
                            boundary_nodes.append((best, 'horiz', yj))
            except:
                pass

    # Lines inside grid (connect consecutive interior nodes per row/column)
    horiz_lines = []
    for j in range(len(ys)):
        # collect (x,point) pairs for this row, sorted by X
        pts = [(xs[i], pts_grid[i][j]) for i in range(len(xs)) if pts_grid[i][j] is not None]
        pts.sort(key=lambda a: a[0])
        for k in range(len(pts)-1):
            a = pts[k][1]; b = pts[k+1][1]
            try:
                horiz_lines.append(Line.ByStartPointEndPoint(a, b))
            except:
                pass

    vert_lines = []
    for i in range(len(xs)):
        pts = [(ys[j], pts_grid[i][j]) for j in range(len(ys)) if pts_grid[i][j] is not None]
        pts.sort(key=lambda a: a[0])
        for k in range(len(pts)-1):
            a = pts[k][1]; b = pts[k+1][1]
            try:
                vert_lines.append(Line.ByStartPointEndPoint(a, b))
            except:
                pass

    # Connect boundary nodes back to correct grid line (robust snapping)
    connector_lines = []
    # filter out any None entries for safety
    boundary_nodes = [(p,typ,idx) for (p,typ,idx) in boundary_nodes if p is not None]

    for bpt, typ, idx in boundary_nodes:
        try:
            if typ == 'vert':
                # find nearest column index to the boundary point's X (safer than trusting idx)
                col_idx = nearest_index(bpt.X, xs)
                if col_idx is None: 
                    continue
                # collect column points (sorted by Y)
                col_pts = [(ys[j], pts_grid[col_idx][j]) for j in range(len(ys)) if pts_grid[col_idx][j] is not None]
                if not col_pts:
                    continue
                col_pts.sort(key=lambda a: a[0])
                # choose end (closest Y) to connect to
                if bpt.Y < col_pts[0][0]:
                    end_pt = col_pts[0][1]
                elif bpt.Y > col_pts[-1][0]:
                    end_pt = col_pts[-1][1]
                else:
                    # find nearest by Y
                    nearest_j = min(range(len(col_pts)), key=lambda ii: abs(col_pts[ii][0] - bpt.Y))
                    end_pt = col_pts[nearest_j][1]
                try:
                    connector_lines.append(Line.ByStartPointEndPoint(end_pt, bpt))
                except:
                    pass

            else:  # 'horiz'
                row_idx = nearest_index(bpt.Y, ys)
                if row_idx is None:
                    continue
                row_pts = [(xs[i], pts_grid[i][row_idx]) for i in range(len(xs)) if pts_grid[i][row_idx] is not None]
                if not row_pts:
                    continue
                row_pts.sort(key=lambda a: a[0])
                if bpt.X < row_pts[0][0]:
                    end_pt = row_pts[0][1]
                elif bpt.X > row_pts[-1][0]:
                    end_pt = row_pts[-1][1]
                else:
                    nearest_i = min(range(len(row_pts)), key=lambda ii: abs(row_pts[ii][0] - bpt.X))
                    end_pt = row_pts[nearest_i][1]
                try:
                    connector_lines.append(Line.ByStartPointEndPoint(end_pt, bpt))
                except:
                    pass
        except:
            pass

    # Combine outputs 
    # Correct iteration order: i over xs, j over ys
    all_nodes = [pts_grid[i][j] for i in range(len(xs)) for j in range(len(ys)) if pts_grid[i][j] is not None]
    all_nodes += [bpt for bpt, _, _ in boundary_nodes]

    # Build perimeter connections
    perimeter_nodes = [bpt for bpt, _, _ in boundary_nodes]
    perimeter_nodes = [p for p in perimeter_nodes if p is not None]
    if len(perimeter_nodes) > 2:
        cx = sum(p.X for p in perimeter_nodes) / len(perimeter_nodes)
        cy = sum(p.Y for p in perimeter_nodes) / len(perimeter_nodes)
        perimeter_nodes = sorted(perimeter_nodes, key=lambda p: math.atan2(p.Y - cy, p.X - cx))

        perimeter_lines = []
        for i in range(len(perimeter_nodes)):
            a = perimeter_nodes[i]
            b = perimeter_nodes[(i+1) % len(perimeter_nodes)]
            try:
                perimeter_lines.append(Line.ByStartPointEndPoint(a, b))
            except:
                pass
    else:
        perimeter_lines = []

    # collect nodes with Z ≈ 0 within tolerance 
    z_tol = 1e-2
    nodes_at_z0 = [p for p in all_nodes if p is not None and abs(p.Z) <= z_tol]

    # FINAL OUTPUTS 
    OUT = (
        all_nodes,                                 # all surface-projected nodes (grid + boundary)
        horiz_lines + vert_lines + connector_lines + perimeter_lines,  # all lines including perimeter
        perimeter_nodes,                           # perimeter (boundary) nodes on the surface
        nodes_at_z0                                # nodes whose Z is within tolerance of 0
    )
