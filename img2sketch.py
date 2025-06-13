# photo2sketch.py (v0.2 — smoother, merged primitives)
"""photo2sketch: Algorithmic photo→normal-map→hand‑sketch converter

Changes in **v0.2**
-------------------
1. **Primitive merging** – Nearly colinear/adjacent line segments are fused
   into single long strokes; arcs with matching centre+radius coalesce too.
2. **Stroke smoothing** – Reduced jitter on merged primitives (amplitude now
   proportional to stroke width) so long strokes look cleaner while keeping a
   subtle hand‑drawn feel.
3. **Config flags** – New CLI flags `--merge_angle`, `--merge_dist`, and
   `--jitter` to tweak behaviour.

The rest of the pipeline is unchanged: Sobel → normals, Canny → skeleton →
primitives → stylised canvas.
"""
from __future__ import annotations

import argparse
import math
import random
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import cv2
import os
import numpy as np
from PIL import Image, ImageDraw

try:
    from noise import pnoise1  # Perlin noise for stroke jitter
except ImportError:
    pnoise1 = None
import sys
sys.setrecursionlimit(20000)

###############################################################################
#                             Data structures                                 #
###############################################################################

@dataclass
class VectorPrimitive:
    kind: str  # "line" | "arc"
    pts: np.ndarray  # 2×2 for line or 1×3 for arc (cx, cy, r)
    width: float = 1.5
    gray: int = 0  # luminance (0=black)

###############################################################################
#                         Normal‑map estimation                                #
###############################################################################

def to_normal_map(gray: np.ndarray, strength: float = 1.0) -> np.ndarray:
    gray32 = gray.astype(np.float32) / 255.0
    gx = cv2.Sobel(gray32, cv2.CV_32F, 1, 0, ksize=5) * strength
    gy = cv2.Sobel(gray32, cv2.CV_32F, 0, 1, ksize=5) * strength
    normal = np.dstack((-gx, -gy, np.ones_like(gx)))
    normal /= np.linalg.norm(normal, axis=2, keepdims=True) + 1e-8
    return ((normal + 1) * 0.5 * 255).astype(np.uint8)

###############################################################################
#                          Edge detection & thinning                          #
###############################################################################

def extract_edges(gray: np.ndarray) -> np.ndarray:
    edges = cv2.Canny(gray, 50, 250)
    edges = cv2.ximgproc.thinning(edges) if hasattr(cv2, "ximgproc") else edges
    return edges

###############################################################################
#                          Line detection                                     #
###############################################################################
class Point:
    def __init__(self, x:int, y:int, v:int=0):
        self.x = x
        self.y = y
        self.value = v
        
    def __bool__(self):
        return self.value == 1
    def __int__(self):
        return self.value
    def __str__(self):
        return f"Point(x: {self.x}, y: {self.y}, v: {self.value})"
    def __tuple__(self):
        return (self.x, self.y)
    
class Line:
    def __init__(self, start:Point, end:Point):
        self.start = start
        self.end = end
        self.slope = Line.calc_slope(start, end)
        self.bias = Line.calc_bias(start, self.slope)

    
    def update_end(self, end:Point):
        self.end = end
        self.slope = Line.calc_slope(self.start, self.end)
        self.bias = Line.calc_bias(self.start, self.slope)
    def manh_dist(self):
        return Line.calc_manh_dist(self.start, self.end)
        
    @staticmethod
    def calc_manh_dist(start:Point, end:Point):
        return abs(start.x - end.x) + abs(start.y - end.y)
    @staticmethod
    def calc_euc_dist(start:Point, end:Point):
        return np.linalg.norm(np.array([start.x, start.y]) - np.array([end.x, end.y]))
    @staticmethod
    def calc_slope(start:Point, end:Point):
        if(end.x - start.x == 0):
            return 10000
        return (end.y - start.y)/(end.x - start.x)
    @staticmethod
    def calc_bias(anchor:Point, slope):
        return anchor.y - slope * anchor.x
    
    @staticmethod
    def calc_angle(slope1, slope2):
        denominator = 1 + slope1 * slope2
        if abs(denominator) < 1e-10:  # Check for near-zero denominator
            return math.pi/2  # Return 90 degrees for perpendicular lines
        return math.atan(abs((slope1 - slope2) / denominator))

    @staticmethod
    def calc_k_distance(A:Point, B:Point, C:Point, penalty=1000.0):
        """
        Return the perpendicular distance from C to the infinite line AB **iff**
        the orthogonal projection of C lies beyond B along the A→B direction
        (i.e. t > 1).  
        Otherwise return the penalty value.

        Parameters
        ----------
        A, B, C  : tuple[float, float]
            Coordinates of the three points, e.g. (x, y).
        penalty : float, default = 1000
            Value returned when the projection falls on or before the segment AB.

        Returns
        -------
        float
            Distance or the penalty.
        """
        ax, ay = A.x, A.y
        bx, by = B.x, B.y
        cx, cy = C.x, C.y

        # vector AB and AC
        ABx, ABy = bx - ax, by - ay
        ACx, ACy = cx - ax, cy - ay
        ab2 = ABx*ABx + ABy*ABy        # |AB|²

        if ab2 == 0:
            raise ValueError("A and B must be distinct to define a line")

        # projection parameter t (0 → A, 1 → B, > 1 → beyond B)
        t = (ACx*ABx + ACy*ABy) / ab2

        # Reject if projection not strictly beyond B
        if t <= 1:
            return penalty

        # Perpendicular distance: |(B−A)×(C−A)| / |B−A|
        distance = abs(ABx*ACy - ABy*ACx) / math.sqrt(ab2)
        return distance
class Matrix:
    def __init__(self, edges:np.ndarray):
        self.width = edges.shape[1]
        self.height = edges.shape[0]
        print(f"width: {self.width} height: {self.height}")
        
        self.matrix = []
        for y in range(self.height):
            self.matrix.append([])
            for x in range(self.width):
                self.matrix[y].append(Point(x,y,int(math.ceil(edges[y][x]/255))))

        
    def get_neighbs(self, point:Point, krome:Point):
        x = point.x
        y = point.y
        ids = [(x-1,y-1), (x,y-1), (x+1,y-1),
               (x-1,y),   (x+1,y),
               (x-1,y+1), (x,y+1), (x+1,y+1)]
        krome_id = (krome.x, krome.y)
        res = []
        for i in ids:
            if not (0 <= i[0] < self.width and 0 <= i[1] < self.height):
                continue
            if(self[i] and i != krome_id):
                res.append(self[i])
        return res
    
    def get_boarded_neighbs(self, point:Point):
        x = point.x
        y = point.y
        ids = [(x+1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]
        res = []
        for i in ids:
            if not (0 <= i[0] < self.width and 0 <= i[1] < self.height):
                continue
            if(self[i]):
                res.append(self[i])
        return res
    
    def __getitem__(self, key):
        x, y = key
        return self.matrix[y][x]
    
    def visualize(self, point:Point, size:int):
        for i in range(size):
            print([int(self[j + point.x - size//2, i + point.y - size//2]) for j in range(size)])
            
    def visualize_array(self, points:List[Point]):
        x_min = min([p.x for p in points])
        x_max = max([p.x for p in points])
        y_min = min([p.y for p in points])
        y_max = max([p.y for p in points])
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                point = [p for p in points if p.x == x and p.y == y]
                if(len(point) == 0):
                    print("0", end="")
                else:
                    print("1", end="")
            print()
                    
                    

  
        
def extract_lines(edges:np.ndarray, min_length = 10):
    matrix = Matrix(edges)
    lines = []
    
    print("Extracting lines ...")
    for y in range(matrix.height):
        for x in range(matrix.width):
            print(f"\rExtracting lines: {y}/{matrix.height} [{((y)/(matrix.height)*100):.1f}%]", end="")
       
            p = matrix[x,y]
            if not p:
                continue
            neighbs = matrix.get_boarded_neighbs(p)
            
            def go_by_line(current_point:Point, line_candidate:Line):
                """
                про next мы ничего не знаем, кроме value и location
    
                ---> current_point = 1 (we inside the edge)
                    ---> if next_point == 1
                            update line_candidate
                    ---> if current point don't have any other candidates - change its value to 0 to skip it later
                    
                """
                if(current_point.x == line_candidate.end.x and current_point.y == line_candidate.end.y):
                    neighbs = matrix.get_neighbs(current_point, line_candidate.start)
                else:
                    neighbs = matrix.get_neighbs(current_point, line_candidate.end)

                # matrix.visualize_array(neighbs)
                n = len(neighbs)
                next_point = None
                line_candidate.update_end(current_point)
                if(n < 2):
                    current_point.value = 0
                    if(n == 1):
                        next_point = neighbs[0]
                    if(n == 0):
                        return line_candidate
                else:
                    if line_candidate.start == current_point:
                        return line_candidate
                    slopes_diff = [Line.calc_k_distance(line_candidate.start, current_point, neighb) for neighb in neighbs]
                    min_id = np.argmin(slopes_diff)
                    if(slopes_diff[min_id] > 0.75):
                        return line_candidate
                    next_point = neighbs[min_id]
    
                return go_by_line(next_point, line_candidate)
            
            line_candidates = [go_by_line(neighb, Line(p, neighb)) for neighb in neighbs]
            for line in line_candidates:
                if(line.manh_dist() >= min_length):
                    lines.append(line)
    return lines


def clip_line_to_rect(slope, bias, W, H):
    """
    Compute the two points where y = slope*x + bias intersects
    the rectangle [0,W] x [0,H].
    Returns a tuple (p1, p2).
    """
    pts = []

    # Intersection with left edge x=0
    y0 = bias
    if 0 <= y0 <= H:
        pts.append(Point(0, y0))

    # Intersection with right edge x=W
    yW = slope * W + bias
    if 0 <= yW <= H:
        pts.append(Point(W, yW))

    # If slope is non-zero, check horizontal boundaries
    if slope != 0:
        # Intersection with bottom edge y=0 → x = -bias/slope
        x0 = -bias / slope
        if 0 <= x0 <= W:
            pts.append(Point(x0, 0))

        # Intersection with top edge y=H → x = (H - bias)/slope
        xH = (H - bias) / slope
        if 0 <= xH <= W:
            pts.append(Point(xH, H))

    if len(pts) < 2:
        raise ValueError("Line does not cross the rectangle.")

    # In case you got more than two (e.g. exactly corner hits) pick any two distinct
    # Use the left-most as start and right-most as end, say:
    pts = sorted(set(pts), key=lambda p: p.x)
    return pts[0], pts[-1]
    

def add_support_lines(lines:List[Line],shape, count = 30, x_shift_range = (0.2, 0.5)):
    lines.sort(key=lambda x: x.manh_dist(), reverse=True)
    min_dist = 10
    create_supports = []
    for i in range(count):
        if i > 3:
            # Check distance between current line and previous lines
            too_close = False
            for j in range(i):
                if (abs(lines[i].start.x - lines[j].start.x) < min_dist and 
                    abs(lines[i].start.y - lines[j].start.y) < min_dist) or \
                   (abs(lines[i].end.x - lines[j].end.x) < min_dist and 
                    abs(lines[i].end.y - lines[j].end.y) < min_dist):
                    too_close = True
                    break
            if too_close:
                continue
        # given - line (Slope, Bias, start, end). Needs to create new line, which has same slope and bias, but start and end points locates far
        start = lines[i].start if(lines[i].start.x < lines[i].end.x) else lines[i].end
        end = lines[i].start if(lines[i].start.x >= lines[i].end.x) else lines[i].end
        shift = random.randint(int(x_shift_range[0]*min(shape[0], shape[1])),int(x_shift_range[1]*min(shape[0], shape[1])))
        if(shift > abs(shift*lines[i].slope)):
            start = Point(start.x - shift, start.y - shift*lines[i].slope, 1)
            end = Point(end.x + shift, end.y + shift * lines[i].slope, 1)
        else:
            start = Point(start.x - shift/lines[i].slope, start.y - shift, 1)
            end = Point(end.x + shift/lines[i].slope, end.y + shift, 1)
        line = Line(start, end)
        start_c, end_c = clip_line_to_rect(line.slope, line.bias, shape[1], shape[0])
        if not (0 <= start.x <= shape[1] and 0 <= start.y <= shape[0]):
            start = start_c
        if not (0 <= end.x   <= shape[1] and 0 <= end.y   <= shape[0]):
            end = end_c
        create_supports.append(Line(start, end))
    
    return create_supports
        
            
    
    
def render_lines(
    screen_size: tuple[int, int],
    lines: list[Line],
    line_color: tuple[int, int, int] = (0, 0, 0),
    background_color: tuple[int, int, int] = (250, 250, 235),
    line_thickness: int = 6,
    support_line_thickness: int = 1,
    out_path: str | None = None,
):
    """Render the given lines to an image and optionally save it."""
    img = Image.new("RGB", (screen_size[1], screen_size[0]), background_color)
    draw = ImageDraw.Draw(img)

    for ln in lines:
        draw.line(
            (ln.start.x, ln.start.y, ln.end.x, ln.end.y),
            fill=line_color,
            width=line_thickness,
        )
    support_lines = add_support_lines(lines, screen_size)
    for ln in support_lines:
        draw.line(
            (ln.start.x, ln.start.y, ln.end.x, ln.end.y),
            fill=line_color,
            width=support_line_thickness,
        )

    if out_path:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        img.save(out_path)

    return img

###############################################################################
#                                  CLI                                        #
###############################################################################

def process(path: Path, out_dir: Path, strength: float, merge_angle: float, merge_dist: float, jitter: float):
    print(f"Processing {path} ...")
    img_bgr = cv2.imread(str(path))
    if img_bgr is None:
        raise FileNotFoundError(path)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    print("Extracting edges ...")
    edges = extract_edges(gray)
    lines = extract_lines(edges)


    stem = path.stem
    render_lines(edges.shape, lines, out_path=str(out_dir / f"{stem}_lines.png"))
    cv2.imwrite(str(out_dir / f"{stem}_edges.png"), edges)
    # sketch_img.save(out_dir / f"{stem}_sketch.png")
    print(f"Saved results to {out_dir.resolve()}")


def _cli():
    p = argparse.ArgumentParser("Photo → hand‑sketch converter")
    # p.add_argument("image", type=Path)
    p.add_argument("--out_dir", type=Path, default=Path("out"))
    p.add_argument("--strength", type=float, default=10.0, help="Gradient scaling for normal map")
    p.add_argument("--merge_angle", type=float, default=60.0, help="Max angle (deg) between lines to merge")
    p.add_argument("--merge_dist", type=float, default=15.0, help="Max endpoint distance (px) to merge lines")
    p.add_argument("--jitter", type=float, default=0.0, help="Stroke jitter amplitude (0=none)")
    args = p.parse_args()
    # process(args.image, args.out_dir, args.strength, args.merge_angle, args.merge_dist, args.jitter)
    process(Path("photo2.png"), args.out_dir, args.strength, args.merge_angle, args.merge_dist, args.jitter)


if __name__ == "__main__":
    _cli()

    # lines = extract_lines(np.array([
    #                             [0,0,1,0,0],
    #                             [0,1,1,1,0],
    #                             [0,1,0,0,0],
    #                             [1,0,0,0,0]]))
    # render_lines((5,4), lines, out_path=str("out/test_lines.png"))

    
    
    