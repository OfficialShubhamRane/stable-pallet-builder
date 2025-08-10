import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random

PALLET_WIDTH, PALLET_DEPTH, MAX_HEIGHT = 100, 120, 200

class Box:
    def __init__(self, w, d, h, weight, name):
        self.w, self.d, self.h, self.weight, self.name = w, d, h, weight, name
        self.x = self.y = self.z = 0

boxes_to_drop = [
    Box(random.randint(10, 40), random.randint(10, 40), random.randint(10, 40),
        random.randint(5, 50), f"B{i+1}") for i in range(30)
]

placed_boxes = []

def draw_pallet():
    ax.cla()
    # Pallet base
    pallet = Poly3DCollection(
        [[[0,0,0], [PALLET_WIDTH,0,0], [PALLET_WIDTH,PALLET_DEPTH,0], [0,PALLET_DEPTH,0]]],
        color='saddlebrown', alpha=0.5
    )
    ax.add_collection3d(pallet)

    colors = plt.cm.tab20.colors
    for i, b in enumerate(placed_boxes):
        X = [b.x, b.x + b.w]
        Y = [b.y, b.y + b.d]
        Z = [b.z, b.z + b.h]
        verts = [
            # Bottom
            [[X[0], Y[0], Z[0]], [X[1], Y[0], Z[0]], [X[1], Y[1], Z[0]], [X[0], Y[1], Z[0]]],
            # Top
            [[X[0], Y[0], Z[1]], [X[1], Y[0], Z[1]], [X[1], Y[1], Z[1]], [X[0], Y[1], Z[1]]],
            # Sides
            [[X[0], Y[0], Z[0]], [X[1], Y[0], Z[0]], [X[1], Y[0], Z[1]], [X[0], Y[0], Z[1]]],
            [[X[1], Y[0], Z[0]], [X[1], Y[1], Z[0]], [X[1], Y[1], Z[1]], [X[1], Y[0], Z[1]]],
            [[X[1], Y[1], Z[0]], [X[0], Y[1], Z[0]], [X[0], Y[1], Z[1]], [X[1], Y[1], Z[1]]],
            [[X[0], Y[1], Z[0]], [X[0], Y[0], Z[0]], [X[0], Y[0], Z[1]], [X[0], Y[1], Z[1]]]
        ]
        ax.add_collection3d(Poly3DCollection(verts, facecolors=colors[i % len(colors)], alpha=0.7, edgecolors='black'))

    ax.set_xlim(0, PALLET_WIDTH)
    ax.set_ylim(0, PALLET_DEPTH)
    ax.set_zlim(0, MAX_HEIGHT)
    ax.set_xlabel("Width")
    ax.set_ylabel("Depth")
    ax.set_zlabel("Height")
    ax.view_init(elev=30, azim=45)
    plt.draw()

def boxes_overlap_xy(b1, b2):
    # Check overlap on XY plane (ignore Z)
    return not (b1.x + b1.w <= b2.x or b1.x >= b2.x + b2.w or
                b1.y + b1.d <= b2.y or b1.y >= b2.y + b2.d)

def find_highest_stack_z(x, y, w, d):
    # Find highest top surface under box footprint at position (x,y)
    max_z = 0
    for b in placed_boxes:
        # If overlap in XY with footprint, consider top Z surface
        if not (x + w <= b.x or x >= b.x + b.w or
                y + d <= b.y or y >= b.y + b.d):
            top_z = b.z + b.h
            if top_z > max_z:
                max_z = top_z
    return max_z

def find_placement(box):
    step = 5
    for x in range(0, PALLET_WIDTH - box.w + 1, step):
        for y in range(0, PALLET_DEPTH - box.d + 1, step):
            # Create a temporary box at (x,y) and z at stack height
            z = find_highest_stack_z(x, y, box.w, box.d)
            # Check no overlap with placed boxes in 3D
            overlap = False
            temp_box = Box(box.w, box.d, box.h, box.weight, box.name)
            temp_box.x, temp_box.y, temp_box.z = x, y, z
            for b in placed_boxes:
                # Check 3D overlap
                if boxes_overlap_xy(temp_box, b):
                    # Check vertical overlap
                    if not (temp_box.z + temp_box.h <= b.z or temp_box.z >= b.z + b.h):
                        overlap = True
                        break
            # Also check height limit
            if not overlap and z + box.h <= MAX_HEIGHT:
                return x, y, z
    return None, None, None

def on_click(event):
    if not boxes_to_drop:
        print("No more boxes to drop.")
        return

    box = boxes_to_drop.pop(0)
    x, y, z = find_placement(box)
    if x is None:
        print("No space left on pallet!")
        return

    box.x, box.y, box.z = x, y, z
    placed_boxes.append(box)
    draw_pallet()

fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(111, projection='3d')
draw_pallet()
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
