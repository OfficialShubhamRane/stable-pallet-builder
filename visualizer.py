import plotly.graph_objects as go
import re

# Paste your arrangement string here
arrangement_text = """
B4(W:13,D:7,H:6,Wt:42) -> Pos(20,24,0)
B16(W:5,D:14,H:8,Wt:42) -> Pos(16,24,0)
B13(W:10,D:7,H:9,Wt:35) -> Pos(20,18,0)
B15(W:8,D:7,H:15,Wt:42) -> Pos(20,30,0)
B5(W:6,D:20,H:6,Wt:34) -> Pos(10,24,0)
B19(W:6,D:11,H:12,Wt:35) -> Pos(28,30,0)
B12(W:7,D:6,H:18,Wt:31) -> Pos(14,18,0)
B23(W:7,D:6,H:13,Wt:21) -> Pos(30,18,0)
B14(W:16,D:15,H:5,Wt:45) -> Pos(20,4,0)
B24(W:10,D:8,H:15,Wt:42) -> Pos(18,38,0)
B8(W:10,D:19,H:8,Wt:44) -> Pos(0,24,0)
B25(W:6,D:5,H:16,Wt:11) -> Pos(14,14,0)
B10(W:10,D:13,H:20,Wt:48) -> Pos(4,12,0)
B6(W:11,D:13,H:15,Wt:39) -> Pos(10,0,0)
B17(W:7,D:18,H:12,Wt:27) -> Pos(34,24,0)
B11(W:20,D:8,H:7,Wt:17) -> Pos(20,10,5)
B1(W:17,D:12,H:10,Wt:26) -> Pos(4,24,8)
B21(W:14,D:17,H:10,Wt:28) -> Pos(20,2,12)
B9(W:18,D:10,H:19,Wt:26) -> Pos(0,36,8)
B2(W:12,D:18,H:19,Wt:31) -> Pos(28,24,12)
B3(W:20,D:18,H:14,Wt:35) -> Pos(8,18,20)
B22(W:18,D:11,H:9,Wt:11) -> Pos(2,2,15)
B18(W:17,D:13,H:12,Wt:15) -> Pos(20,6,22)
B20(W:15,D:14,H:12,Wt:14) -> Pos(6,4,24)
B7(W:17,D:16,H:13,Wt:16) -> Pos(20,24,34)
"""

# Parse arrangement
pattern = re.compile(
    r"(B\d+)\(W:(\d+),D:(\d+),H:(\d+),Wt:(\d+)\) -> Pos\((\d+),(\d+),(\d+)\)"
)
boxes = []
for match in pattern.findall(arrangement_text):
    box_id, w, d, h, wt, x, y, z = match
    boxes.append({
        "id": box_id,
        "w": int(w),
        "d": int(d),
        "h": int(h),
        "wt": int(wt),
        "x": int(x),
        "y": int(y),
        "z": int(z)
    })

# Get weight range for coloring
min_wt = min(b["wt"] for b in boxes)
max_wt = max(b["wt"] for b in boxes)

def weight_to_color(weight):
    t = (weight - min_wt) / (max_wt - min_wt)
    if t < 0.5:
        r = int(255 * (t * 2))
        g = 255
        b = 0
    else:
        r = 255
        g = int(255 * (1 - (t - 0.5) * 2))
        b = 0
    return f"rgb({r},{g},{b})"

# Create Plotly 3D figure
fig = go.Figure()

for b in boxes:
    x0, y0, z0 = b["x"], b["y"], b["z"]
    x1, y1, z1 = x0 + b["w"], y0 + b["d"], z0 + b["h"]

    corners = [
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1]
    ]

    faces = [
        [0, 1, 2], [0, 2, 3],
        [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6],
        [3, 0, 4], [3, 4, 7],
    ]

    xs, ys, zs = zip(*corners)

    # Draw the box mesh (colored by weight)
    fig.add_trace(go.Mesh3d(
        x=xs, y=ys, z=zs,
        i=[f[0] for f in faces],
        j=[f[1] for f in faces],
        k=[f[2] for f in faces],
        opacity=0.5,
        color=weight_to_color(b["wt"]),
        hovertext=f"{b['id']}<br>W:{b['w']} D:{b['d']} H:{b['h']}<br>Wt:{b['wt']}",
        hoverinfo="text"
    ))

    # Draw black edges for the box
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom square
        (4, 5), (5, 6), (6, 7), (7, 4),  # top square
        (0, 4), (1, 5), (2, 6), (3, 7)   # verticals
    ]
    for e in edges:
        fig.add_trace(go.Scatter3d(
            x=[corners[e[0]][0], corners[e[1]][0]],
            y=[corners[e[0]][1], corners[e[1]][1]],
            z=[corners[e[0]][2], corners[e[1]][2]],
            mode="lines",
            line=dict(color="black", width=4),
            showlegend=False
        ))

fig.update_layout(
    scene=dict(
        xaxis_title="Width",
        yaxis_title="Depth",
        zaxis_title="Height"
    ),
    title="Pallet Arrangement Visualization (Weight-colored with Borders)"
)

fig.show()
