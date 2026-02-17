import xml.etree.ElementTree as ET
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import math

def get_coords_from_pos(shape_str, target_pos):
    """
    Interpolates (x, y) coordinates from a SUMO shape string and a position value.
    shape_str: Space-separated "x,y" coordinates (e.g., "0.0,0.0 10.0,10.0")
    target_pos: Distance in meters from the start of the shape.
    """
    if not shape_str:
        return None

    points = []
    for p in shape_str.split():
        try:
            x, y = map(float, p.split(','))
            points.append((x, y))
        except ValueError:
            continue
            
    if not points:
        return None

    current_len = 0.0
    
    # Iterate through segments of the polyline
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        segment_len = math.sqrt(dx*dx + dy*dy)
        
        if current_len + segment_len >= target_pos:
            # The target is on this segment
            remaining = target_pos - current_len
            ratio = remaining / segment_len if segment_len > 0 else 0
            
            rx = p1[0] + dx * ratio
            ry = p1[1] + dy * ratio
            return (rx, ry)
        
        current_len += segment_len
        
    # If pos is beyond the end, return the last point
    return points[-1]

def get_lane_index(lane_id):
    """Robustly extracts the lane index (e.g., 'edge_0' -> 0)."""
    try:
        return int(lane_id.split('_')[-1])
    except (ValueError, AttributeError):
        return -1

print("Step 1/3: Parsing Network Geometry (map.net.xml)...")

net_tree = ET.parse('map.net.xml')
net_root = net_tree.getroot()

lane_db = {}        # Stores length, edge_id, and shape for every lane
all_lane_coords = [] # List of lists of [x,y] for drawing the map background

for edge in net_root.findall('edge'):
    # Optional: Skip internal edges (intersections) if desired
    # if edge.get('id').startswith(':'): continue
    
    edge_id = edge.get('id')
    for lane in edge.findall('lane'):
        lane_id = lane.get('id')
        length = float(lane.get('length'))
        shape = lane.get('shape')
        
        lane_db[lane_id] = {
            'edge_id': edge_id,
            'length': length,
            'shape': shape
        }
        
        # Parse points for drawing the map background later
        if shape:
            pts = []
            for p in shape.split():
                try:
                    pts.append(list(map(float, p.split(','))))
                except ValueError:
                    pass
            if pts:
                all_lane_coords.append(pts)

print("Step 2/3: Processing Lane Changes (lannchange.xml)...")

change_tree = ET.parse('lannchange.xml')
change_root = change_tree.getroot()

records = []

for change in change_root.findall('change'):
    from_lane = change.get('from')
    to_lane = change.get('to')
    
    # Skip if the lane isn't in our map database
    if from_lane not in lane_db:
        continue

    # Extract basic info
    pos = float(change.get('pos'))
    time = float(change.get('time'))
    v_type = change.get('type')
    reason = change.get('reason')
    direction = change.get('dir')
    
    # Calculate derived metrics
    lane_data = lane_db[from_lane]
    road_length = lane_data['length']
    norm_pos = pos / road_length if road_length > 0 else 0
    
    # Get Spatial Coordinates (X, Y)
    xy = get_coords_from_pos(lane_data['shape'], pos)
    x_coord, y_coord = xy if xy else (None, None)
    
    # Get Lane Transition indices
    from_idx = get_lane_index(from_lane)
    to_idx = get_lane_index(to_lane)
    
    records.append({
        'Time': time,
        'Type': v_type,
        'Reason': reason,
        'Edge': lane_data['edge_id'],
        'Lane': from_lane,
        'Position': pos,
        'Road_Length': road_length,
        'Norm_Pos': norm_pos,
        'X': x_coord,
        'Y': y_coord,
        'From_Idx': from_idx,
        'To_Idx': to_idx,
        'Transition': f"Lane {from_idx} → {to_idx}",
        'Dir_Label': 'Left (1)' if direction == '1' else 'Right (-1)'
    })

df = pd.DataFrame(records)
print(f"Step 3/3: Generating Visualizations for {len(df)} events...")

sns.set_theme(style="whitegrid")

plt.figure(figsize=(14, 8))
# Filter top 20 busiest edges to avoid clutter
busiest_edges = df['Edge'].value_counts().head(20).index
df_filtered = df[df['Edge'].isin(busiest_edges)]

sns.stripplot(
    data=df_filtered, 
    x='Edge', y='Position', hue='Type', 
    dodge=True, alpha=0.6, jitter=True, palette='viridis'
)
plt.title('1. Lane Changes by Road Segment (Top 20 Edges)', fontsize=14)
plt.xticks(rotation=45)
plt.ylabel('Position (meters)')
plt.legend(title='Vehicle Type')
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
sns.kdeplot(
    data=df, x='Norm_Pos', hue='Type', 
    fill=True, common_norm=False, palette='coolwarm', alpha=0.4
)
plt.title('2. Normalized Merging Behavior (0=Start, 1=End)', fontsize=14)
plt.xlabel('Relative Position on Edge')
plt.xlim(0, 1)
plt.show()

plt.figure(figsize=(12, 12))
ax = plt.gca()

line_segments = LineCollection(all_lane_coords, colors='lightgray', linewidths=1.5, alpha=0.5)
ax.add_collection(line_segments)

df_auto = df[df['Type'].str.contains('Autonomous', case=False, na=False)]
df_def = df[~df['Type'].str.contains('Autonomous', case=False, na=False)]

if not df_def.empty:
    plt.scatter(df_def['X'], df_def['Y'], c='blue', s=10, alpha=0.6, label='Default')
if not df_auto.empty:
    plt.scatter(df_auto['X'], df_auto['Y'], c='red', s=10, alpha=0.6, label='Autonomous')

plt.title('3. Physical Map of Lane Change Hotspots', fontsize=16)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.axis('equal')
plt.grid(False)
if not df.empty and df['X'].notnull().any():
    plt.xlim(df['X'].min() - 50, df['X'].max() + 50)
    plt.ylim(df['Y'].min() - 50, df['Y'].max() + 50)
plt.show()

plt.figure(figsize=(12, 6))
df['Time_Bin'] = (df['Time'] // 100) * 100
time_counts = df.groupby(['Time_Bin', 'Type']).size().reset_index(name='Counts')

sns.lineplot(data=time_counts, x='Time_Bin', y='Counts', hue='Type', marker='o')
plt.title('4. Lane Change Frequency Over Time', fontsize=14)
plt.xlabel('Simulation Time (s)')
plt.ylabel('Changes per 100s')
plt.show()

plt.figure(figsize=(10, 6))
transition_counts = df.groupby(['Type', 'Transition']).size().reset_index(name='Count')

total_per_type = df['Type'].value_counts()
transition_counts['Percentage'] = transition_counts.apply(
    lambda row: (row['Count'] / total_per_type[row['Type']]) * 100 if total_per_type[row['Type']] > 0 else 0, 
    axis=1
)

sns.barplot(data=transition_counts, x='Transition', y='Percentage', hue='Type', palette='muted')
plt.title('5. Overtaking vs. Returning (Lane Transitions)', fontsize=14)
plt.ylabel('Percentage of Changes')
plt.show()