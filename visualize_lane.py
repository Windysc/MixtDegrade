import xml.etree.ElementTree as ET
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.patches as mpatches
import math
import numpy as np

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

print("Step 2/3: Processing Lane Changes (lanechange.xml)...")

change_tree = ET.parse('lanechange.xml')
change_root = change_tree.getroot()

records = []

for change in change_root.findall('change'):
    from_lane = change.get('from')
    to_lane = change.get('to')
    
    # Skip if the lane isn't in our map database
    if from_lane not in lane_db:
        continue

    # Extract basic info
    vehicle_id = change.get('id')  # Vehicle ID
    pos = float(change.get('pos'))
    time = float(change.get('time'))
    v_type = change.get('type')
    reason = change.get('reason')
    direction = change.get('dir')
    speed = float(change.get('speed')) if change.get('speed') else None
    
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
        'Vehicle_ID': vehicle_id,
        'Time': time,
        'Type': v_type,
        'Reason': reason,
        'Edge': lane_data['edge_id'],
        'Lane': from_lane,
        'To_Lane': to_lane,
        'Position': pos,
        'Speed': speed,
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


def visualize_vehicle_activity(df, vehicle_id, lane_db, all_lane_coords):
    """Visualizes a single vehicle's lane change activity on the whole segment."""
    df_vehicle = df[df['Vehicle_ID'] == vehicle_id].copy()
    
    if df_vehicle.empty:
        print(f"No data found for vehicle: {vehicle_id}")
        return
    
    df_vehicle = df_vehicle.sort_values('Time')
    
    print(f"\nVehicle: {vehicle_id} | Type: {df_vehicle['Type'].iloc[0]}")
    print(f"Lane Changes: {len(df_vehicle)} | Time: {df_vehicle['Time'].min():.2f}s - {df_vehicle['Time'].max():.2f}s")
    print(f"Edges: {df_vehicle['Edge'].nunique()} | Reasons: {df_vehicle['Reason'].value_counts().to_dict()}")
    
    fig = plt.figure(figsize=(16, 14))
    
    ax1 = fig.add_subplot(2, 2, 1)
    line_segments = LineCollection(all_lane_coords, colors='grey', linewidths=1.0, alpha=0.5)
    ax1.add_collection(line_segments)
    
    if not df_vehicle.empty and df_vehicle['X'].notnull().any():
        times = df_vehicle['Time'].values
        norm_times = (times - times.min()) / (times.max() - times.min() + 1e-6)
        
        scatter = ax1.scatter(
            df_vehicle['X'], df_vehicle['Y'], 
            c=norm_times, cmap='plasma', s=100, 
            edgecolors='black', linewidths=0.5, zorder=5
        )
        
        coords = df_vehicle[['X', 'Y']].dropna().values
        if len(coords) > 1:
            ax1.plot(coords[:, 0], coords[:, 1], 'k--', alpha=0.5, linewidth=1, zorder=4)
        
        ax1.scatter(coords[0, 0], coords[0, 1], c='green', s=200, marker='^', 
                   edgecolors='black', linewidths=2, zorder=6, label='Start')
        ax1.scatter(coords[-1, 0], coords[-1, 1], c='red', s=200, marker='v', 
                   edgecolors='black', linewidths=2, zorder=6, label='End')
        
        cbar = plt.colorbar(scatter, ax=ax1, shrink=0.6)
        cbar.set_label('Time Progress (0=Start, 1=End)')
        
        x_margin = (df_vehicle['X'].max() - df_vehicle['X'].min()) * 0.15 + 50
        y_margin = (df_vehicle['Y'].max() - df_vehicle['Y'].min()) * 0.15 + 50
        ax1.set_xlim(df_vehicle['X'].min() - x_margin, df_vehicle['X'].max() + x_margin)
        ax1.set_ylim(df_vehicle['Y'].min() - y_margin, df_vehicle['Y'].max() + y_margin)
    
    ax1.set_title(f'Spatial Trajectory: {vehicle_id}', fontsize=12, fontweight='bold')
    ax1.set_xlabel('X Coordinate')
    ax1.set_ylabel('Y Coordinate')
    ax1.legend(loc='upper right')
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(2, 2, 2)
    times = df_vehicle['Time'].values
    from_lanes = df_vehicle['From_Idx'].values
    to_lanes = df_vehicle['To_Idx'].values
    
    for i in range(len(times)):
        if i > 0:
            ax2.hlines(from_lanes[i], times[i-1], times[i], colors='steelblue', linewidth=2)
        ax2.plot([times[i], times[i]], [from_lanes[i], to_lanes[i]], 
                 'r-', linewidth=2, alpha=0.7)
        ax2.scatter(times[i], from_lanes[i], c='blue', s=50, zorder=5)
        ax2.scatter(times[i], to_lanes[i], c='red', s=50, zorder=5)
    
    if len(times) > 0:
        ax2.hlines(to_lanes[-1], times[-1], times[-1] + 10, colors='steelblue', linewidth=2)
    
    ax2.set_title(f'Lane Index Over Time: {vehicle_id}', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Lane Index')
    ax2.set_yticks(range(max(max(from_lanes), max(to_lanes)) + 2))
    ax2.grid(True, alpha=0.3)
    
    blue_patch = mpatches.Patch(color='blue', label='From Lane')
    red_patch = mpatches.Patch(color='red', label='To Lane')
    ax2.legend(handles=[blue_patch, red_patch], loc='upper right')
    
    ax3 = fig.add_subplot(2, 2, 3)
    edges = df_vehicle['Edge'].unique()
    colors_map = plt.cm.tab10(np.linspace(0, 1, len(edges)))
    edge_color = {edge: colors_map[i] for i, edge in enumerate(edges)}
    
    for edge in edges:
        edge_data = df_vehicle[df_vehicle['Edge'] == edge]
        ax3.scatter(edge_data['Time'], edge_data['Position'], 
                   c=[edge_color[edge]], s=80, label=edge[:15] + '...' if len(edge) > 15 else edge,
                   edgecolors='black', linewidths=0.5)
    
    ax3.plot(df_vehicle['Time'], df_vehicle['Position'], 'k--', alpha=0.3, linewidth=1)
    
    ax3.set_title(f'Position on Edge Over Time: {vehicle_id}', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (meters)')
    ax3.legend(title='Edge', loc='upper left', fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    ax4 = fig.add_subplot(2, 2, 4)
    
    if df_vehicle['Speed'].notnull().any():
        reasons = df_vehicle['Reason'].unique()
        reason_colors = plt.cm.Set2(np.linspace(0, 1, len(reasons)))
        reason_color_map = {reason: reason_colors[i] for i, reason in enumerate(reasons)}
        
        for reason in reasons:
            reason_data = df_vehicle[df_vehicle['Reason'] == reason]
            ax4.scatter(reason_data['Time'], reason_data['Speed'], 
                       c=[reason_color_map[reason]], s=80, label=reason,
                       edgecolors='black', linewidths=0.5)
        
        ax4.plot(df_vehicle['Time'], df_vehicle['Speed'], 'k--', alpha=0.3, linewidth=1)
        ax4.set_ylabel('Speed (m/s)')
        ax4.legend(title='Reason', loc='upper right')
    else:
        ax4.text(0.5, 0.5, 'No speed data available', ha='center', va='center', 
                fontsize=12, transform=ax4.transAxes)
    
    ax4.set_title(f'Speed at Lane Changes: {vehicle_id}', fontsize=12, fontweight='bold')
    ax4.set_xlabel('Time (s)')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f'Vehicle Activity Analysis: {vehicle_id}', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.show()


def list_active_vehicles(df, top_n=20):
    """Lists vehicles with most lane change activity."""
    vehicle_counts = df.groupby(['Vehicle_ID', 'Type']).size().reset_index(name='Lane_Changes')
    vehicle_counts = vehicle_counts.sort_values('Lane_Changes', ascending=False).head(top_n)
    print(f"\nTop {top_n} Active Vehicles:")
    for i, row in vehicle_counts.iterrows():
        print(f"  {row['Vehicle_ID']:15} | {row['Type']:12} | {row['Lane_Changes']} changes")
    return vehicle_counts['Vehicle_ID'].tolist()


print("\n6. Single Vehicle Activity Visualization")
active_vehicles = list_active_vehicles(df, top_n=15)
print("\nEnter vehicle ID (or press Enter for most active):")
user_input = input("> ").strip()

if user_input:
    selected_vehicle = user_input
else:
    selected_vehicle = active_vehicles[0] if active_vehicles else None

if selected_vehicle:
    visualize_vehicle_activity(df, selected_vehicle, lane_db, all_lane_coords)
else:
    print("No vehicles found in the dataset.")