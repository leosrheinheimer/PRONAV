import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

def plot_trajectories(target, pursuer, burn_time=5.2):
    # Extract positions from vehicle telemetry history
    target_path = np.array([state['pos'] for state in target.history])
    pursuer_path = np.array([state['pos'] for state in pursuer.history])
    
    # Calculate scalar speed magnitudes from velocity history
    pursuer_speeds = np.array([np.linalg.norm(state['v']) for state in pursuer.history])

    # Find telemetry state closest to motor burnout (5.2s)
    burnout_state = next((state for state in pursuer.history if state['timestamp'] >= burn_time), pursuer.history[-1])
    burnout_point = burnout_state['pos']

    # Calculate true CPA across the entire flight path
    distances = np.linalg.norm(target_path - pursuer_path, axis=1)
    cpa_idx = np.argmin(distances)
    min_distance = distances[cpa_idx]
    cpa_point = pursuer_path[cpa_idx]

    fig, ax = plt.subplots(figsize=(10, 6))
    
    # 1. Plot Target path
    ax.plot(target_path[:, 0], target_path[:, 1], 'r--', label='Target Path', alpha=0.7)

    # 2. Plot Pursuer path colored by speed using LineCollection
    points = pursuer_path.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    lc = LineCollection(segments, cmap='plasma', norm=plt.Normalize(pursuer_speeds.min(), pursuer_speeds.max()))
    lc.set_array(pursuer_speeds)
    lc.set_linewidth(2.5)
    line = ax.add_collection(lc)
    
    # Add colorbar for speed scale
    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label('Pursuer Speed (m/s)')

    # 3. Add marker for Motor Burnout point (Orange)
    ax.scatter([burnout_point[0]], [burnout_point[1]], color='orange', s=110, zorder=6, edgecolors='black', label=f'Motor Burnout ({burn_time}s)')

    # 4. Plot Closest Point of Approach (CPA)
    ax.scatter([cpa_point[0]], [cpa_point[1]], color='cyan', s=110, zorder=5, edgecolors='black', label=f'Closest Approach ({min_distance:.2f}m)')
    
    # Annotate CPA distance on the graph
    ax.annotate(f'CPA: {min_distance:.2f}m', 
                (cpa_point[0], cpa_point[1]),
                textcoords="offset points", 
                xytext=(12, 12), 
                ha='left',
                fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.3", fc="cyan", alpha=0.6, ec="black"),
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0", lw=1.5))

    # Formatting
    ax.set_title(f'AIM-9M Proportional Navigation (Closest Approach: {min_distance:.2f}m)')
    ax.set_xlabel('X Position (meters)')
    ax.set_ylabel('Y Position (meters)')
    ax.grid(True)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.legend(loc='upper left')
    ax.axis('equal') 
    plt.show()