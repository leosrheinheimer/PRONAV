import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

def plot_trajectories(target, pursuer, burn_time=5.2, intercept_threshold=11.0):
    target_path = np.array([state['pos'] for state in target.history])
    pursuer_path = np.array([state['pos'] for state in pursuer.history])
    
    target_times = np.array([state['timestamp'] for state in target.history])
    pursuer_times = np.array([state['timestamp'] for state in pursuer.history])

    g_0 = 9.81
    target_accels = np.array([np.linalg.norm(state['a']) / g_0 for state in target.history])
    pursuer_accels = np.array([np.linalg.norm(state['a']) / g_0 for state in pursuer.history])

    burnout_state = next((state for state in pursuer.history if state['timestamp'] >= burn_time), pursuer.history[-1])
    burnout_point = burnout_state['pos']

    distances = np.linalg.norm(target_path - pursuer_path, axis=1)
    cpa_idx = np.argmin(distances)
    min_distance = distances[cpa_idx]
    cpa_point = pursuer_path[cpa_idx]

    intercepted = min_distance <= intercept_threshold

    fig, ax = plt.subplots(figsize=(11, 7))
    
    all_accels = np.concatenate([target_accels, pursuer_accels])
    norm = plt.Normalize(vmin=all_accels.min(), vmax=all_accels.max())
    cmap = 'plasma'

    def plot_colored_path(path, accels, linestyle='-'):
        points = path.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=cmap, norm=norm)
        lc.set_array(accels)
        lc.set_linewidth(2.5)
        lc.set_linestyle(linestyle)
        line = ax.add_collection(lc)
        return line

    plot_colored_path(target_path, target_accels, linestyle='--')
    line_p = plot_colored_path(pursuer_path, pursuer_accels, linestyle='-')

    cbar = fig.colorbar(line_p, ax=ax)
    cbar.set_label('Instantaneous Acceleration (G)')

    max_time = int(np.floor(min(target_times[-1], pursuer_times[-1])))
    target_sec_indices = [np.argmin(np.abs(target_times - t)) for t in range(1, max_time + 1)]
    pursuer_sec_indices = [np.argmin(np.abs(pursuer_times - t)) for t in range(1, max_time + 1)]
    
    ax.scatter(target_path[target_sec_indices, 0], target_path[target_sec_indices, 1], color='red', s=20, zorder=4, edgecolors='black', linewidths=0.5, label='Target (1s Ticks)')
    ax.scatter(pursuer_path[pursuer_sec_indices, 0], pursuer_path[pursuer_sec_indices, 1], color='black', s=20, zorder=4, edgecolors='white', linewidths=0.5, label='Pursuer (1s Ticks)')

    ax.scatter([burnout_point[0]], [burnout_point[1]], color='orange', s=110, zorder=6, edgecolors='black', label=f'Motor Burnout ({burn_time}s)')

    if intercepted:
        ax.scatter([cpa_point[0]], [cpa_point[1]], color='lime', s=130, zorder=7, edgecolors='black', label=f'Interception ({min_distance:.2f}m)')
        
        ax.annotate(f'HIT: {min_distance:.2f}m', 
                    (cpa_point[0], cpa_point[1]),
                    textcoords="offset points", 
                    xytext=(14, 14), 
                    ha='left',
                    fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", fc="lime", alpha=0.7, ec="black"),
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0", lw=1.5))
                    
        title_suffix = f' (Successful Intercept - CPA: {min_distance:.2f}m)'
    else:
        ax.scatter([cpa_point[0]], [cpa_point[1]], color='cyan', s=110, zorder=7, edgecolors='black', label=f'Closest Approach ({min_distance:.2f}m)')
        
        ax.annotate(f'CPA: {min_distance:.2f}m', 
                    (cpa_point[0], cpa_point[1]),
                    textcoords="offset points", 
                    xytext=(14, 14), 
                    ha='left',
                    fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", fc="cyan", alpha=0.7, ec="black"),
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0", lw=1.5))
        
        title_suffix = f' (Timeout / Miss - CPA: {min_distance:.2f}m)'

    ax.set_title('AIM-9M Proportional Navigation Telemetry' + title_suffix)
    ax.set_xlabel('X Position (meters)')
    ax.set_ylabel('Y Position (meters)')
    ax.grid(True, alpha=0.5)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    
    # --- LEGEND MOVED TO BOTTOM LEFT ---
    ax.legend(loc='lower left', framealpha=0.9)
    
    ax.axis('equal') 
    
    plt.savefig('trajectory_plot.png', dpi=300, bbox_inches='tight')
    plt.show()