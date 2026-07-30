import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import numpy as np
import pickle
import os
from path_planning import config

def create_custom_env(env_number: str):
    """
    Click to place obstacle vertices. 
    Right click to close current polygon.
    Press 'q' to quit and save.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(config.ENV_X_BOUNDS)
    ax.set_ylim(config.ENV_Y_BOUNDS)
    ax.set_title("Left click: add vertex | Right click: close polygon | 'q': save and quit")
    ax.grid(True)
    
    # draw start and goal
    ax.plot(*config.START, 'go', markersize=10, label='START')
    ax.plot(*config.GOAL, 'bo', markersize=10, label='GOAL')
    ax.legend()

    obstacles = []
    current_vertices = []
    current_dots = []
    current_lines = []

    def onclick(event):
        if event.inaxes != ax:
            return
        
        if event.button == 1:  # left click - add vertex
            current_vertices.append((int(round(event.xdata)), int(round(event.ydata))))
            dot = ax.plot(event.xdata, event.ydata, 'r.', markersize=8)[0]
            current_dots.append(dot)
            if len(current_vertices) > 1:
                line = ax.plot([current_vertices[-2][0], current_vertices[-1][0]],
                               [current_vertices[-2][1], current_vertices[-1][1]], 'r-')[0]
                current_lines.append(line)
            fig.canvas.draw()

        elif event.button == 3:  # right click - close polygon
            if len(current_vertices) >= 3:
                obstacles.append(current_vertices.copy())
                poly = Polygon(np.array(current_vertices), facecolor='gray', 
                               edgecolor='black', alpha=0.5)
                ax.add_patch(poly)
                print(f"Obstacle {len(obstacles)} saved: {current_vertices}")
                current_vertices.clear()
                current_dots.clear()
                current_lines.clear()
                fig.canvas.draw()
            else:
                print("Need at least 3 vertices")

    def onkey(event):
        if event.key == 'q':
            plt.close()

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', onkey)
    plt.show()

    # save
    if obstacles:
        base_dir = os.path.abspath(os.path.dirname(__file__))
        env_dir = os.path.join(base_dir, '..', 'environments')
        os.makedirs(env_dir, exist_ok=True)
        filepath = os.path.join(env_dir, f'environment_polygon_{env_number}.pickle')
        with open(filepath, 'wb') as f:
            pickle.dump(obstacles, f)
        print(f"Saved {len(obstacles)} obstacles to {filepath}")
    else:
        print("No obstacles saved")

if __name__ == "__main__":
    env_number = input("Enter environment number (e.g. '1b'): ")
    create_custom_env(env_number)