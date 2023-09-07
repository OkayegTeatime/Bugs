import cv2
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from process_image import find_edges
from bugs import Bug, Anim

image_path = "Bug Problem.jpg"
img = cv2.imread(image_path)
image = cv2.flip(img, 0)
start_point = [75, 223]
goal_point = [682, 221]

if __name__ == '__main__':
    edges = find_edges(image)
    bug = Bug(start_point, goal_point, edges, 1, steps=4000, step_distance=1)
    # bug = Bug(start_point, goal_point, edges, 2, steps=1500, step_distance=1)
    bug.run()
    animation = Anim(bug.position_history, start_point, goal_point, edges, title=f'Bug Motion')
    anim = FuncAnimation(animation.fig, animation.update_plot, blit=False, interval=10, save_count=1000)
    plt.show()
