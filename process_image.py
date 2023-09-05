import numpy as np
import matplotlib.pyplot as plt
import cv2

image_path = r"C:\Users\Yusif\Desktop\Bug Problem.JPG"
img = cv2.imread(image_path)
image = cv2.flip(img, 0)
start_point = [75, 223]
goal_point = [682, 221]


def find_contours(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 200]
    return filtered_contours


def find_vertices(contour):
    epsilon = 0.005 * cv2.arcLength(contour, True)  # Adjust the epsilon parameter as needed
    polygon = cv2.approxPolyDP(contour, epsilon, True)
    vertices = polygon.squeeze()
    return np.vstack([vertices, vertices[0]])


if __name__ == "__main__":
    plt.figure()
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), origin='upper')
    contours_ = find_contours(image)

    # Plot the polygon by connecting its vertices
    for contour_ in contours_:
        vertices_ = find_vertices(contour_)
        polygon_x, polygon_y = zip(*vertices_)
        plt.plot(polygon_x, polygon_y, marker='o', linestyle='-', color='red')
        plt.plot([start_point[0], goal_point[0]], [start_point[1], goal_point[1]], marker='o', linestyle='--', color='blue')
    plt.tight_layout()
    plt.gca().invert_yaxis()
    plt.show()
