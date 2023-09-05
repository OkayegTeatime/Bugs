from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2

image_path = r"C:\Users\Yusif\Desktop\Bug Problem.JPG"
img = cv2.imread(image_path)
image = cv2.flip(img, 0)
start_point = [75, 223]
goal_point = [682, 221]

print(image.shape)


def find_line_equation(point1, point2):
    # Calculate the slope (m) of the line
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        a, b, c = 1, 0, x1
    else:
        m = (y2 - y1) / (x2 - x1)
        a = -m
        b = 1
        c = a * x1 + b * y1
    x_limit = [x1 + 5, x2 + 5]
    x_limit.sort()
    y_limit = [y1 + 5, y2 + 5]
    y_limit.sort()
    edge = {'coefficients': [a, b, c],
            'limit': {'x': x_limit,
                      'y': y_limit},
            'points': [point1, point2]}
    print(f'\nEquation of line segment: {a}x + {b}y = {c} \nfor x in {x_limit} and y in {y_limit}')
    return edge


def find_vertices(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 200]
    return filtered_contours


# plt.subplot(221)
# plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
# plt.title('Original Image with Contours')

# plt.subplot(221)
# plt.imshow(gray, cmap='gray')
# plt.title('Grayscale Image')
#
# plt.subplot(222)
# plt.imshow(edged, cmap='gray')
# plt.title('Edged Image')
#
# # Create a new subplot for the polygon
# plt.subplot(223)
# plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
# plt.title('Detected Polygons')

plt.figure()
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), origin='upper')
contours = find_vertices(image)

# Plot the polygon by connecting its vertices
for contour in contours:
    epsilon = 0.005 * cv2.arcLength(contour, True)  # Adjust the epsilon parameter as needed
    polygon = cv2.approxPolyDP(contour, epsilon, True)
    vertices = polygon.squeeze()
    vertices = np.vstack([vertices, vertices[0]])
    print(vertices)
    for i in range(len(vertices) - 1):
        find_line_equation(vertices[i], vertices[i + 1])
    polygon_x, polygon_y = zip(*vertices)
    plt.plot(polygon_x, polygon_y, marker='o', linestyle='-', color='red')
    plt.plot([start_point[0], goal_point[0]], [start_point[1], goal_point[1]], marker='o', linestyle='--', color='blue')

plt.tight_layout()
plt.gca().invert_yaxis()

# plt.show()

if __name__ == '__main__':
    pass
