import cv2
import numpy as np
import matplotlib.pyplot as plt
import poly_point_isect
import math

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
class IntersectionPoints():
    # Lower ratio --> smaller the image
    def __init__(self, img, result_image, hor_ratio = None, ver_ratio = None):
        self.img = img
        self.hor_ratio = hor_ratio
        self.ver_ratio = ver_ratio
        self.line_image = np.copy(self.img) * 0

        self.rho = 1  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 180  # angular resolution in radians of the Hough grid
        self.threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 50  # minimum number of pixels making up a line
        self.max_line_gap = 20  # maximum gap in pixels between connectable line segments

    def get_lines(self, cond=None):
        # img_cond = self.result_image
        img_cond = np.copy(self.img)

        if cond == 'bottom':
            img_cond[:int(len(self.img)/self.ver_ratio), :] = 0
        elif cond == 'top':
            img_cond[int(len(self.img)/self.ver_ratio):, :] = 0
        elif cond == 'left':
            img_cond[:, int(len(self.img[0])/self.hor_ratio):] = 0
        elif cond == 'right':
            img_cond[:, :int(len(self.img[0])/self.hor_ratio)] = 0
        
        self.gray = cv2.cvtColor(img_cond,cv2.COLOR_BGR2GRAY)
        kernel_size = 5
        self.blur_gray = cv2.GaussianBlur(self.gray,(kernel_size, kernel_size),0)

        low_threshold = 50
        high_threshold = 150
        self.edges = cv2.Canny(self.blur_gray, low_threshold, high_threshold)

        lines = cv2.HoughLinesP(self.edges, self.rho, self.theta, self.threshold, np.array([]),
                    self.min_line_length, self.max_line_gap)
        
        points = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                points.append(((x1 + 0.0, y1 + 0.0), (x2 + 0.0, y2 + 0.0)))
                cv2.line(self.line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)

        # lines_edges = cv2.addWeighted(img_cond, 0.8, self.line_image, 1, 0)

        intersections = self.get_intersections(points)
        if cond == 'right':
            return self.get_average_image(intersections=intersections)
        elif cond == 'bottom':
            return self.get_bottom_image(intersections=intersections)

    def get_intersections(self, points):
        intersections = poly_point_isect.isect_segments(points)
        intersections = self.remove_close(intersections=intersections)

        return intersections
    
    def remove_close(self, intersections):
        for idx, inter in enumerate(intersections):
            a, b = inter
            match = 0
            for other_inter in intersections[idx:]:
                if other_inter == inter:
                    continue
                c, d = other_inter
                if abs(c-a) < 35 and abs(d-b) < 35:
                    match = 1
                    intersections[idx] = ((c+a)/2, (d+b)/2)
                    intersections.remove(other_inter)

            if match == 0:
                intersections.remove(inter)

        return intersections

    def get_average_image(self, intersections):
        for inter1 in intersections:
            for inter2 in intersections:
                a, b = inter2
                if inter1 == inter2:
                    continue
                
                for inter3 in intersections:
                    if inter3 == inter2 or inter3 == inter1:
                        continue

                    if abs(inter2[0] - (inter1[0] + inter3[0])/2) <= 35 and abs(inter2[1] - (inter1[1] + inter3[1])/2) <= 35:
                        for i in range(3):
                            for j in range(3):
                                self.result_image[int(b) + i, int(a) + j] = [0, 255, 0]
                        
                        # cv2.imwrite('line_parking.png', self.result_image)
                        return np.array([int(inter2[0]), int(inter2[1])])

        return None

    def plot_image(self, intersections):
        for inter in intersections:
            a, b = inter
            for i in range(3):
                for j in range(3):
                    self.result_image[int(b) + i, int(a) + j] = [0, 255, 0]
    
    def get_image(self):
        return self.result_image
        # cv2.imwrite('line_parking.png', self.result_image)

    def get_bottom_image(self, intersections):
        for inter in intersections:
            a, b = inter
            for i in range(3):
                for j in range(3):
                    self.result_image[int(b) + i, int(a) + j] = [0, 255, 0]
            
            # cv2.imwrite('line_parking.png', self.result_image)
            return np.array([int(b), int(a)])

        return None