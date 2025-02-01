import pygame
import math

class Node():
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    " Return list of tuples as a path from given start to the end"
    print("Starting A* Search")

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    start_node.g = start_node.h = start_node.f = 0

    open_list = []
    closed_list = []
    visit_count = {}
    penalty = {}
    threshold = 2
    penalty_weight = 1000
    open_list.append(start_node)

    while open_list:

        # take minimum f node in the open list

        q = min(open_list, key=lambda obj: obj.f)
        open_list.remove(q)
        if q.position in visit_count:
            visit_count[q.position] += 1
        else:
            visit_count[q.position] = 1

        if visit_count[q.position] > threshold:
            penalty[q.position] = (visit_count[q.position] - threshold) * penalty_weight
        else:
            penalty[q.position] = 0

        if q == end_node:
            path = []
            current = q
            while current is not None:
                path.append(current.position)
                if current.parent is not None:
                    pygame.draw.line(screen, "blue", current.position, current.parent.position, 2)
                    pygame.display.update()
                current = current.parent
            return path[::-1]  # return reversed path

        # find successors of that node
        successors = []
        for successor in [(0, -5), (0, 5), (-5, 0), (5, 0), (-5, -5), (-5, 5), (5, -5), (5, 5)]:
            node_pos = (q.position[0] + successor[0], q.position[1] + successor[1])
            # if it is defined as a wall
            if maze[node_pos[0]][node_pos[1]] != 0:
                continue

            # if it is out of the map
            if node_pos[0] > len(maze) - 1 or node_pos[0] < 0 or node_pos[1] > len(maze) - 1 or node_pos[1] < 0:
                continue



            new_node = Node(q, node_pos)
            successors.append(new_node)

        for successor in successors:

            if successor in closed_list:
                continue
            # create the f, g and h values
            if abs(successor.position[0] - q.position[0]) + abs(successor.position[1] - q.position[1]) == 1:
                successor.g = 1 + q.g  # 1 = distance  between successor and q
            else:
                successor.g = 1*math.sqrt(2) + q.g
            # pythagore for eucledian heuristic
            #successor.h = (successor.position[0] - end_node.position[0] )** 2 + (
                        #(successor.position[1] - end_node.position[1]) )** 2
            penalty_value = penalty.get(successor.position, 0)
            #wall_distance = distance_to_wall(successor, maze)
            successor.h = abs(successor.position[0] - end_node.position[0]) + abs(successor.position[1] - end_node.position[1])
            successor.f = successor.g + 1.5 * successor.h + 0.001 * abs(successor.position[0] - start[0]) + abs(successor.position[1] - start[1]) + penalty_value * penalty_weight

            #for node in open_list:
                #if successor == node and successor.f >= node.f:
                    #continue

            pygame.draw.line(screen, "red", q.position, successor.position, 2)
            pygame.display.update()
            open_list.append(successor)

        closed_list.append(q)


def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

pygame.init()
screen = pygame.display.set_mode((500, 700))
clock = pygame.time.Clock()
running = True
drawing = False
color = (0, 0, 0)
screen.fill("grey")

# Button dimensions and color
button_width = 200
button_height = 50
button_color = (255, 255, 255)  # White
button_rect = pygame.Rect(150, 550, 150, 50)
start = pygame.Rect(0, 0, 10, 10)
end = pygame.Rect(420, 350, 10, 10)
font = pygame.font.Font(None, 36)
text = font.render("Search path", True, "black")
text_rect = text.get_rect(center=(220, 570))  # Center of the screen
control_rect = pygame.Rect(0, 500, 500, 200)

pygame.draw.rect(screen, "purple", control_rect)
pygame.draw.rect(screen, "red", start)
pygame.draw.rect(screen, "green", end)
pygame.draw.rect(screen, button_color, button_rect)
screen.blit(text, text_rect)
pygame.display.set_caption("A* Search")

# create empty map
maze = [[0 for _ in range(500)] for _ in range(500)]
start_pos = (0, 0)
end_pos = (420, 350)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if button_rect.collidepoint(event.pos):
                path = astar(maze, start_pos, end_pos)
                continue
            if not control_rect.collidepoint(pygame.mouse.get_pos()):
                drawing = True
            else:
                drawing = False
            last_pos = pygame.mouse.get_pos()
        elif event.type == pygame.MOUSEBUTTONUP:
            drawing = False
        elif event.type == pygame.MOUSEMOTION:
            if drawing and not control_rect.collidepoint(pygame.mouse.get_pos()):
                line_points = bresenham(last_pos[0], last_pos[1], pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1])
                for point in line_points:
                    for dx in [-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,1,0,1,2,3,4,5,6,7,8,9,10]:
                        for dy in [-5,-4,-3,-2,-1,1,0,1,2,3,4,5]:
                            nx, ny = point[0] + dx, point[1] + dy
                            if 0 <= nx < len(maze[0]) and 0 <= ny < len(maze):
                                maze[nx][ny] = 1
                pygame.draw.line(screen, color, last_pos, pygame.mouse.get_pos(), 6)
                maze[last_pos[0]][last_pos[1]] = 1
                last_pos = pygame.mouse.get_pos()

    pygame.display.update()

pygame.quit()