import heapq


class Node:
    def __init__(self, x, y, cost, p):
        self.x = x
        self.y = y
        self.cost = cost
        self.p = p

    def __lt__(self, other):
        return self.cost < other.cost


class AStar:
    def __init__(self, sx, sy, gx, gy, w, h, obstacle):
        self.sx, self.sy = sx, sy
        self.gx, self.gy = gx, gy
        self.w, self.h = w, h
        self.obstacle = obstacle
        self.dx = [1, 0, -1, 0]
        self.dy = [0, 1, 0, -1]
        self.board = [[-1] * w for _ in range(h)]
        for (x, y) in obstacle:
            self.board[y][x] = -2

    def heuristic(self, x, y):
        return abs(self.gx - x) + abs(self.gy - y)

    def astar(self):
        open_list = []
        self.board[self.sy][self.sx] = 0
        heapq.heappush(open_list, Node(self.sx, self.sy, 0, None))

        while open_list:
            node = heapq.heappop(open_list)
            if node.x == self.gx and node.y == self.gy:
                path = []
                while node:
                    path.append((node.x, node.y))
                    node = node.p
                return path[::-1]

            for i in range(4):
                nx, ny = node.x + self.dx[i], node.y + self.dy[i]
                if nx < 0 or ny < 0 or nx >= self.w or ny >= self.h:
                    continue
                if self.board[ny][nx] == -2:
                    continue
                if self.board[ny][nx] == -1 or self.board[ny][nx] > node.cost + 1:
                    self.board[ny][nx] = node.cost + 1
                    heapq.heappush(open_list,
                                   Node(nx, ny, self.board[ny][nx] + self.heuristic(nx, ny), node))

        return None


if __name__ == '__main__':
    # start position
    start_x = 0
    start_y = 0
    # goal position
    goal_x = 9
    goal_y = 9
    # map size
    width = 10
    height = 10
    # obstacles list
    obstacles = [(1, 2), (3, 4), (5, 6)]
    astar = AStar(start_x, start_y, goal_x, goal_y, width, height, obstacles)
    path = astar.astar()
    print(path)
