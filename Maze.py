import random
from pyamaze import maze, agent, textLabel, COLOR


class SearchingAlgorithm:
    def __init__(self, m, start, goal):
        self.__m = m
        self.__start = start
        self.__goal = goal

    def RandomWalk(self):
        visited, path, close = set(), dict(), list()

        curr = self.__start
        while curr != self.__goal:
            visited.add(curr)
            if curr not in close:
                close.append(curr)
            move, child = 'ESWN', None
            for d in random.sample(move, len(move)):
                if self.__m.maze_map[curr][d]:
                    if d == 'E':
                        child = (curr[0], curr[1] + 1)
                    elif d == 'W':
                        child = (curr[0], curr[1] - 1)
                    elif d == 'N':
                        child = (curr[0] - 1, curr[1])
                    elif d == 'S':
                        child = (curr[0] + 1, curr[1])
                    break
            if child in visited:
                curr = visited.pop()
            else:
                path[child] = curr
                curr = child
        close.append(self.__goal)
        return self.__reconstruct(path, self.__start, self.__goal), close

    # def __search(self, curr):
    #     path = list()
    #     for d in 'ESWN':
    #         child, temp = list(), curr
    #         while self.__m.maze_map[temp][d]:
    #             if d == 'E':
    #                 temp = (temp[0], temp[1] + 1)
    #             elif d == 'W':
    #                 temp = (temp[0], temp[1] - 1)
    #             elif d == 'N':
    #                 temp = (temp[0] - 1, temp[1])
    #             elif d == 'S':
    #                 temp = (temp[0] + 1, temp[1])
    #             child.append(temp)
    #         path.append(child)
    #     path = sorted(path, key=lambda x: len(x), reverse=True)
    #     return path[0]
    #
    # def LineSearch(self, curr=None, open=None, close=None, path=None):
    #     if curr is None:
    #         curr, open, close, path = self.__start, list(), list(), dict()
    #
    #     if curr not in close:
    #         close.append(curr)
    #
    #     if curr == self.__goal or self.__goal in close:
    #         return path, close
    #
    #     open.extend(self.__search(curr))
    #     return open

    def DFS(self):
        open, close = list(), list()
        open.append(self.__start)

        path = dict()
        while open:
            curr = open.pop()
            if curr not in close:
                close.append(curr)

            if curr == self.__goal:
                return self.__reconstruct(path, self.__start, self.__goal), close

            for d in 'ESWN':
                if self.__m.maze_map[curr][d]:
                    child = None
                    if d == 'E':
                        child = (curr[0], curr[1] + 1)
                    elif d == 'W':
                        child = (curr[0], curr[1] - 1)
                    elif d == 'N':
                        child = (curr[0] - 1, curr[1])
                    elif d == 'S':
                        child = (curr[0] + 1, curr[1])

                    if child in close:
                        continue
                    open.append(child)
                    path[child] = curr

    def BFS(self):
        open, close = list(), list()
        open.append(self.__start)

        path = dict()
        while open:
            curr = open.pop(0)
            if curr not in close:
                close.append(curr)

            if curr == self.__goal:
                return self.__reconstruct(path, self.__start, self.__goal), close

            for d in 'ESWN':
                if self.__m.maze_map[curr][d]:
                    child = None
                    if d == 'E':
                        child = (curr[0], curr[1] + 1)
                    elif d == 'W':
                        child = (curr[0], curr[1] - 1)
                    elif d == 'N':
                        child = (curr[0] - 1, curr[1])
                    elif d == 'S':
                        child = (curr[0] + 1, curr[1])

                    if child in close:
                        continue
                    open.append(child)
                    path[child] = curr

    def Dijkstra(self):
        open = {n: float('inf') for n in self.__m.grid}
        open[self.__start] = 0

        close, path = list(), dict()
        while open:
            curr = min(open, key=open.get)
            if curr == self.__goal:
                return self.__reconstruct(path, self.__start, self.__goal), close

            for d in 'ESWN':
                if self.__m.maze_map[curr][d]:
                    child = None
                    if d == 'E':
                        child = (curr[0], curr[1] + 1)
                    elif d == 'W':
                        child = (curr[0], curr[1] - 1)
                    elif d == 'N':
                        child = (curr[0] - 1, curr[1])
                    elif d == 'S':
                        child = (curr[0] + 1, curr[1])

                    if child in close:
                        continue
                    temp = open[curr]+1
                    if temp < open[child]:
                        open[child] = temp
                        path[child] = curr
            open.pop(curr)
            close.append(curr)

    # heuristic function
    def __h(self, curr, goal=None):
        if goal is None:
            goal = self.__goal
        x1, y1 = curr
        x2, y2 = goal
        # return abs(x1 - x2) + abs(y1 - y2)  # Manhattan Distance
        return 2*(abs(x1 - x2) + abs(y1 - y2))  # Manhattan Distance

    def AStar(self, start=None, goal=None):
        if start is None and goal is None:
            start, goal = self.__start, self.__goal

        # each node contain {current position, level, f(n)}
        g_score = 0     # level
        f_score = g_score + self.__h(self.__start)
        node = (start, g_score, f_score)

        path, open, close = dict(), list(), list()
        open.append(node)
        while open:
            curr = open.pop(0)
            if curr[0] not in close:
                close.append(curr[0])

            if curr[0] == goal:
                return self.__reconstruct(path, start, goal), close

            for d in 'ESWN':
                if self.__m.maze_map[curr[0]][d]:
                    child = None
                    if d == 'E':
                        child = (curr[0][0], curr[0][1] + 1)
                    elif d == 'W':
                        child = (curr[0][0], curr[0][1] - 1)
                    elif d == 'N':
                        child = (curr[0][0] - 1, curr[0][1])
                    elif d == 'S':
                        child = (curr[0][0] + 1, curr[0][1])

                    if child in close:
                        continue

                    temp_score = g_score + 1
                    f_score = temp_score + self.__h(child)
                    open.append((child, temp_score, f_score))
                    path[child] = curr[0]
            open.sort(key=lambda x: x[2])
            g_score += 1

    def S_AStar(self):
        # Random Points
        check_points = [(random.randint(1, self.__m.rows), random.randint(1, self.__m.cols)) for _ in range(1, self.__m.rows*self.__m.cols//10)]
        if self.__goal in check_points:
            check_points.remove(self.__goal)

        temp_goal_set = [(i, self.__h(self.__start, i)) for i in check_points]
        temp_goal_set.sort(key=lambda x: x[1])

        path = list()
        while len(temp_goal_set):
            z = temp_goal_set.pop(0)[0]
            path1, close1 = self.AStar(self.__start, z)
            path2, close2 = self.AStar(z, self.__goal)
            path1.update(path2)
            for i in close2:
                if i in close1:
                    continue
                close1.append(i)
            path.append([path1, close1])
        temp, close = self.AStar()
        path.append([temp, close])
        # path.sort(key=lambda x: len(x[1]))
        # path.sort(key=lambda x: len(x[0]))
        path.sort(key=lambda x: len(x[0]) and len(x[1]))
        return path[0][0], path[0][1]

    # reconstructing the path from the goal to the start
    @staticmethod
    def __reconstruct(path, start, goal):
        repath, cell = dict(), goal
        while cell != start:
            repath[path[cell]] = cell
            cell = path[cell]
        return repath


class MazeProblem:
    def __init__(self, length=10, breadth=10):
        self.__m = maze(length, breadth)
        # self.__m.CreateMaze()
        self.__m.CreateMaze(loopPercent=100)
        # self.__m.CreateMaze(loadMaze='./maze--2023-01-19--14-38-26.csv')

        self.__goal = (1, 1)
        self.__start = (self.__m.rows, self.__m.cols)

    def solve(self):
        path, close = None, None
        sa = SearchingAlgorithm(self.__m, self.__start, self.__goal)

        match int(input("Enter number from above algorithm respectively: ")):
            case 1:  # DFS algorithm
                path, close = sa.DFS()
                textLabel(self.__m, 'Depth-First Search (DFS)', "")
            case 2:  # BFS algorithm
                path, close = sa.BFS()
                textLabel(self.__m, 'Breadth-First Search (BFS)', "")
            case 3:  # A* algorithm
                path, close = sa.AStar()
                textLabel(self.__m, 'A* using Manhattan distance', "")
            case 4:  # Dijkstra algorithm
                path, close = sa.Dijkstra()
                textLabel(self.__m, 'Dijkstra Algorithm', "")
            case 5:  # Random Walk Algorithm
                path, close = sa.RandomWalk()
                textLabel(self.__m, 'Random Walk Algorithm', "")
            case 6:  # Stochastic A* algorithm
                path, close = sa.S_AStar()
                textLabel(self.__m, 'Stochastic A* using Manhattan distance', "")

        a = agent(self.__m, filled=True, footprints=True, color=COLOR.yellow)
        self.__m.tracePath({a: close}, delay=100)
        textLabel(self.__m, 'No. of visited cell', len(close))

        b = agent(self.__m, filled=True, footprints=True, color=COLOR.red)
        self.__m.tracePath({b: path}, delay=100)
        textLabel(self.__m, 'No. of move to reach Goal', len(path))

        self.__m.run()


def main():
    print("\n------------------------------------------------------------------------------------\n")
    print("                                    Maze Problem                                    ")
    print("\n------------------------------------------------------------------------------------\n")

    l, b = map(int, input("Enter Maze length and breadth: ").split())
    m = MazeProblem(l, b)

    print("\n------------------------------------------------------------------------------------\n")
    print("Here, we can solve Maze problem using following algorithm:")
    print("1. Depth-First Search (DFS)")
    print("2. Breadth-First Search (BFS)")
    print("3. A* Star using Manhattan distance")
    print("4. Dijkstra Algorithm")
    print("5. Random Walk Algorithm")
    print("6. Stochastic A* Star using Manhattan distance")
    print("\n------------------------------------------------------------------------------------\n")

    m.solve()


if __name__ == "__main__":
    main()
