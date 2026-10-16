# THIS IS THE PATH COMPUTATION TO WHERE IT USES A* SEARCH TO DETERMINE IF THE ROBOT HAS A SAFE PATH TO TRAVERSE
import heapq

def plan_path(occupancy_grid, start, goal):
    """
    occupancy_grid:
        0 = free cell
        1 = obstacle

    start, goal = (row, column)

    Returns a list of grid coordinates, or None
    if no path exists.
    """
    rows, cols = occupancy_grid.shape

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    frontier = [(heuristic(start, goal), 0, start)]
    came_from = {start: None}
    cost = {start: 0}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)

        if current_cost != cost.get(current):
            continue

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        r, c = current

        for nr, nc in (
            (r - 1, c), (r + 1, c),
            (r, c - 1), (r, c + 1)
        ):
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            if occupancy_grid[nr, nc] != 0:
                continue

            neighbor = (nr, nc)
            new_cost = current_cost + 1

            if new_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(
                    frontier, (priority, new_cost, neighbor)
                )

    return None
