import math
from itertools import combinations

def euclid(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def held_karp_path(start, pts):
    """
    Exact TSP path (start -> visit all pts exactly once, no return),
    using Held-Karp DP. pts is a list of points (tuples).
    Returns best_order (list of pts in visiting order).
    Complexity: O(n^2 * 2^n). Use for n <= ~12.
    """
    n = len(pts)
    if n == 0:
        return []
    # index pts 0..n-1
    # DP[mask][i] = minimal cost to reach i with visited mask (mask includes i)
    INF = float('inf')
    size = 1 << n
    dp = [[INF]*n for _ in range(size)]
    parent = [[-1]*n for _ in range(size)]
    # base: directly from start to each i
    for i in range(n):
        m = 1 << i
        dp[m][i] = euclid(start, pts[i])
    # iterate masks
    for mask in range(size):
        for last in range(n):
            if not (mask & (1<<last)): 
                continue
            cur_cost = dp[mask][last]
            if cur_cost == INF:
                continue
            # try to go to next
            for nxt in range(n):
                if mask & (1<<nxt): 
                    continue
                nxt_mask = mask | (1<<nxt)
                cost = cur_cost + euclid(pts[last], pts[nxt])
                if cost < dp[nxt_mask][nxt]:
                    dp[nxt_mask][nxt] = cost
                    parent[nxt_mask][nxt] = last
    full_mask = size - 1
    # find best endpoint (we don't return to start)
    best_cost = INF
    best_last = -1
    for i in range(n):
        if dp[full_mask][i] < best_cost:
            best_cost = dp[full_mask][i]
            best_last = i
    # reconstruct path
    order = []
    mask = full_mask
    cur = best_last
    while cur != -1:
        order.append(pts[cur])
        prev = parent[mask][cur]
        mask ^= (1<<cur)
        cur = prev
    order.reverse()  # now order is visiting order from first visited -> last visited
    return order

def nearest_neighbor(start, pts):
    """
    Greedy nearest neighbor from start. Returns visiting order list.
    """
    pts_left = pts.copy()
    order = []
    cur = start
    while pts_left:
        # find nearest
        idx, best_d = 0, float('inf')
        for i, p in enumerate(pts_left):
            d = euclid(cur, p)
            if d < best_d:
                best_d = d
                idx = i
        order.append(pts_left.pop(idx))
        cur = order[-1]
    return order

def two_opt(order, start):
    """
    2-opt improvement on a path (order is visiting order).
    This improves without changing the start; path is not a cycle.
    Returns improved order.
    """
    def path_length(ordr):
        if not ordr:
            return 0.0
        s = euclid(start, ordr[0])
        for a, b in zip(ordr, ordr[1:]):
            s += euclid(a, b)
        return s

    improved = True
    n = len(order)
    if n < 3:
        return order
    while improved:
        improved = False
        best_gain = 0
        best_i, best_k = None, None
        base_len = path_length(order)
        for i in range(0, n-1):
            for k in range(i+1, n):
                # reverse segment i..k
                new_order = order[:i] + order[i:k+1][::-1] + order[k+1:]
                new_len = path_length(new_order)
                gain = base_len - new_len
                if gain > best_gain + 1e-12:
                    best_gain = gain
                    best_i, best_k = i, k
        if best_i is not None:
            order = order[:best_i] + order[best_i:best_k+1][::-1] + order[best_k+1:]
            improved = True
    return order

def optimize_destinations(destinations, exact_threshold=12):
    """
    destinations: list of tuples [(x0,y0), (x1,y1), ...]
      where destinations[0] is START coordinate.
    Returns: a new list in the same format where the first element is the same start,
             and the remaining elements are ordered so that destination.pop() yields
             the *next* target to visit (i.e., last element is immediate next).
    Strategy:
      - Extract start and targets.
      - If small number of targets (<= exact_threshold), use Held-Karp exact path.
      - Else use nearest-neighbor + 2-opt.
      - Then reverse the visiting order and produce final list [start, ..., last, first].
    """
    if not destinations:
        return []
    start = destinations[0]
    targets = destinations[1:]
    if not targets:
        return [start]
    n = len(targets)
    if n <= exact_threshold:
        visiting_order = held_karp_path(start, targets)
    else:
        visiting_order = nearest_neighbor(start, targets)
        visiting_order = two_opt(visiting_order, start)

    # visiting_order is [first_target, second_target, ... last_target]
    # but code uses pop() to get next location, so reverse them:
    reversed_for_pop = list(reversed(visiting_order))
    return [start] + reversed_for_pop

# ---------------------
# Example usage:
if __name__ == "__main__":
    destinations = [
        (0,0),   # start
        (5,2),
        (1,1),
        (6,6),
        (2,5),
        (9,0)
    ]
    new_list = optimize_destinations(destinations)
    print("Optimized destination list (first is start). Rover should pop() to get next target:")
    print(new_list)
    # simulate popping sequence
    sim = new_list.copy()
    print("\nSimulated travel sequence (start -> next -> ...):")
    cur = sim[0]
    print("Start:", cur)
    while len(sim) > 1:
        nxt = sim.pop()
        print(" ->", nxt)
