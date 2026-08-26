from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Route:
    order: np.ndarray
    length: float

    def summary(self) -> str:
        return f"Route: {len(self.order)} Posen, Flugweg {self.length:.1f} m"


def _nn_order(points: np.ndarray, start: int = 0) -> list[int]:
    n = len(points)
    if n <= 1:
        return list(range(n))
    unvisited = set(range(n))
    order = [start]
    unvisited.discard(start)
    while unvisited:
        last = order[-1]
        nxt = min(unvisited, key=lambda i: np.hypot(*(points[i] - points[last])))
        order.append(nxt)
        unvisited.discard(nxt)
    return order


def _path_len(points: np.ndarray, order: list[int]) -> float:
    return float(sum(np.hypot(*(points[order[i + 1]] - points[order[i]]))
                     for i in range(len(order) - 1)))


def _two_opt(points: np.ndarray, order: list[int]) -> list[int]:
    best = order[:]
    best_len = _path_len(points, best)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for k in range(i + 1, len(best)):
                cand = best[:i] + best[i:k + 1][::-1] + best[k + 1:]
                cl = _path_len(points, cand)
                if cl + 1e-9 < best_len:
                    best, best_len = cand, cl
                    improved = True
    return best


def sequence_route(pose_positions: np.ndarray) -> Route:
    if len(pose_positions) == 0:
        return Route(np.empty(0, np.int64), 0.0)
    order = _nn_order(pose_positions, start=0)
    order = _two_opt(pose_positions, order)
    return Route(np.asarray(order, np.int64), _path_len(pose_positions, order))
