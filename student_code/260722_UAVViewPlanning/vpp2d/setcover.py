from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp


@dataclass
class CoverResult:
    poses: np.ndarray
    method: str
    n_coverable: int = 0
    n_covered: int = 0
    runtime: float = 0.0
    optimal: bool = False
    extra: dict = field(default_factory=dict)

    def summary(self) -> str:
        opt = " (Optimum bewiesen)" if self.optimal else ""
        return (
            f"Set Cover [{self.method}{opt}]:\n"
            f"  Drohnenposen (Scans)            : {len(self.poses)}\n"
            f"  Abdeckung erreichbarer Facetten : "
            f"{self.n_covered}/{self.n_coverable} "
            f"({self.n_covered / max(1, self.n_coverable):.1%})\n"
            f"  Rechenzeit                      : {self.runtime:.2f} s"
        )


def _coverable(V: sp.csr_matrix) -> np.ndarray:
    return np.asarray(V.sum(axis=0)).ravel() > 0


def greedy_set_cover(V: sp.csr_matrix, trace: list | None = None) -> CoverResult:
    t0 = time.perf_counter()
    Vc = V.tocsr().astype(np.int64)
    coverable = _coverable(Vc)
    deficit = coverable.copy()
    chosen = np.zeros(Vc.shape[0], dtype=bool)
    selected: list[int] = []

    while deficit.any():
        gain = np.asarray(Vc @ deficit.astype(np.int64)).ravel()
        gain[chosen] = 0
        j = int(np.argmax(gain))
        if gain[j] == 0:
            break
        seen = Vc.indices[Vc.indptr[j]:Vc.indptr[j + 1]]
        if trace is not None:
            open_seen = deficit[seen]
            trace.append({
                "pose": int(j),
                "covered_before": (coverable & ~deficit).copy(),
                "new_facets": seen[open_seen].copy(),
                "overlap_facets": seen[~open_seen].copy(),
            })
        chosen[j] = True
        selected.append(j)
        deficit[seen] = False

    n_covered = int(coverable.sum() - deficit.sum())
    return CoverResult(
        poses=np.asarray(selected, dtype=np.int64),
        method="greedy",
        n_coverable=int(coverable.sum()),
        n_covered=n_covered,
        runtime=time.perf_counter() - t0,
    )


def ilp_set_cover(V: sp.csr_matrix, time_limit: float = 120.0) -> CoverResult:
    from ortools.sat.python import cp_model

    t0 = time.perf_counter()
    Vcsc = V.tocsc()
    M, N = V.shape
    coverable = _coverable(V)

    model = cp_model.CpModel()
    x = [model.new_bool_var(f"x{j}") for j in range(M)]
    for i in np.flatnonzero(coverable):
        rows = Vcsc.indices[Vcsc.indptr[i]:Vcsc.indptr[i + 1]]
        model.add(sum(x[j] for j in rows) >= 1)
    model.minimize(sum(x))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit)
    solver.parameters.num_workers = 8
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError(f"CP-SAT ohne Loesung ({solver.status_name(status)})")

    sel = np.asarray([j for j in range(M) if solver.value(x[j])], dtype=np.int64)
    times = np.asarray(V.tocsr()[sel].sum(axis=0)).ravel() if sel.size else np.zeros(N)
    n_cov = int((coverable & (times >= 1)).sum())
    return CoverResult(
        poses=sel,
        method="ilp",
        n_coverable=int(coverable.sum()),
        n_covered=n_cov,
        runtime=time.perf_counter() - t0,
        optimal=status == cp_model.OPTIMAL,
        extra={"solver_status": solver.status_name(status)},
    )
