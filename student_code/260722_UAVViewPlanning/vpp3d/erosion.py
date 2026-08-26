from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp
from scipy.spatial import cKDTree


@dataclass
class ErosionResult:
    V: sp.csr_matrix
    erosion: float
    radius: float
    n_iter: int
    nnz_before: int
    nnz_after: int
    reachable_before: int
    reachable_after: int
    n_empty: int

    def summary(self) -> str:
        drop = 1.0 - self.nnz_after / max(1, self.nnz_before)
        return (
            f"Footprint-Erosion (Rand {self.erosion:.2f} m ~= {self.n_iter} "
            f"Ring(e) a {self.radius:.2f} m):\n"
            f"  Sichtbarkeiten        : {self.nnz_before:,} -> "
            f"{self.nnz_after:,} (-{drop:.0%})\n"
            f"  erreichbare Facetten  : {self.reachable_before:,} -> "
            f"{self.reachable_after:,} (Planung; real gilt das volle V)\n"
            f"  leere Footprints      : {self.n_empty:,} von {self.V.shape[0]:,}"
        )


def _facet_adjacency(centers: np.ndarray, radius: float) -> sp.csr_matrix:
    pairs = cKDTree(centers).query_pairs(radius, output_type="ndarray")
    n = len(centers)
    loops = np.arange(n, dtype=np.int64)
    i = np.concatenate([pairs[:, 0], pairs[:, 1], loops])
    j = np.concatenate([pairs[:, 1], pairs[:, 0], loops])
    return sp.csr_matrix((np.ones(len(i), dtype=np.int32), (i, j)), shape=(n, n))


def erode_footprints(
    V: sp.csr_matrix,
    centers: np.ndarray,
    resolution: float,
    erosion: float,
) -> ErosionResult:
    radius = float(resolution)
    n_iter = max(1, int(round(erosion / radius)))
    adj = _facet_adjacency(np.asarray(centers, dtype=np.float64), radius)
    deg = np.asarray(adj.sum(axis=0)).ravel()

    Vc = V.tocsr().astype(np.int32)
    nnz_before = int(Vc.nnz)
    reach_before = int((np.asarray(Vc.sum(axis=0)).ravel() > 0).sum())

    Ve = Vc
    for _ in range(n_iter):
        D = (Ve @ adj).tocoo()
        keep = D.data == deg[D.col]
        Ve = sp.csr_matrix(
            (np.ones(int(keep.sum()), dtype=np.int32),
             (D.row[keep], D.col[keep])),
            shape=V.shape,
        )

    return ErosionResult(
        V=Ve.astype(bool).tocsr(),
        erosion=float(erosion),
        radius=radius,
        n_iter=n_iter,
        nnz_before=nnz_before,
        nnz_after=int(Ve.nnz),
        reachable_before=reach_before,
        reachable_after=int((np.asarray(Ve.sum(axis=0)).ravel() > 0).sum()),
        n_empty=int((np.diff(Ve.indptr) == 0).sum()),
    )


def cover_residual(
    V: sp.csr_matrix, selected: np.ndarray
) -> tuple[np.ndarray, int]:
    from .setcover import greedy_set_cover

    Vc = V.tocsr()
    reach = np.asarray(Vc.sum(axis=0)).ravel() > 0
    cov = (np.asarray(Vc[selected].sum(axis=0)).ravel() > 0
           if len(selected) else np.zeros(Vc.shape[1], dtype=bool))
    residual = np.flatnonzero(reach & ~cov)
    if not len(residual):
        return np.empty(0, dtype=np.int64), 0
    extra = greedy_set_cover(Vc[:, residual]).poses
    return np.setdiff1d(extra, selected), int(len(residual))
