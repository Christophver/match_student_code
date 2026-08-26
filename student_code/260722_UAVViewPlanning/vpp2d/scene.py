from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Facets:
    centers: np.ndarray
    normals: np.ndarray
    a: np.ndarray
    b: np.ndarray
    ids: np.ndarray

    def __len__(self) -> int:
        return len(self.ids)


@dataclass
class Scene:
    polygons: list[np.ndarray]
    facets: Facets
    edges: np.ndarray
    name: str = "scene"

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        allv = np.vstack(self.polygons)
        return (allv[:, 0].min(), allv[:, 1].min(),
                allv[:, 0].max(), allv[:, 1].max())

    def summary(self) -> str:
        x0, y0, x1, y1 = self.bounds
        return (
            f"Scene '{self.name}': {len(self.polygons)} Polygon(e), "
            f"{len(self.facets)} Facetten, {len(self.edges)} Occluder-Kanten\n"
            f"  Ausdehnung: {x1 - x0:.1f} × {y1 - y0:.1f} m"
        )


def _signed_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _ensure_ccw(poly: np.ndarray) -> np.ndarray:
    return poly if _signed_area(poly) > 0 else poly[::-1].copy()


def build_scene(polygons: list[np.ndarray], resolution: float,
                name: str = "scene") -> Scene:
    centers, normals, segs_a, segs_b = [], [], [], []
    edges = []
    for poly in polygons:
        poly = _ensure_ccw(np.asarray(poly, dtype=float))
        m = len(poly)
        for i in range(m):
            v0 = poly[i]
            v1 = poly[(i + 1) % m]
            edges.append([v0, v1])
            d = v1 - v0
            length = float(np.hypot(*d))
            if length < 1e-12:
                continue
            n = np.array([d[1], -d[0]]) / length
            k = max(1, int(round(length / resolution)))
            ts = (np.arange(k) + 0.5) / k
            for j in range(k):
                a = v0 + d * (j / k)
                b = v0 + d * ((j + 1) / k)
                centers.append(v0 + d * ts[j])
                normals.append(n)
                segs_a.append(a)
                segs_b.append(b)

    centers = np.asarray(centers, dtype=float)
    normals = np.asarray(normals, dtype=float)
    facets = Facets(
        centers=centers,
        normals=normals,
        a=np.asarray(segs_a, dtype=float),
        b=np.asarray(segs_b, dtype=float),
        ids=np.arange(len(centers), dtype=np.int64),
    )
    return Scene(
        polygons=[_ensure_ccw(np.asarray(p, float)) for p in polygons],
        facets=facets,
        edges=np.asarray(edges, dtype=float),
        name=name,
    )


def box(resolution: float, size: float = 4.0, name: str = "box") -> Scene:
    h = size / 2.0
    poly = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    return build_scene([poly], resolution, name)


def notched_box(resolution: float, w: float = 4.4, h: float = 3.1,
                notch: float = 1.2, name: str = "notched_box") -> Scene:
    x0, y0 = 0.0, 0.0
    x1, y1 = w, h
    nd = notch
    cy = h / 2.0
    poly = np.array([
        [x0, y0],
        [x1, y0],
        [x1, cy - nd / 2],
        [x1 - nd, cy - nd / 2],
        [x1 - nd, cy + nd / 2],
        [x1, cy + nd / 2],
        [x1, y1],
        [x0, y1],
    ])
    return build_scene([poly], resolution, name)


def _read_stl_triangles(path: str) -> np.ndarray:
    with open(path, "rb") as fh:
        head = fh.read(5)
        fh.seek(0)
        if head == b"solid":
            data = fh.read()
            if b"facet" in data[:512]:
                verts = []
                for line in data.decode("ascii", "ignore").splitlines():
                    p = line.split()
                    if len(p) == 4 and p[0] == "vertex":
                        verts.append([float(p[1]), float(p[2]), float(p[3])])
                tri = np.asarray(verts, dtype=float).reshape(-1, 3, 3)
                return tri
            fh.seek(0)
        fh.seek(80)
        n = int(np.frombuffer(fh.read(4), dtype="<u4")[0])
        rec = np.frombuffer(fh.read(n * 50), dtype=np.uint8).reshape(n, 50)
        floats = rec[:, :48].copy().view("<f4").reshape(n, 12)
        return floats[:, 3:].reshape(n, 3, 3).astype(float)


def from_stl_slice(path: str, axis: int = 2, level: float | None = None,
                   plane: tuple[int, int] = (0, 1), resolution: float = 0.25,
                   name: str | None = None) -> Scene:
    tri = _read_stl_triangles(path)
    if level is None:
        level = float((tri[..., axis].min() + tri[..., axis].max()) / 2.0)

    segs: list[np.ndarray] = []
    pa, pb = plane
    for t in tri:
        d = t[:, axis] - level
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            di, dj = d[i], d[j]
            if (di <= 0 < dj) or (dj <= 0 < di):
                s = di / (di - dj)
                p = t[i] + s * (t[j] - t[i])
                pts.append([p[pa], p[pb]])
        if len(pts) == 2:
            segs.append(np.asarray(pts, dtype=float))

    if not segs:
        raise ValueError(f"Schnitt bei {axis}={level} ergab keine Segmente.")

    polygons = _chain_segments(np.asarray(segs))
    stem = path.replace("\\", "/").split("/")[-1].rsplit(".", 1)[0]
    nm = name or f"slice_{stem}"
    return build_scene(polygons, resolution, nm)


def _chain_segments(segs: np.ndarray, tol: float = 1e-6) -> list[np.ndarray]:
    remaining = list(map(tuple, segs.reshape(-1, 2, 2)))
    polys: list[np.ndarray] = []
    while remaining:
        seg = remaining.pop()
        chain = [np.asarray(seg[0]), np.asarray(seg[1])]
        changed = True
        while changed:
            changed = False
            for idx, s in enumerate(remaining):
                s0, s1 = np.asarray(s[0]), np.asarray(s[1])
                if np.allclose(chain[-1], s0, atol=tol):
                    chain.append(s1); remaining.pop(idx); changed = True; break
                if np.allclose(chain[-1], s1, atol=tol):
                    chain.append(s0); remaining.pop(idx); changed = True; break
        poly = np.asarray(chain)
        if len(poly) >= 3:
            if np.allclose(poly[0], poly[-1], atol=tol):
                poly = poly[:-1]
            polys.append(poly)
    return polys


SCENES = {
    "box": box,
    "notched_box": notched_box,
}
