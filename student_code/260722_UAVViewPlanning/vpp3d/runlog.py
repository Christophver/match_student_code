from __future__ import annotations

import datetime
import shutil
from pathlib import Path

import numpy as np

_RUNS_ROOT = Path(__file__).with_name("output") / "runs"


class RunLog:
    def __init__(self, scene_name: str = "scene"):
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.dir = _RUNS_ROOT / f"{ts}_{scene_name}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.scene_name = scene_name
        print(f"  RunLog → {self.dir}")


    def save_matrix_png(self, vis, selected_rows: np.ndarray) -> Path:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        V_sel = np.asarray(vis.V[selected_rows].todense()).astype(np.uint8)
        n_poses, n_facets = V_sel.shape

        col_order = np.argsort(-V_sel.sum(axis=0))
        V_disp = V_sel[:, col_order]

        MAX_W, MAX_H = 2000, 800
        step_c = max(1, n_facets // MAX_W)
        step_r = max(1, n_poses  // MAX_H)
        V_disp = V_disp[::step_r, ::step_c]

        ds_c = f" (1:{step_c})" if step_c > 1 else ""
        ds_r = f" (1:{step_r})" if step_r > 1 else ""

        fig_w = max(8, min(20, V_disp.shape[1] / 80))
        fig_h = max(3, min(10, V_disp.shape[0] / 15 + 1.2))
        fig, ax = plt.subplots(figsize=(fig_w, fig_h))
        ax.imshow(V_disp, aspect="auto", interpolation="nearest",
                  cmap="Greens", vmin=0, vmax=1)
        ax.set_xlabel(f"Facetten ({n_facets}{ds_c}, sortiert nach Abdeckung)")
        ax.set_ylabel(f"Gewaehlte Posen ({n_poses}{ds_r})")
        ax.set_title(
            f"Sichtbarkeitsmatrix  {self.scene_name}  —  "
            f"{n_poses} Posen × {n_facets} Facetten  "
            f"({int(V_sel.sum())} sichtbare Eintraege)"
        )
        fig.tight_layout()

        path = self.dir / "visibility_matrix.png"
        fig.savefig(path, dpi=120)
        plt.close(fig)
        print(f"  Matrix PNG → {path}")
        return path


    def copy_png(self, src: "str | Path") -> Path | None:
        src = Path(src)
        if src.exists():
            dst = self.dir / src.name
            shutil.copy2(src, dst)
            return dst
        return None
