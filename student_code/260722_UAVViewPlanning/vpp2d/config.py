from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path

_DEFAULT_PATH = Path(__file__).with_name("config.toml")


@dataclass(frozen=True)
class Config:
    d_min: float
    d_max: float
    d_opt: float

    fov_deg: float

    baseline: float

    theta_max_deg: float

    safety_distance: float

    min_overlap: float

    n_incidence: int
    n_distance: int
    voxel: float
    yaw_bin_deg: float
    resolution: float

    @property
    def fov_rad(self) -> float:
        return math.radians(self.fov_deg)

    @property
    def theta_max_rad(self) -> float:
        return math.radians(self.theta_max_deg)

    @property
    def yaw_bin_rad(self) -> float:
        return math.radians(self.yaw_bin_deg)

    def summary(self) -> str:
        return (
            "Config(2D):\n"
            f"  Arbeitsabstand : [{self.d_min}, {self.d_max}] m (d_opt={self.d_opt})\n"
            f"  FoV / Inzidenz : {self.fov_deg}° / ≤{self.theta_max_deg}°\n"
            f"  Basislinie     : {self.baseline} m "
            f"({'duale Sicht' if self.baseline > 0 else 'Einzelsicht'})\n"
            f"  Sampling       : {self.n_incidence}×Inzidenz × {self.n_distance}×Distanz, "
            f"Dedup {self.voxel:.2f} m / {self.yaw_bin_deg}°"
        )


def load_config(path: str | Path | None = None) -> Config:
    path = Path(path) if path is not None else _DEFAULT_PATH
    with open(path, "rb") as fh:
        raw = tomllib.load(fh)

    wd = raw["working_distance"]
    d_min = float(wd["d_min"])
    d_max = float(wd["d_max"])
    d_opt = float(wd.get("d_opt", (d_min + d_max) / 2.0))

    smp = raw.get("sampling", {})
    voxel = float(smp.get("voxel", 0.15 * d_opt))

    return Config(
        d_min=d_min,
        d_max=d_max,
        d_opt=d_opt,
        fov_deg=float(raw["fov"]["fov_deg"]),
        baseline=float(raw.get("sensor", {}).get("baseline", 0.0)),
        theta_max_deg=float(raw["incidence"]["theta_max_deg"]),
        safety_distance=float(raw["drone"]["safety_distance"]),
        min_overlap=float(raw["registration"]["min_overlap"]),
        n_incidence=int(smp.get("n_incidence", 7)),
        n_distance=int(smp.get("n_distance", 2)),
        voxel=voxel,
        yaw_bin_deg=float(smp.get("yaw_bin_deg", 15.0)),
        resolution=float(smp.get("resolution", 0.25)),
    )
