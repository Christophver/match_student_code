from .config import Config, load_config
from .scene import Scene, Facets, build_scene, box, notched_box, from_file, SCENES
from .candidates import Poses, sample_pose_regions, project_and_dedup
from .visibility import VisibilityMatrix, compute_visibility
from .setcover import (
    CoverResult, greedy_set_cover, ilp_set_cover, connected_set_cover_ilp,
)
from .overlap import (
    OverlapResult, ensure_connected, registration_graph, overlap_adjacency,
)
from .sequencing import Route, sequence_route
from .refine import (
    RefineInfo, pose_quality, refine_poses, target_facets_from_visibility,
)

__all__ = [
    "Config", "load_config",
    "Scene", "Facets", "build_scene", "box", "notched_box", "from_file", "SCENES",
    "Poses", "sample_pose_regions", "project_and_dedup",
    "VisibilityMatrix", "compute_visibility",
    "CoverResult", "greedy_set_cover", "ilp_set_cover", "connected_set_cover_ilp",
    "OverlapResult", "ensure_connected", "registration_graph", "overlap_adjacency",
    "Route", "sequence_route",
    "RefineInfo", "pose_quality", "refine_poses", "target_facets_from_visibility",
]
