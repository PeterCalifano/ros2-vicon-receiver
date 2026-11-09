"""Access to a robot model (URDF + meshes) for the live viewer and the marker mount estimate.

The model is the G1 Unitree profile fetched by scripts/fetch_g1_model.sh. Its URDF refers to the
meshes with ``package://`` URIs of another workspace; Resolve_mesh_path() finds them in the local
copy by matching the end of the path.

Needs yourdfpy and trimesh (in the conda environment of environment.yml).

Example::

    from pathlib import Path
    from vicon_receiver_tools.robot_model import Load_urdf, Rigid_mesh

    urdf = Load_urdf(Path("models/g1"), "model.urdf", load_meshes=False)
    mesh = Rigid_mesh(urdf, "torso_link", Path("models/g1"))
    print(mesh.bounds)   # torso + head + logo, in the torso_link frame
"""

from __future__ import annotations

import functools
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy


class RobotModelError(Exception):
    """The robot model cannot be loaded or does not contain what is needed."""


def Resolve_mesh_path(fname: str, model_dir: Path) -> str:
    """Find the local file of a mesh referenced in the URDF.

    ``package://`` and ``file://`` prefixes are removed, then shorter and shorter tails of the
    path are tried under model_dir until one exists. For example
    ``package://legged_nav_system/profiles/g1/meshes/pelvis.obj`` resolves to
    ``<model_dir>/meshes/pelvis.obj``.

    Args:
        fname: Mesh file name as written in the URDF (the keyword yourdfpy uses).
        model_dir: Directory of the local model copy.

    Returns:
        The path of the existing file, or the unchanged name if nothing matches (yourdfpy then
        reports the missing mesh).
    """
    path_text_ = fname
    for prefix_ in ("package://", "file://"):
        if path_text_.startswith(prefix_):
            path_text_ = path_text_[len(prefix_) :]

    parts_ = Path(path_text_).parts
    for start_ in range(len(parts_)):
        candidate_ = model_dir.joinpath(*parts_[start_:])
        if candidate_.is_file():
            return str(candidate_)
    return fname


def Load_urdf(model_dir: Path, urdf_name: str, load_meshes: bool) -> yourdfpy.URDF:
    """Load the URDF of the model, resolving its mesh paths in model_dir.

    Raises:
        RobotModelError: If the URDF file does not exist.
    """
    urdf_path_ = model_dir / urdf_name
    if not urdf_path_.is_file():
        raise RobotModelError(f"URDF not found: {urdf_path_} (run scripts/fetch_g1_model.sh)")

    handler_ = functools.partial(Resolve_mesh_path, model_dir=model_dir)
    return yourdfpy.URDF.load(
        str(urdf_path_),
        filename_handler=handler_,
        load_meshes=load_meshes,
        build_scene_graph=True,
    )


def Link_pose(urdf: yourdfpy.URDF, link: str, reference: str) -> np.ndarray:
    """Pose (4x4) of a link in the frame of another link, at the current joint configuration.

    The configuration of the URDF is zero after loading; update_cfg() changes it. Works in both
    directions of the kinematic tree (the scene graph is directed).

    Raises:
        RobotModelError: If a link does not exist or the two are not connected.
    """
    known_links_ = {link_.name for link_ in urdf.robot.links}
    for name_ in (link, reference):
        if name_ not in known_links_:
            raise RobotModelError(f"link '{name_}' not in the URDF")

    try:
        return np.asarray(urdf.get_transform(frame_to=link, frame_from=reference))
    except ValueError:
        pass
    # The scene graph only has parent -> child edges: invert the opposite query
    try:
        return np.linalg.inv(np.asarray(urdf.get_transform(frame_to=reference, frame_from=link)))
    except ValueError as error_:
        raise RobotModelError(
            f"no transform between '{link}' and '{reference}': {error_}"
        ) from error_


def Rigid_subtree(urdf: yourdfpy.URDF, link: str) -> list[str]:
    """Links rigidly attached below a link (through fixed joints), the link itself first."""
    children_by_parent_: dict[str, list[yourdfpy.Joint]] = {}
    for joint_ in urdf.robot.joints:
        children_by_parent_.setdefault(joint_.parent, []).append(joint_)

    subtree_ = [link]
    pending_ = [link]
    while pending_:
        parent_ = pending_.pop()
        for joint_ in children_by_parent_.get(parent_, []):
            if joint_.type == "fixed":
                subtree_.append(joint_.child)
                pending_.append(joint_.child)
    return subtree_


def _Visual_meshes(
    link: yourdfpy.Link, link_pose: np.ndarray, model_dir: Path
) -> list[trimesh.Trimesh]:
    """Visual meshes of a URDF link, moved by the pose of the link."""
    meshes_: list[trimesh.Trimesh] = []
    for visual_ in link.visuals:
        if visual_.geometry.mesh is None:
            continue
        mesh_path_ = Resolve_mesh_path(visual_.geometry.mesh.filename, model_dir)
        mesh_ = trimesh.load(mesh_path_, force="mesh")
        visual_origin_ = np.eye(4) if visual_.origin is None else visual_.origin
        mesh_.apply_transform(link_pose @ visual_origin_)
        meshes_.append(mesh_)
    return meshes_


def Rigid_mesh(urdf: yourdfpy.URDF, link: str, model_dir: Path) -> trimesh.Trimesh:
    """Visual meshes of a link and of the links rigidly attached to it, in the link frame.

    Raises:
        RobotModelError: If none of these links has a mesh.
    """
    links_by_name_ = {link_.name: link_ for link_ in urdf.robot.links}
    meshes_: list[trimesh.Trimesh] = []

    for name_ in Rigid_subtree(urdf, link):
        meshes_ += _Visual_meshes(links_by_name_[name_], Link_pose(urdf, name_, link), model_dir)

    if not meshes_:
        raise RobotModelError(f"no visual mesh on '{link}' or the links fixed to it")
    return trimesh.util.concatenate(meshes_)


def Link_mesh(
    urdf: yourdfpy.URDF, mesh_link: str, frame_link: str, model_dir: Path
) -> trimesh.Trimesh:
    """Visual meshes of one link only, expressed in the frame of another link.

    Raises:
        RobotModelError: If the link has no mesh.
    """
    links_by_name_ = {link_.name: link_ for link_ in urdf.robot.links}
    if mesh_link not in links_by_name_:
        raise RobotModelError(f"link '{mesh_link}' not in the URDF")

    link_pose_ = Link_pose(urdf, mesh_link, frame_link)
    meshes_ = _Visual_meshes(links_by_name_[mesh_link], link_pose_, model_dir)

    if not meshes_:
        raise RobotModelError(f"link '{mesh_link}' has no visual mesh")
    return trimesh.util.concatenate(meshes_)
