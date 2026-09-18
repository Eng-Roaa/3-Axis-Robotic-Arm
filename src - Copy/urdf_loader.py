import os
import re
import tempfile

from roboticstoolbox.models.URDF.URDFRobot import URDFRobot


def load_solidworks_urdf(urdf_path):
    """
    Loads a SolidWorks-exported URDF into Robotics Toolbox.

    SolidWorks commonly creates mesh paths like:
        package://RobotName/meshes/Link_1.STL

    This function converts them into absolute Windows paths.
    """

    urdf_path = os.path.abspath(urdf_path)

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(
            f"URDF file not found:\n{urdf_path}"
        )

    urdf_dir = os.path.dirname(urdf_path)

    # Expected:
    # Robotics Arm URDF/
    #   urdf/
    #      robot.urdf
    #   meshes/
    meshes_dir = os.path.abspath(
        os.path.join(urdf_dir, "..", "meshes")
    )

    if not os.path.isdir(meshes_dir):
        raise FileNotFoundError(
            f"Meshes directory not found:\n{meshes_dir}"
        )

    with open(urdf_path, "r", encoding="utf-8") as f:
        text = f.read()

    # ---------------------------------------------------------
    # 1. Fix SolidWorks empty material names
    #
    # SolidWorks exports:
    # <material name="">
    #
    # Give it a valid name.
    # ---------------------------------------------------------
    text = text.replace(
        '<material name="">',
        '<material name="solidworks_default">'
    )

    # ---------------------------------------------------------
    # 2. Convert:
    #
    # package://Robotics ARm.SLDASM/meshes/Link_1.STL
    #
    # to:
    #
    # C:/.../Robotics Arm URDF/meshes/Link_1.STL
    # ---------------------------------------------------------

    def replace_package_path(match):
        mesh_name = match.group(1)

        local_mesh = os.path.join(
            meshes_dir,
            mesh_name
        )

        local_mesh = os.path.abspath(local_mesh)

        if not os.path.isfile(local_mesh):
            raise FileNotFoundError(
                f"Mesh file not found:\n{local_mesh}"
            )

        # URDF/XML accepts forward slashes on Windows.
        return local_mesh.replace("\\", "/")

    pattern = r'package://[^/]+/meshes/([^"\']+)'

    text = re.sub(
        pattern,
        replace_package_path,
        text
    )

    # ---------------------------------------------------------
    # 3. Write a temporary patched URDF
    # ---------------------------------------------------------

    temp_dir = tempfile.mkdtemp(prefix="solidworks_urdf_")

    patched_urdf = os.path.join(
        temp_dir,
        "robot_patched.urdf"
    )

    with open(
        patched_urdf,
        "w",
        encoding="utf-8"
    ) as f:
        f.write(text)

    # ---------------------------------------------------------
    # 4. Load using current Robotics Toolbox API
    # ---------------------------------------------------------

    robot = URDFRobot(patched_urdf)

    return robot