"""Load the prepared model on a host or in the fixed container mount."""
from pathlib import Path

HERE = Path(__file__).resolve().parent


def model_xml(root=None):
    root=HERE if root is None else Path(root)
    path = root/'generated/cap_button/cap_button.xml'
    text = path.read_text().replace('/workspace/project/mujoco/', str(root.parent) + '/')
    return path, text


def load_model(root=None):
    import mujoco
    _, text = model_xml(root)
    return mujoco.MjModel.from_xml_string(text)
