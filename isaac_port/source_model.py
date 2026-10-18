"""Load the prepared model on a host or in the fixed container mount."""
from pathlib import Path

HERE = Path(__file__).resolve().parent


def model_xml():
    path = HERE/'generated/cap_button/cap_button.xml'
    text = path.read_text().replace('/workspace/project/mujoco/', str(HERE.parent) + '/')
    return path, text


def load_model():
    import mujoco
    _, text = model_xml()
    return mujoco.MjModel.from_xml_string(text)
