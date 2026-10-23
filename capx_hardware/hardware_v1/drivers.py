"""Import source-pinned ORIGINAL drivers without changing their module files."""
import importlib.util
import json
import sys
from pathlib import Path
from .config import sha, resolve


class DriverLoader:
    def __init__(self, manifest, expected_sha256):
        self.path = Path(manifest).resolve()
        self.expected_sha256 = expected_sha256
        if sha(self.path) != expected_sha256:
            raise ValueError("Driver manifest hash differs")
        self.manifest = json.loads(self.path.read_text())
        if not isinstance(self.manifest, dict) or not isinstance(self.manifest.get("files"), dict) or not self.manifest["files"] or not isinstance(self.manifest.get("packages"), dict) or not self.manifest["packages"]:
            raise ValueError("Nonempty original-driver files/packages manifest required")
        self.verify()

    def verify(self):
        if sha(self.path) != self.expected_sha256:
            raise ValueError("Driver manifest changed after construction")
        for path, expected in self.manifest["files"].items():
            if not isinstance(path, str) or not path or not isinstance(expected, str) or len(expected) != 64 or any(x not in "0123456789abcdef" for x in expected):
                raise ValueError("Malformed driver source identity")
            if sha(resolve(self.path.parent, path)) != expected:
                raise ValueError("Original driver source changed: "+path)

    def module(self, family, module):
        self.verify()
        root = resolve(self.path.parent, self.manifest["packages"][family])
        target = root / (module+".py")
        registered = {resolve(self.path.parent, p) for p in self.manifest["files"]}
        if not target.is_file() or target not in registered:
            raise ValueError("Unregistered driver module")
        name = "_capx_driver_"+family+"_"+sha(self.path)[:16]
        old = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            if name not in sys.modules:
                spec = importlib.util.spec_from_file_location(name, root/"__init__.py", submodule_search_locations=[str(root)])
                package = importlib.util.module_from_spec(spec)
                sys.modules[name] = package
                spec.loader.exec_module(package)
            qualified = name+"."+module
            if qualified not in sys.modules:
                spec = importlib.util.spec_from_file_location(qualified, target)
                value = importlib.util.module_from_spec(spec)
                sys.modules[qualified] = value
                try:
                    spec.loader.exec_module(value)
                except BaseException:
                    sys.modules.pop(qualified, None)
                    raise
            return sys.modules[qualified]
        finally:
            sys.dont_write_bytecode = old
