"""Evidence-bound FR3 skill library; no generated code is executed here.

Adapted from CaP-X ``capx/skills/library.py`` and ``extractor.py`` at
53e9966d7a8e2fa7494676772bccc35280f5c0ed, Copyright (c) 2026 Max Fu, MIT.
See THIRD_PARTY_LICENSES/CaP-X-MIT.txt. Occurrence promotion and ``exec``
injection are intentionally replaced with externally verified evidence.
"""
from __future__ import annotations

import ast
import hashlib
import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable

UPSTREAM_COMMIT = "53e9966d7a8e2fa7494676772bccc35280f5c0ed"

# CaP-X's original patterns remain available for partial code compatibility.
FUNCTION_DEF_PATTERN = re.compile(
    r'^(def\s+\w+\s*\([^)]*\)\s*(?:->\s*[^:]+)?:\s*\n(?:(?:[ \t]+.+\n?)+))',
    re.MULTILINE,
)
FUNCTION_SIG_PATTERN = re.compile(
    r'^def\s+(\w+)\s*\(([^)]*)\)\s*(?:->\s*([^:]+))?:', re.MULTILINE
)
DOCSTRING_PATTERN = re.compile(
    r'^\s+(?:\"\"\"([\s\S]*?)\"\"\"|\'\'\'([\s\S]*?)\'\'\')', re.MULTILINE
)


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def extract_docstring(func_code: str) -> str:
    """CaP-X docstring extraction for regex-compatible partial snippets."""
    lines = func_code.split("\n", 1)
    if len(lines) < 2:
        return ""
    match = DOCSTRING_PATTERN.match(lines[1])
    return (match.group(1) or match.group(2) or "").strip() if match else ""


def extract_functions(code: str) -> list[dict[str, str]]:
    """Extract top-level functions without compiling or executing the code.

    CaP-X's regex is retained as a fallback for incomplete trial snippets.
    Complete code uses AST source extents to retain blank lines, decorators,
    nested parentheses and async definitions omitted by the original regex.
    Extraction by itself never validates or activates a skill.
    """
    try:
        tree = ast.parse(code)
    except SyntaxError:
        functions = []
        for match in FUNCTION_DEF_PATTERN.finditer(code):
            full_def = match.group(1).rstrip()
            sig_match = FUNCTION_SIG_PATTERN.match(full_def)
            if sig_match:
                name, params, result = sig_match.groups()
                signature = f"def {name}({params.strip()})"
                if result:
                    signature += f" -> {result.strip()}"
                functions.append(dict(name=name, signature=signature, code=full_def,
                                      docstring=extract_docstring(full_def)))
        return functions
    lines = code.splitlines(keepends=True)
    result = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            start = min([node.lineno] + [d.lineno for d in node.decorator_list]) - 1
            full_def = "".join(lines[start:node.end_lineno]).rstrip()
            signature = ("async " if isinstance(node, ast.AsyncFunctionDef) else "")
            signature += f"def {node.name}({ast.unparse(node.args)})"
            if node.returns:
                signature += f" -> {ast.unparse(node.returns)}"
            result.append(dict(name=node.name, signature=signature, code=full_def,
                               docstring=ast.get_docstring(node) or ""))
    return result


@dataclass(frozen=True)
class EvidenceBinding:
    """Exact environment identities; a changed hash requires new qualification."""
    scene_sha256: str
    config_sha256: str
    source_sha256: str

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not re.fullmatch(r"[a-fA-F0-9]{64}", value):
                raise ValueError(f"{name} must be a SHA256 hex digest")


@dataclass(frozen=True)
class ValidationEvidence:
    """Claims supplied to a mandatory trusted external proof verifier.

    These fields are not proof: the verifier must check actual run artifacts,
    their digests, full native collision coverage and completion observations.
    A pre-existing 575 recovery/visual-alignment success cannot qualify as a
    normal fixed-path insertion success through this interface.
    """
    evidence_id: str
    binding: EvidenceBinding
    function_sha256: str
    proof_path: str
    proof_sha256: str
    domain: str = "simulation"
    scope: str = "full_task"
    mode: str = "normal"
    completed: bool = False
    recovery_used: bool = False
    fixed_insert: bool = False
    collision_method: str = ""
    phases: tuple[str, ...] = ()


REQUIRED_PHASES = frozenset({
    "perception", "grasp", "carry", "preinsert", "fixed_insert", "release", "withdraw", "final_check"
})


@dataclass
class Skill:
    """CaP-X skill record, extended with immutable-code identity and provenance."""
    name: str
    code: str
    docstring: str = ""
    occurrences: int = 1
    source_tasks: list[str] = field(default_factory=list)
    promoted: bool = False
    code_sha256: str = ""
    evidence: list[ValidationEvidence] = field(default_factory=list)


class EvidenceRejected(ValueError):
    pass


class SkillLibrary:
    """Extract, persist and review skills without automatic execution.

    Unlike CaP-X, frequency is diagnostic only. Promotion and every reuse need
    an exact binding and an external verifier. No verifier means no promotion.
    Simulation evidence is never silently reused for hardware.
    """
    DEFAULT_PATH = Path(".fr3_capx_skills.json")

    def __init__(self, path: Path | str | None = None,
                 verify_evidence: Callable[[ValidationEvidence], bool] | None = None):
        self.path = Path(path) if path else self.DEFAULT_PATH
        self.verify_evidence = verify_evidence
        self.skills: dict[str, Skill] = {}
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        data = json.loads(self.path.read_text())
        if data.get("schema") != "fr3-capx-skills/v1":
            raise ValueError("Unrecognized skill schema; implicit upstream promotion is forbidden")
        for name, raw in data.get("skills", {}).items():
            info = dict(raw)
            evidence = []
            for entry in info.pop("evidence", []):
                entry = dict(entry)
                entry["binding"] = EvidenceBinding(**entry["binding"])
                entry["phases"] = tuple(entry.get("phases", ()))
                evidence.append(ValidationEvidence(**entry))
            skill = Skill(**info, evidence=evidence)
            if name != skill.name or skill.code_sha256 != sha256_text(skill.code):
                raise ValueError("Skill source identity mismatch")
            self.skills[name] = skill

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"schema": "fr3-capx-skills/v1", "upstream_commit": UPSTREAM_COMMIT,
                "skills": {name: asdict(skill) for name, skill in self.skills.items()}}
        temporary = self.path.with_name(self.path.name + ".tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n")
        temporary.replace(self.path)

    def extract_from_code(self, code: str, task_name: str = "") -> list[str]:
        new_names = []
        for func in extract_functions(code):
            if func["name"] not in self.skills:
                new_names.append(func["name"])
            self.add_skill(func["name"], func["code"], func["docstring"], task_name)
        return new_names

    def add_skill(self, name: str, code: str, docstring: str = "", source_task: str = "") -> None:
        functions = extract_functions(code)
        if len(functions) != 1 or functions[0]["name"] != name:
            raise ValueError("A skill must contain one matching top-level function")
        # Invalid partial snippets may be catalogued by extract_functions, but
        # must not be added to the reusable library as valid source.
        ast.parse(code)
        digest = sha256_text(code)
        if name in self.skills and self.skills[name].code_sha256 == digest:
            skill = self.skills[name]
            skill.occurrences += 1
            if docstring:
                skill.docstring = docstring
        else:
            # Updating a function revokes previous promotion and all proof.
            skill = Skill(name=name, code=code, docstring=docstring, code_sha256=digest)
            self.skills[name] = skill
        if source_task and source_task not in skill.source_tasks:
            skill.source_tasks.append(source_task)

    def _qualifies(self, skill: Skill, evidence: ValidationEvidence,
                   binding: EvidenceBinding, domain: str) -> bool:
        if domain not in ("simulation", "hardware"):
            return False
        if (evidence.binding != binding or evidence.domain != domain or
                evidence.function_sha256 != sha256_text(skill.code) or
                skill.code_sha256 != sha256_text(skill.code) or
                evidence.scope != "full_task" or evidence.mode != "normal" or
                not evidence.completed or evidence.recovery_used or not evidence.fixed_insert or
                evidence.collision_method != "native_full_path" or
                not REQUIRED_PHASES.issubset(evidence.phases) or
                not evidence.evidence_id or not evidence.proof_path or
                not re.fullmatch(r"[a-fA-F0-9]{64}", evidence.proof_sha256)):
            return False
        if self.verify_evidence is None:
            return False
        try:
            return self.verify_evidence(evidence) is True
        except Exception:
            return False

    def promote(self, name: str, *, evidence: ValidationEvidence,
                binding: EvidenceBinding, domain: str = "simulation") -> None:
        skill = self.skills[name]
        if not self._qualifies(skill, evidence, binding, domain):
            raise EvidenceRejected("Normal fixed-path full-task/native proof is missing or mismatched")
        if evidence not in skill.evidence:
            skill.evidence.append(evidence)
        skill.promoted = True

    def get_promoted_skills(self, *, binding: EvidenceBinding,
                            domain: str = "simulation") -> dict[str, str]:
        return {name: s.code for name, s in self.skills.items()
                if s.promoted and any(self._qualifies(s, e, binding, domain) for e in s.evidence)}

    def get_skill_docs(self, *, binding: EvidenceBinding, domain: str = "simulation") -> str:
        promoted = self.get_promoted_skills(binding=binding, domain=domain)
        if not promoted:
            return "No evidence-qualified skills available."
        return "\n\n".join(f"## {name}\n{self.skills[name].docstring}\n```python\n{code}\n```"
                            for name, code in sorted(promoted.items()))

    def inject_into_namespace(self, namespace: dict[str, Any]) -> None:
        raise RuntimeError("Generated skill execution is disabled; use reviewed backend callbacks")

    def remove_skill(self, name: str) -> None:
        self.skills.pop(name, None)

    def summary(self) -> str:
        return (f"Skill Library: {len(self.skills)} skills "
                f"({sum(s.promoted for s in self.skills.values())} promotion records; "
                "reuse requires fresh binding verification)")

    def functions(self) -> dict[str, Callable[..., Any]]:
        return {name: getattr(self, name) for name in (
            "extract_from_code", "get_promoted_skills", "get_skill_docs", "summary"
        )}
