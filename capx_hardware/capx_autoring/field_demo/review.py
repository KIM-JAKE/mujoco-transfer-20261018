"""Prepare exact path-review records and seal a completed operator review; never moves hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

from .handover import recipe_content_sha256
from .calibration import utc_stamp

HANDOVER_STAGES = ('donor_transport_stage', 'receiver_approach_stage', 'donor_retreat_stage')
CYCLE_STAGES = ('pickup_stage', 'insert_stage', 'release_stage', 'withdraw_stage')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError('JSON object required: ' + str(path))
    return value


def payload(value):
    return json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n'


def relative(path, directory):
    return os.path.relpath(Path(path).resolve(), Path(directory).resolve())


def stage_binding(config_path, config, stage):
    if not isinstance(stage, str) or not stage.strip():
        raise ValueError('Fill every referenced taught stage before preparing its review')
    row = config.get('stages', {}).get(stage)
    if not isinstance(row, dict) or not isinstance(row.get('csv'), str) or not row['csv']:
        raise ValueError('Stage CSV is missing from site configuration: ' + stage)
    path = (config_path.parent / row['csv']).resolve()
    actual = digest(path)
    if actual != row.get('sha256'):
        raise ValueError('Stage CSV changed; site stored hash does not match: ' + stage)
    return actual, path


def check_completed_review(recipe, recipe_path):
    name = recipe.get('review_file')
    if not isinstance(name, str) or not name:
        raise ValueError('Recipe must reference its review file')
    path = (recipe_path.parent / name).resolve()
    review = load(path)
    if review.get('recipe_content_sha256') != recipe_content_sha256(recipe):
        raise ValueError('Recipe changed after review preparation; prepare a new review')
    reviewer = review.get('reviewer')
    if not isinstance(reviewer, str) or not reviewer.strip():
        raise ValueError('A named operator must actually complete the review before sealing')
    stamp = review.get('at_utc')
    if not isinstance(stamp, str) or not stamp.strip() or utc_stamp(stamp) > datetime.now(timezone.utc):
        raise ValueError('Review needs a nonfuture timezone-aware at_utc timestamp')
    if review.get('full_path_reviewed') is not True or review.get('checked_actual_cell') is not True:
        raise ValueError('Actual full-path and actual-cell review must both be confirmed by the operator')
    return path, review


def collect_bindings(recipe, recipe_path, config_path, config, require_nested_seal=True):
    """Resolve local files only; no Site instance, robot, model, or PLC queries."""
    stages, files, nested = {}, {}, []
    def add(stage):
        actual, path = stage_binding(config_path, config, stage)
        stages[stage] = actual
        files[stage] = path
    if recipe.get('schema_version') != 1:
        raise ValueError('Recipe schema_version 1 required')
    if 'parts' in recipe:
        entries = recipe['parts']
        if (not isinstance(entries, list) or len(entries) != 3 or
                any(not isinstance(part, dict) for part in entries) or
                {part.get('part') for part in entries} != {'ring1', 'ring2', 'pin'}):
            raise ValueError('Cycle must contain ring1, ring2 and pin exactly once')
        for part in entries:
            for key in CYCLE_STAGES:
                add(part.get(key))
            name = part.get('handover_recipe')
            if name:
                if not isinstance(name, str):
                    raise ValueError('handover_recipe must be a file path')
                nested_path = (recipe_path.parent / name).resolve()
                if nested_path == recipe_path:
                    raise ValueError('A cycle cannot reference itself as a handover')
                transfer = load(nested_path)
                if 'parts' in transfer:
                    raise ValueError('Nested recipe must be a handover, not another cycle')
                transfer_stages, transfer_files, _ = collect_bindings(
                    transfer, nested_path, config_path, config, require_nested_seal=False)
                if transfer.get('site_config_sha256') != digest(config_path):
                    raise ValueError('Nested handover site configuration changed; prepare and seal that review first')
                if transfer.get('stage_sha256') != transfer_stages:
                    raise ValueError('Nested handover stage bindings changed; prepare and seal that review first')
                review_path, _ = check_completed_review(transfer, nested_path)
                if require_nested_seal and transfer.get('review_sha256') != digest(review_path):
                    raise ValueError('Nested handover must already have a sealed actual operator review')
                if (transfer.get('part'), transfer.get('donor_side'), transfer.get('receiver_side')) != (
                        part.get('part'), part.get('pickup_side'), part.get('insert_side')):
                    raise ValueError('Nested handover part/arms differ from the cycle entry')
                stages.update(transfer_stages)
                files.update(transfer_files)
                nested.append((part, nested_path, digest(nested_path)))
            elif part.get('pickup_side') != part.get('insert_side'):
                raise ValueError('Different pickup/insert arms require a reviewed handover recipe')
    else:
        if recipe.get('part') not in ('ring1', 'ring2', 'pin'):
            raise ValueError('A handover recipe needs a known part')
        for key in HANDOVER_STAGES:
            add(recipe.get(key))
    return stages, files, nested


def replace_recipe(path, recipe, expected_bytes):
    """Keep the exact original, then atomically replace only this requested recipe."""
    text = payload(recipe)
    if path.read_bytes() != expected_bytes:
        raise ValueError('Recipe changed during preparation; no overwrite performed')
    backup = path.with_name(path.name + '.bak-' + str(time.time_ns()))
    with backup.open('xb') as stream:
        stream.write(expected_bytes)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix='.' + path.name + '.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if path.read_bytes() != expected_bytes:
            raise ValueError('Recipe changed during preparation; original file preserved')
        os.replace(temporary, path)
    finally:
        if temporary and temporary.exists():
            temporary.unlink()
    return backup


def prepare(config_path, recipe_path, output):
    config_path, recipe_path, output = (Path(path).resolve() for path in (config_path, recipe_path, output))
    if output in (recipe_path, config_path) or output.exists():
        raise ValueError('Review output must be a NEW file separate from the site and recipe')
    original = recipe_path.read_bytes()
    config, recipe = load(config_path), load(recipe_path)
    stages, files, nested = collect_bindings(recipe, recipe_path, config_path, config)
    recipe['site_config_sha256'] = digest(config_path)
    recipe['stage_sha256'] = stages
    for entry, _, value in nested:
        entry['handover_sha256'] = value
    recipe['review_file'] = relative(output, recipe_path.parent)
    recipe['review_sha256'] = None
    review = {'schema_version': 1, 'recipe_content_sha256': recipe_content_sha256(recipe),
              'source_config_path': relative(config_path, output.parent),
              'site_config_sha256': recipe['site_config_sha256'],
              'stage_files': {stage: {'path': relative(path, output.parent), 'sha256': stages[stage]}
                              for stage, path in files.items()},
              'reviewer': None, 'at_utc': None, 'full_path_reviewed': False, 'checked_actual_cell': False,
              'notes': 'An operator must review the complete taught paths, both arms, actual hands, parts, basket and fixture. Fill reviewer and timezone-aware at_utc only after that review. Preparation does not perform or certify this review.'}
    # Serialize before creating either output so NaN or malformed content fails early.
    review_text = payload(review)
    payload(recipe)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        stream.write(review_text)
    backup = replace_recipe(recipe_path, recipe, original)
    return {'state': 'unreviewed_record_prepared', 'recipe': str(recipe_path), 'review': str(output),
            'backup': str(backup), 'stage_count': len(stages), 'hardware_ready': False,
            'next': 'Operator reviews the actual cell and full paths, fills review record, then runs review seal.'}


def seal(recipe_path, review_path):
    recipe_path, review_path = (Path(path).resolve() for path in (recipe_path, review_path))
    original = recipe_path.read_bytes()
    recipe, explicit_review = load(recipe_path), load(review_path)
    expected_review, review = check_completed_review(recipe, recipe_path)
    if expected_review != review_path or review != explicit_review:
        raise ValueError('Use the exact review file referenced by this recipe')
    name = review.get('source_config_path')
    if not isinstance(name, str) or not name:
        raise ValueError('Run review prepare first to bind the actual local site and stage files')
    config_path = (review_path.parent / name).resolve()
    config = load(config_path)
    actual_site = digest(config_path)
    if actual_site != recipe.get('site_config_sha256') or actual_site != review.get('site_config_sha256'):
        raise ValueError('Site configuration changed after preparation; prepare a new review')
    stages, files, nested = collect_bindings(recipe, recipe_path, config_path, config)
    if recipe.get('stage_sha256') != stages:
        raise ValueError('Recipe stage bindings changed after preparation')
    expected_files = {stage: {'path': relative(path, review_path.parent), 'sha256': stages[stage]}
                      for stage, path in files.items()}
    if review.get('stage_files') != expected_files:
        raise ValueError('Review stage file bindings changed after preparation')
    if any(entry.get('handover_sha256') != value for entry, _, value in nested):
        raise ValueError('Nested handover changed after preparation; prepare a new review')
    recipe['review_sha256'] = digest(review_path)
    backup = replace_recipe(recipe_path, recipe, original)
    return {'state': 'operator_review_sealed', 'recipe': str(recipe_path), 'review': str(review_path),
            'backup': str(backup), 'hardware_ready': False,
            'note': 'Records operator review only; run the recipe validation and onsite commissioning separately.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    prepare_parser = commands.add_parser('prepare', help='Write a NEW unreviewed record and back up/update exact recipe hashes')
    prepare_parser.add_argument('--config', type=Path, required=True)
    prepare_parser.add_argument('--recipe', type=Path, required=True)
    prepare_parser.add_argument('--output', type=Path, required=True)
    seal_parser = commands.add_parser('seal', help='Bind a completed actual operator review; rejects untouched templates or changed files')
    seal_parser.add_argument('--recipe', type=Path, required=True)
    seal_parser.add_argument('--review', type=Path, required=True)
    args = parser.parse_args(argv)
    result = prepare(args.config, args.recipe, args.output) if args.command == 'prepare' else seal(args.recipe, args.review)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
