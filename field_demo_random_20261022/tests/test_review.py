"""Synthetic operator-review binding tests only; no actual operator review or hardware."""
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from field_demo import review


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config_path = self.root / 'site.json'
        self.config = {'schema_version': 1, 'stages': {}}
        self.transfer = {'schema_version': 1, 'part': 'ring2', 'donor_side': 'right', 'receiver_side': 'left',
                         'donor_transport_stage': 'right.approach.transfer',
                         'receiver_approach_stage': 'left.approach.receive',
                         'donor_retreat_stage': 'right.approach.retreat', 'stage_sha256': {},
                         'review_file': None, 'review_sha256': None, 'site_config_sha256': None}
        for stage in (self.transfer[key] for key in review.HANDOVER_STAGES):
            self.add_stage(stage)
        self.recipe_path = self.root / 'handover.json'
        self.review_path = self.root / 'records/handover-review.json'
        self.write(self.config_path, self.config)
        self.write(self.recipe_path, self.transfer)

    def add_stage(self, stage):
        path = self.root / (stage + '.csv')
        path.write_text('time,q1\n0,0\n1,0\n')
        self.config['stages'][stage] = {'csv': path.name, 'sha256': review.digest(path)}
        return path

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2) + '\n')

    def completed_record(self, path=None):
        path = path or self.review_path
        value = review.load(path)
        value.update(reviewer='synthetic offline test fixture', at_utc='2026-01-01T00:00:00+00:00',
                     full_path_reviewed=True, checked_actual_cell=True)
        self.write(path, value)
        return value

    def test_prepare_writes_false_flags_exact_hashes_and_original_backup_without_commands(self):
        original = self.recipe_path.read_bytes()
        with patch.object(review, 'collect_bindings', wraps=review.collect_bindings), \
                patch('subprocess.run', side_effect=AssertionError('Review helper cannot execute commands')):
            result = review.prepare(self.config_path, self.recipe_path, self.review_path)
        recipe = review.load(self.recipe_path)
        record = review.load(self.review_path)
        self.assertEqual(recipe['site_config_sha256'], review.digest(self.config_path))
        self.assertEqual(recipe['stage_sha256'], {name: row['sha256'] for name, row in self.config['stages'].items()})
        self.assertEqual(record['recipe_content_sha256'], review.recipe_content_sha256(recipe))
        self.assertEqual((self.recipe_path.parent / recipe['review_file']).resolve(), self.review_path)
        self.assertIsNone(recipe['review_sha256'])
        self.assertIsNone(record['reviewer'])
        self.assertIsNone(record['at_utc'])
        self.assertFalse(record['full_path_reviewed'])
        self.assertFalse(record['checked_actual_cell'])
        self.assertEqual(Path(result['backup']).read_bytes(), original)
        self.assertFalse(result['hardware_ready'])

    def test_untouched_review_and_blank_reviewer_cannot_seal(self):
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        with self.assertRaises(ValueError):
            review.seal(self.recipe_path, self.review_path)
        value = self.completed_record()
        value['reviewer'] = '  '
        self.write(self.review_path, value)
        with self.assertRaisesRegex(ValueError, 'named operator'):
            review.seal(self.recipe_path, self.review_path)
        self.assertIsNone(review.load(self.recipe_path)['review_sha256'])

    def test_explicit_completed_test_review_seals_exact_record_without_certifying_hardware(self):
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        result = review.seal(self.recipe_path, self.review_path)
        self.assertEqual(review.load(self.recipe_path)['review_sha256'], review.digest(self.review_path))
        self.assertFalse(result['hardware_ready'])
        self.assertTrue(Path(result['backup']).is_file())

    def test_recipe_edit_after_preparation_requires_new_review(self):
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        value = review.load(self.recipe_path)
        value['notes'] = 'recipe edited after reviewed digest'
        self.write(self.recipe_path, value)
        with self.assertRaisesRegex(ValueError, 'Recipe changed'):
            review.seal(self.recipe_path, self.review_path)

    def test_csv_mutation_is_rejected_on_prepare_and_seal(self):
        stage = next(iter(self.config['stages'].values()))
        path = self.root / stage['csv']
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        path.write_text(path.read_text() + '2,1\n')
        with self.assertRaisesRegex(ValueError, 'Stage CSV changed'):
            review.seal(self.recipe_path, self.review_path)
        with self.assertRaisesRegex(ValueError, 'Stage CSV changed'):
            review.prepare(self.config_path, self.recipe_path, self.root / 'other-review.json')
        self.assertFalse((self.root / 'other-review.json').exists())

    def test_site_mutation_after_preparation_rejected(self):
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        self.config['other_setting'] = 'changed'
        self.write(self.config_path, self.config)
        with self.assertRaisesRegex(ValueError, 'Site configuration changed'):
            review.seal(self.recipe_path, self.review_path)

    def test_future_or_naive_timestamp_rejected(self):
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        value = self.completed_record()
        for timestamp in ('2026-01-01T00:00:00', (datetime.now(timezone.utc) + timedelta(days=1)).isoformat()):
            value['at_utc'] = timestamp
            self.write(self.review_path, value)
            with self.assertRaises(ValueError):
                review.seal(self.recipe_path, self.review_path)

    def test_existing_review_is_never_overwritten(self):
        self.review_path.parent.mkdir()
        self.review_path.write_text('operator existing record')
        original = self.recipe_path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'NEW file'):
            review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.assertEqual(self.recipe_path.read_bytes(), original)
        self.assertEqual(self.review_path.read_text(), 'operator existing record')

    def cycle_recipe(self):
        cycle = {'schema_version': 1, 'parts': [], 'review_file': None, 'review_sha256': None}
        for part, pickup, insert in [('ring1', 'right', 'right'), ('ring2', 'right', 'left'), ('pin', 'left', 'left')]:
            entry = {'part': part, 'pickup_side': pickup, 'insert_side': insert, 'handover_recipe': None}
            for key, side, action in [('pickup_stage', pickup, 'grasp'), ('insert_stage', insert, 'insert'),
                                      ('release_stage', insert, 'release'), ('withdraw_stage', insert, 'approach')]:
                entry[key] = f'{side}.{action}.{part}'
                self.add_stage(entry[key])
            if part == 'ring2':
                entry['handover_recipe'] = self.recipe_path.name
            cycle['parts'].append(entry)
        self.write(self.config_path, self.config)
        return cycle

    def test_cycle_aggregates_sealed_nested_handover_and_does_not_edit_it(self):
        cycle = self.cycle_recipe()
        cycle_path, cycle_review = self.root / 'cycle.json', self.root / 'records/cycle-review.json'
        self.write(cycle_path, cycle)
        # Preparing a cycle cannot silently approve its nested transfer.
        with self.assertRaises(ValueError):
            review.prepare(self.config_path, cycle_path, cycle_review)
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        review.seal(self.recipe_path, self.review_path)
        nested_before = self.recipe_path.read_bytes()
        review.prepare(self.config_path, cycle_path, cycle_review)
        prepared = review.load(cycle_path)
        self.assertEqual(len(prepared['stage_sha256']), 15)
        self.assertEqual(prepared['parts'][1]['handover_sha256'], review.digest(self.recipe_path))
        self.assertEqual(self.recipe_path.read_bytes(), nested_before)
        self.completed_record(cycle_review)
        review.seal(cycle_path, cycle_review)
        self.assertEqual(review.load(cycle_path)['review_sha256'], review.digest(cycle_review))

    def test_nested_handover_edit_after_cycle_prepare_rejected(self):
        cycle = self.cycle_recipe()
        cycle_path, cycle_review = self.root / 'cycle.json', self.root / 'records/cycle-review.json'
        self.write(cycle_path, cycle)
        review.prepare(self.config_path, self.recipe_path, self.review_path)
        self.completed_record()
        review.seal(self.recipe_path, self.review_path)
        review.prepare(self.config_path, cycle_path, cycle_review)
        self.completed_record(cycle_review)
        # Even whitespace changes the nested file digest bound by the cycle.
        self.recipe_path.write_text(self.recipe_path.read_text() + '\n')
        with self.assertRaisesRegex(ValueError, 'Nested handover changed'):
            review.seal(cycle_path, cycle_review)


if __name__ == '__main__':
    unittest.main()
