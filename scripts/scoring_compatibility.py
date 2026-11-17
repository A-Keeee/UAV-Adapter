"""Qualify one reviewed evaluator-only change without changing flight history.

The hashes recorded by a flight remain complete and immutable.  This module
does not ignore scoring files: it accepts exactly the reviewed old evaluator
against exactly the delivered replacement and its new phase-classification
dependency.  Navigation, orchestration and every other recorded runtime source
must still match.  Binaries, configuration and scene checks remain in the
release audit and are not waived here.
"""
import hashlib
from pathlib import Path

from autonomy_provenance import ROOT, RUNTIME_SOURCES, runtime_source_hashes


MIGRATION_ID = 'wait_takeoff_data_enroute_scoring_v3'
EVALUATOR = 'scripts/autonomy_evaluator.py'
PHASES = 'scripts/evaluation_phases.py'
OLD_EVALUATOR_SHA256 = '00e53dbc791d33c0b60d70b2a089e77b35f6e7540bff67530765c31321b911c4'
NEW_EVALUATOR_SHA256 = '840910ab5df5217fb429cfe9309a9409c3519c96e7a86596ffed6c1bdbedd72c'
PHASES_SHA256 = '2e8fd743c296593c0f669d4f1eb0c59bc5716d78a62efdab9ba078f6cd334d18'


def delivered_scoring_hashes():
    """Read the actual delivered scoring code, including its new dependency."""
    result = {}
    for name in (EVALUATOR, PHASES):
        path = ROOT/name
        if path.is_file():
            result[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def migration_definition():
    return dict(migration_id=MIGRATION_ID,
        qualification_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_evaluator_sha256=OLD_EVALUATOR_SHA256,
        delivered_evaluator_sha256=NEW_EVALUATOR_SHA256,
        delivered_phase_helper_sha256=PHASES_SHA256,
        delivered_scoring_files_sha256=delivered_scoring_hashes(),
        scope='Only the reviewed evaluator and its phase helper; all other runtime sources must match exactly.')


def runtime_source_qualification(recorded, current=None):
    """Return explicit proof for an exact match or this fixed scoring migration.

    Missing/extra fields, unknown scoring revisions and any other runtime
    change fail closed.  ``current`` is injectable for read-only regression
    tests; actual scoring files are always checked independently on disk.
    """
    current = runtime_source_hashes() if current is None else current
    evidence = migration_definition()
    evidence.update(compatible=False, mode='rejected')
    required = set(RUNTIME_SOURCES)
    if not isinstance(recorded, dict) or not isinstance(current, dict):
        evidence['reason'] = 'Runtime source hashes are missing or malformed.'
        return evidence
    if set(recorded) != required or set(current) != required:
        evidence['reason'] = 'Runtime source names are missing or unexpected.'
        return evidence
    if any(not isinstance(value, str) or len(value) != 64 or
           any(c not in '0123456789abcdef' for c in value)
           for values in (recorded, current) for value in values.values()):
        evidence['reason'] = 'Runtime source hashes are malformed.'
        return evidence
    expected_scoring = {EVALUATOR: NEW_EVALUATOR_SHA256, PHASES: PHASES_SHA256}
    if evidence['delivered_scoring_files_sha256'] != expected_scoring or current[EVALUATOR] != NEW_EVALUATOR_SHA256:
        evidence['reason'] = 'Delivered scoring code is not the reviewed migration revision.'
        return evidence
    if recorded[EVALUATOR] not in (OLD_EVALUATOR_SHA256, NEW_EVALUATOR_SHA256):
        evidence['reason'] = 'Recorded evaluator revision is not qualified by this migration.'
        return evidence
    differences = sorted(name for name in required if recorded[name] != current[name])
    evidence['changed_recorded_runtime_sources'] = differences
    if any(name != EVALUATOR for name in differences):
        evidence['reason'] = 'A non-scoring runtime source changed.'
        return evidence
    evidence.update(compatible=True,
        mode='exact_runtime_match' if not differences else 'reviewed_scoring_only_migration',
        recorded_evaluator_sha256=recorded[EVALUATOR])
    return evidence


def runtime_sources_compatible(recorded, current=None):
    return runtime_source_qualification(recorded, current)['compatible']
