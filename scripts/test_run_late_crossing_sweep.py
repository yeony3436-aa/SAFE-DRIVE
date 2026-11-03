"""Check late-crossing matrix inputs and independent variants."""

import pytest

import run_late_crossing_sweep
from run_late_crossing_sweep import crossing_variant, positive_values


def test_positive_values_reject_duplicate_and_nonfinite_inputs():
    assert positive_values([1.2, 1.6], 'speeds') == [1.2, 1.6]
    for values in ([1.2, 1.2], [float('nan')], [0], []):
        with pytest.raises(ValueError):
            positive_values(values, 'speeds')


def test_crossing_variant_preserves_template():
    template = {'signal_crossing': {'ros__parameters': {
        'late_trigger_distance': 3.0, 'pedestrian_speed': 1.0}}}
    result = crossing_variant(template, 4.0)
    assert result['signal_crossing']['ros__parameters'][
        'late_trigger_distance'] == 4.0
    assert template['signal_crossing']['ros__parameters'][
        'late_trigger_distance'] == 3.0


def test_interrupt_stops_matrix_and_keeps_partial_summary(
        tmp_path, monkeypatch):
    calls = []

    def interrupted_run(name, case, output, timeout, gui=False):
        calls.append((name, gui))
        (output / name).mkdir()
        return {'passed': False, 'interrupted': True,
                'error': 'interrupted by user', 'results': {}}

    monkeypatch.setattr(run_late_crossing_sweep, 'run_case', interrupted_run)
    monkeypatch.setattr('sys.argv', [
        'run_late_crossing_sweep.py', '--output-dir', str(tmp_path / 'out'),
        '--gui'])
    assert run_late_crossing_sweep.main() == 130
    assert len(calls) == 1
    assert calls[0][1] is True
    import json
    summary = json.loads((tmp_path / 'out' / 'summary.json').read_text())
    assert summary['interrupted'] is True
    assert len(summary['runs']) == 1
