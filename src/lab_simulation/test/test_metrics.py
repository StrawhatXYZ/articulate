"""Tests for statistics, summaries, reports and comparisons."""

from lab_simulation import metrics
import pytest


def trial(task='reach', success=True, failure=None, cycle=1.0, error=0.005, disturbed=False):
    return {'task': task, 'index': 0, 'success': success, 'failure': failure,
            'cycle_time_s': cycle, 'error_m': error, 'disturbed': disturbed, 'details': {}}


@pytest.mark.parametrize('k,n,expected', [
    (83, 100, (0.7445, 0.8911)),
    (50, 100, (0.4038, 0.5962)),
    (0, 10, (0.0, 0.2775)),
    (10, 10, (0.7225, 1.0)),
])
def test_wilson_interval(k, n, expected):
    assert metrics.wilson_interval(k, n) == pytest.approx(expected, abs=1e-4)


def test_wilson_interval_with_no_trials():
    assert metrics.wilson_interval(0, 0) == (0.0, 1.0)


def test_percentile():
    assert metrics.percentile([1, 2, 3, 4], 95) == pytest.approx(3.85)
    assert metrics.percentile([1, 2, 3, 4], 50) == pytest.approx(2.5)
    assert metrics.percentile([7], 95) == 7
    assert metrics.percentile([], 95) is None


def test_summarize_task_counts_and_stats():
    trials = [trial(cycle=1.0), trial(cycle=3.0),
              trial(success=False, failure='timeout', cycle=9.0, error=0.05),
              trial(success=False, failure='timeout', error=None, disturbed=True)]
    s = metrics.summarize_task(trials)
    assert (s['trials'], s['successes'], s['success_rate']) == (4, 2, 0.5)
    assert s['cycle_time_s']['mean'] == pytest.approx(2.0)  # successful trials only
    assert s['error_m']['mean'] == pytest.approx((0.005 + 0.005 + 0.05) / 3)
    assert s['failures'] == {'timeout': 2}
    assert s['disturbed'] == 1


def test_summary_with_no_successes_renders():
    summary = metrics.summarize([trial(success=False, failure='grasp_miss', error=None)],
                                {'seed': 1})
    task = summary['tasks']['reach']
    assert task['cycle_time_s'] == {'mean': None, 'median': None, 'p95': None}
    assert 'n/a' in metrics.render_report(summary)


def summary_with(rate_k, n=100, cycle=2.0, rtf=1.0):
    trials = [trial(cycle=cycle) for _ in range(rate_k)] + \
        [trial(success=False, failure='timeout') for _ in range(n - rate_k)]
    return metrics.summarize(trials, {'real_time_factor': rtf, 'trials_per_minute': 10.0})


def test_compare_flags_significance_and_direction():
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(95))}
    assert rows['reach success rate']['significant'] is True
    assert rows['reach success rate']['better'] is True
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(83))}
    assert rows['reach success rate']['significant'] is False


def test_compare_lower_cycle_time_is_better_and_ratio():
    rows = {r['metric']: r
            for r in metrics.compare(summary_with(90, cycle=4.0), summary_with(90, cycle=1.0))}
    row = rows['reach cycle time mean (s)']
    assert row['better'] is True
    assert row['ratio'] == pytest.approx(0.25)


def test_compare_only_uses_common_tasks():
    a = metrics.summarize([trial()], {})
    b = metrics.summarize([trial(task='pick_place')], {})
    rows = metrics.compare(a, b)
    assert all(not r['metric'].startswith(('reach', 'pick_place')) for r in rows)
    assert 'not significant' not in metrics.render_comparison(rows, 'A', 'B')
