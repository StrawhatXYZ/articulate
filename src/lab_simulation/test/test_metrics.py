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
    assert task['cycle_time_s'] == {'mean': None, 'median': None, 'p95': None, 'n': 0}
    assert 'n/a' in metrics.render_report(summary)


def summary_with(rate_k, n=100, cycle=2.0, rtf=1.0, seed=42):
    trials = [trial(cycle=cycle) for _ in range(rate_k)] + \
        [trial(success=False, failure='timeout') for _ in range(n - rate_k)]
    return metrics.summarize(trials, {'real_time_factor': rtf, 'trials_per_minute': 10.0,
                                      'seed': seed, 'trials_per_task': n,
                                      'tasks': ['reach'], 'complete': True})


def test_compare_flags_significance_and_direction():
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(95))}
    assert rows['reach success rate']['significant'] is True
    assert rows['reach success rate']['better'] is True
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(83))}
    assert rows['reach success rate']['significant'] is False


def test_compare_lower_cycle_time_is_better_and_ratio():
    rows = {r['metric']: r
            for r in metrics.compare(summary_with(90, cycle=4.0), summary_with(90, cycle=1.0))}
    row = rows['reach cycle time mean, successes (s)']
    assert row['better'] is True
    assert row['ratio'] == pytest.approx(0.25)


def test_compare_only_uses_common_tasks():
    a = metrics.summarize([trial()], {})
    b = metrics.summarize([trial(task='pick_place')], {})
    rows = metrics.compare(a, b)
    assert all(not r['metric'].startswith(('reach', 'pick_place')) for r in rows)
    assert 'not significant' not in metrics.render_comparison(rows, 'A', 'B')


def test_statistics_state_which_trials_they_cover():
    trials = [trial(cycle=1.0), trial(cycle=3.0),
              trial(success=False, failure='placement_error', cycle=9.0, error=0.05),
              trial(success=False, failure='dropped', error=None)]
    s = metrics.summarize_task(trials)
    assert s['cycle_time_s']['n'] == 2      # successful trials only
    assert s['error_m']['n'] == 3           # trials where an error was measured
    assert s['error_m']['median'] == pytest.approx(0.005)
    report = metrics.render_report(metrics.summarize(trials, {}))
    assert 'successes' in report and 'measured' in report


def test_only_success_rates_get_a_verdict():
    rows = {r['metric']: r for r in metrics.compare(summary_with(90, cycle=4.0),
                                                    summary_with(90, cycle=1.0))}
    table = metrics.render_comparison(list(rows.values()), 'A', 'B')
    cycle_line = next(line for line in table.splitlines() if 'cycle time' in line)
    assert cycle_line.endswith('| not tested |')
    assert any('error median' in metric for metric in rows)


def test_compare_warns_when_runs_are_not_comparable():
    a = summary_with(90, seed=1)
    b = summary_with(90, seed=2)
    b['run']['complete'] = False
    warnings = metrics.comparability_warnings(a, b)
    assert any('seed' in w for w in warnings)
    assert any('incomplete' in w for w in warnings)
    assert metrics.comparability_warnings(a, summary_with(80, seed=1)) == []
    assert 'Warning' in metrics.render_comparison(metrics.compare(a, b), 'A', 'B', warnings)


def test_compare_warns_across_robots_and_grasp_modes():
    step1 = summary_with(84, seed=42)                 # predates the robot keys
    step2 = summary_with(90, seed=42)
    step2['run'].update(robot='so101', grasp_mode='latched')
    warnings = metrics.comparability_warnings(step1, step2)
    assert 'different robot: simple_arm vs so101' in warnings
    assert 'different grasp mode: suction vs latched' in warnings
    friction = summary_with(70, seed=42)
    friction['run'].update(robot='so101', grasp_mode='friction')
    assert metrics.comparability_warnings(step2, friction) == [
        'different grasp mode: latched vs friction']


def test_report_states_robot_and_grasp_mode():
    summary = metrics.summarize([trial()], {'robot': 'so101', 'grasp_mode': 'latched'})
    assert '- Robot: so101, grasp mode: latched' in metrics.render_report(summary)


def test_throughput_is_not_compared_across_different_task_sets():
    a = metrics.summarize([trial()], {'tasks': ['reach'], 'trials_per_minute': 10.0})
    b = metrics.summarize([trial(), trial(task='pick_place')],
                          {'tasks': ['reach', 'pick_place'], 'trials_per_minute': 3.0})
    rows = [r['metric'] for r in metrics.compare(a, b)]
    assert 'trials per minute' not in rows
    assert any('task set' in w for w in metrics.comparability_warnings(a, b))
