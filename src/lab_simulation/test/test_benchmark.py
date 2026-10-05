"""Tests for the benchmark runner's simulation-independent parts."""

from datetime import datetime
import json

from lab_simulation import benchmark
from lab_simulation.tasks import SimulationUnresponsive, TrialResult


class CountingTask:
    """Succeeds on even indices; optionally raises SimulationUnresponsive at one index."""

    def __init__(self, name, hang_at=None):
        self.name = name
        self.hang_at = hang_at
        self.seen = []

    def run(self, rng, index):
        if index == self.hang_at:
            raise SimulationUnresponsive('no joint states for 5 s')
        self.seen.append(rng.random())
        ok = index % 2 == 0
        return TrialResult(self.name, index, ok, None if ok else 'timeout', 1.0, 0.001)


def test_execute_writes_every_trial(tmp_path):
    path = tmp_path / 'trials.jsonl'
    trials, complete = benchmark.execute([CountingTask('reach')], 4, 42, lambda rng: True, path)
    assert complete
    assert [t['index'] for t in trials] == [0, 1, 2, 3]
    assert [json.loads(line)['index'] for line in path.read_text().splitlines()] == [0, 1, 2, 3]


def test_execute_is_seeded_per_task(tmp_path):
    alone = CountingTask('pick_place')
    benchmark.execute([alone], 3, 7, lambda rng: True, tmp_path / 'a.jsonl')
    together = CountingTask('pick_place')
    benchmark.execute([CountingTask('reach'), together], 3, 7, lambda rng: True,
                      tmp_path / 'b.jsonl')
    assert alone.seen == together.seen


def test_failed_reset_records_timeout(tmp_path):
    trials, complete = benchmark.execute([CountingTask('reach')], 2, 1, lambda rng: False,
                                         tmp_path / 't.jsonl')
    assert complete
    assert all(t['failure'] == 'timeout' and t['details'] == {'stage': 'reset'} for t in trials)


def test_unresponsive_simulation_stops_and_keeps_partial_results(tmp_path):
    path = tmp_path / 'trials.jsonl'
    trials, complete = benchmark.execute([CountingTask('reach', hang_at=2)], 5, 1,
                                         lambda rng: True, path)
    assert not complete
    assert len(trials) == 2
    assert len(path.read_text().splitlines()) == 2


def test_make_run_dir_never_reuses_a_directory(tmp_path):
    now = datetime(2026, 10, 5, 12, 0, 0)
    first = benchmark.make_run_dir(tmp_path / 'results', now)
    second = benchmark.make_run_dir(tmp_path / 'results', now)
    assert first.name == '20261005-120000'
    assert second.name == '20261005-120000-1'
    assert first.is_dir() and second.is_dir()


def test_write_outputs_and_compare(tmp_path):
    trials = [TrialResult('reach', i, True, None, 2.0, 0.004).to_dict() for i in range(10)]
    a = tmp_path / 'a'
    b = tmp_path / 'b'
    a.mkdir()
    b.mkdir()
    benchmark.write_outputs(a, trials, {'real_time_factor': 1.0, 'trials_per_minute': 5.0})
    faster = [dict(t, cycle_time_s=1.0) for t in trials]
    benchmark.write_outputs(b, faster, {'real_time_factor': 4.0, 'trials_per_minute': 20.0})
    assert (a / 'report.md').read_text().startswith('# Benchmark report')
    table = benchmark.compare_dirs(a, b)
    assert '| reach cycle time mean, successes (s) | 2.000 | 1.000 |' in table
    assert '4.00×' in table


def test_main_compare(tmp_path, capsys):
    trials = [TrialResult('reach', 0, True, None, 2.0, 0.004).to_dict()]
    for name in ('a', 'b'):
        (tmp_path / name).mkdir()
        benchmark.write_outputs(tmp_path / name, trials, {})
    assert benchmark.main(['compare', str(tmp_path / 'a'), str(tmp_path / 'b')]) == 0
    assert '| Metric | a | b |' in capsys.readouterr().out


def test_run_is_uncapped_by_default_and_realtime_on_request():
    parser = benchmark.build_parser()
    assert parser.parse_args(['run']).realtime is False
    assert parser.parse_args(['run', '--realtime']).realtime is True
    assert parser.parse_args(['run', '--realtime', 'false']).realtime is False
