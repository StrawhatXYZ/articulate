"""Scoring detections against the true piece positions."""

from lab_simulation.perception_eval import evaluate, summarize
import pytest

TRUTH = {'red_cube': (0.20, -0.05), 'blue_cylinder': (0.12, -0.15), 'green_block': (0.15, 0.08)}


def test_perfect_detections():
    dets = [(name, xy) for name, xy in TRUTH.items()]
    e = evaluate(dets, TRUTH)
    assert e['missed'] == 0 and e['false'] == 0
    assert all(p['correct'] and p['error_m'] == 0 for p in e['pieces'].values())


def test_wrong_label_missed_and_false():
    dets = [('blue_cylinder', (0.201, -0.05)),      # red cube, misnamed
            ('green_block', (0.15, 0.085)),         # right, 5 mm off
            (None, (0.30, 0.30))]                   # nothing there
    e = evaluate(dets, TRUTH)
    assert e['pieces']['red_cube'] == {'label': 'blue_cylinder', 'correct': False,
                                       'error_m': pytest.approx(0.001)}
    assert e['pieces']['green_block']['correct']
    assert e['pieces']['green_block']['error_m'] == pytest.approx(0.005)
    assert e['pieces']['blue_cylinder'] == {'label': None, 'correct': False, 'error_m': None}
    assert e['missed'] == 1 and e['false'] == 1


def test_each_detection_matches_one_piece_at_most():
    close = {'a': (0.0, 0.0), 'b': (0.02, 0.0)}
    e = evaluate([('a', (0.001, 0.0))], close)
    assert e['pieces']['a']['label'] == 'a' and e['pieces']['b']['label'] is None
    assert e['missed'] == 1


def test_summary_over_many_observations():
    e1 = evaluate([(n, xy) for n, xy in TRUTH.items()], TRUTH)
    e2 = evaluate([('blue_cylinder', (0.20, -0.04))], TRUTH)
    s = summarize([e1, e2], latencies_s=[0.5, 0.7])
    assert s['observations'] == 2 and s['pieces'] == 6
    assert s['named_correctly'] == 3 and s['naming_accuracy'] == pytest.approx(3 / 4)
    assert s['missed'] == 2 and s['false'] == 0
    assert s['position_error_mm']['n'] == 4
    assert s['latency_ms']['mean'] == pytest.approx(600)


def test_summary_of_nothing():
    s = summarize([], latencies_s=[])
    assert s['observations'] == 0 and s['naming_accuracy'] is None
