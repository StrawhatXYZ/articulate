"""The instruction matcher: keyword mode and a fake embedder."""

from lab_simulation.instruction import InstructionMatcher
import numpy as np


def test_keywords_by_colour_and_synonym():
    matcher = InstructionMatcher(None)
    assert matcher.match('put the crimson block in its box').piece == 'red_cube'
    assert matcher.match('move the navy thing').piece == 'blue_cylinder'
    assert matcher.match('pick the lime one').piece == 'green_block'


def test_keywords_by_shape_and_unknown():
    matcher = InstructionMatcher(None)
    assert matcher.match('grab the cylinder').piece == 'blue_cylinder'
    assert matcher.match('dance please').piece is None
    assert matcher.match('red and blue').piece is None


def fake_embedder(texts):
    """One axis per piece; a text points at the piece whose colour it mentions."""
    axes = {'red': 0, 'blue': 1, 'green': 2}
    rows = []
    for text in texts:
        row = np.full(3, 0.01)
        for word, axis in axes.items():
            if word in text:
                row[axis] = 1.0
        rows.append(row / np.linalg.norm(row))
    return np.array(rows)


def test_embedder_decides_when_confident():
    matcher = InstructionMatcher(fake_embedder, min_score=0.5, min_margin=0.05)
    result = matcher.match('the red thing')
    assert result.piece == 'red_cube' and result.method == 'embeddinggemma'


def test_falls_back_to_keywords_when_not_confident():
    matcher = InstructionMatcher(fake_embedder, min_score=0.5, min_margin=0.05)
    result = matcher.match('the crimson cube')
    assert result.piece == 'red_cube' and result.method == 'keyword'
    assert matcher.match('nothing useful').piece is None
