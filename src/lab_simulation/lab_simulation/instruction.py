"""
Map a free-text instruction to one of the pieces.

Pure Python and NumPy, no ROS. An optional text embedder (EmbeddingGemma) is
tried first; a deterministic keyword match is always available as a fallback.
"""

import re
from typing import Callable, Dict, NamedTuple, Optional, Sequence, Tuple

import numpy as np

PHRASES: Dict[str, Tuple[str, ...]] = {
    'red_cube': ('the red cube', 'red block', 'crimson cube', 'scarlet square block',
                 'the small red box'),
    'blue_cylinder': ('the blue cylinder', 'blue round piece', 'navy cylinder',
                      'blue can', 'the blue tube'),
    'green_block': ('the green block', 'green rectangular block', 'lime green brick',
                    'green bar', 'the long green piece'),
}

COLOURS: Dict[str, Tuple[str, ...]] = {
    'red_cube': ('red', 'crimson', 'scarlet', 'maroon', 'ruby', 'लाल'),
    'blue_cylinder': ('blue', 'navy', 'azure', 'cyan', 'cobalt', 'नीला', 'नीले'),
    'green_block': ('green', 'lime', 'emerald', 'olive', 'हरा', 'हरे'),
}
SHAPES: Dict[str, Tuple[str, ...]] = {
    'red_cube': ('cube', 'square'),
    'blue_cylinder': ('cylinder', 'can', 'tube', 'round'),
    'green_block': ('rectangular', 'brick', 'bar', 'block'),
}


class Match(NamedTuple):
    """The piece an instruction refers to (None if not understood) and how."""

    piece: Optional[str]
    method: str
    score: float = 0.0


def keyword_match(text: str) -> Optional[str]:
    """Return the piece named by colour (preferred) or shape words, or None if unclear."""
    words = set(re.findall(r'\w+', text.lower()))
    for table in (COLOURS, SHAPES):
        hits = [piece for piece, keys in table.items() if words & set(keys)]
        if len(hits) == 1:
            return hits[0]
        if len(hits) > 1:
            return None
    return None


class InstructionMatcher:
    """Choose a piece for an instruction, by embedding similarity then keywords."""

    def __init__(self, embed_texts: Optional[Callable[[Sequence[str]], np.ndarray]] = None,
                 min_score: float = 0.5, min_margin: float = 0.05):
        self._embed = embed_texts
        self._min_score = min_score
        self._min_margin = min_margin
        self._owners = [piece for piece, phrases in PHRASES.items() for _ in phrases]
        self._phrases = None
        if embed_texts is not None:
            all_phrases = [p for phrases in PHRASES.values() for p in phrases]
            self._phrases = np.asarray(embed_texts(all_phrases), dtype=float)

    def match(self, text: str) -> Match:
        """Return the piece the text refers to, and which method decided."""
        if self._phrases is not None:
            query = np.asarray(self._embed([text]), dtype=float)[0]
            sims = self._phrases @ query
            best: Dict[str, float] = {}
            for owner, sim in zip(self._owners, sims):
                best[owner] = max(best.get(owner, -1.0), float(sim))
            ranked = sorted(best.items(), key=lambda item: item[1], reverse=True)
            (piece, score), (_, second) = ranked[0], ranked[1]
            if score >= self._min_score and score - second >= self._min_margin:
                return Match(piece, 'embeddinggemma', score)
        piece = keyword_match(text)
        return Match(piece, 'keyword' if piece else 'none')
