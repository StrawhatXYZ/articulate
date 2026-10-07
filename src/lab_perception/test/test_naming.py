"""Naming objects by matching image and text embeddings."""

from conftest import box, cylinder, render
from lab_perception.naming import ColourEmbedder, LabelMatcher
import numpy as np
import pytest

PROMPTS = {'red_cube': 'a photo of a red cube', 'blue_cylinder': 'a photo of a blue cylinder',
           'green_block': 'a photo of a green rectangular block'}


def patch(rgb, size=24):
    return np.full((size, size, 3), rgb, dtype=np.uint8)


class CountingEmbedder(ColourEmbedder):
    def __init__(self):
        self.text_calls = 0

    def embed_texts(self, texts):
        self.text_calls += 1
        return super().embed_texts(texts)


def matcher(embedder=None, min_score=0.5, min_margin=0.1):
    return LabelMatcher(embedder or ColourEmbedder(), PROMPTS, min_score, min_margin)


def test_names_coloured_crops():
    names = matcher().name([patch((200, 30, 30)), patch((30, 30, 200)), patch((30, 180, 50))])
    assert [label for label, _ in names] == ['red_cube', 'blue_cylinder', 'green_block']
    assert all(score > 0.5 for _, score in names)


def test_grey_crop_is_unknown():
    [(label, score)] = matcher().name([patch((120, 120, 120))])
    assert label is None


def test_ambiguous_crop_is_unknown():
    # Magenta is as red as it is blue.
    [(label, _)] = matcher(min_score=0.0).name([patch((200, 30, 200))])
    assert label is None


def test_no_crops_no_names():
    assert matcher().name([]) == []


def test_prompts_are_embedded_once():
    embedder = CountingEmbedder()
    m = matcher(embedder)
    m.name([patch((200, 30, 30))])
    m.name([patch((30, 30, 200))])
    assert embedder.text_calls == 1


def test_labels_are_the_prompt_keys():
    assert matcher().labels == list(PROMPTS)


def _cached(model_id):
    try:
        from huggingface_hub import try_to_load_from_cache
    except ImportError:
        return False
    return isinstance(try_to_load_from_cache(model_id, 'config.json'), str)


@pytest.mark.skipif(not _cached('google/siglip2-base-patch16-224'),
                    reason='SigLIP 2 weights are not downloaded')
def test_siglip_names_rendered_pieces(camera):
    from lab_perception.naming import SiglipEmbedder
    from lab_perception.segmentation import find_blobs
    scene = [box(0.20, -0.05, (0.03, 0.03, 0.03), colour=(200, 30, 30)),
             cylinder(0.12, -0.15, 0.015, 0.05, colour=(30, 30, 200)),
             box(0.15, 0.08, (0.05, 0.03, 0.03), yaw=-0.6, colour=(30, 180, 50))]
    rgb, depth = render(camera, scene)
    blobs = find_blobs(depth, camera, 0.75, (0.0388353, 0.0))
    crops = [rgb[v0:v1 + 1, u0:u1 + 1] for u0, v0, u1, v1 in (b.bbox for b in blobs)]
    names = LabelMatcher(SiglipEmbedder(), PROMPTS, -1.0, 0.0).name(crops)
    found = {label for label, _ in names}
    assert found == set(PROMPTS)
