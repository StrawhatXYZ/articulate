"""
Name objects by matching image embeddings against text embeddings.

An embedding model maps image crops and text prompts into one space where
matching pairs point the same way. Each crop gets the label whose prompt it
is most similar to (cosine similarity), or none when the match is weak or
ambiguous. New objects need only a new prompt: the vocabulary is open.

The core is pure Python and NumPy; SiglipEmbedder imports PyTorch and
transformers only when it is created (they live in /opt/lab-ai).
"""

from typing import Dict, List, Optional, Protocol, Sequence, Tuple

import numpy as np

DEFAULT_MODEL = 'google/siglip2-base-patch16-224'
EMBEDDINGGEMMA_MODEL = 'google/embeddinggemma-2'


class Embedder(Protocol):
    """Maps images and texts to unit vectors in one space."""

    def embed_images(self, crops: Sequence[np.ndarray]) -> np.ndarray:
        """Return one unit row per HxWx3 uint8 crop."""

    def embed_texts(self, texts: Sequence[str]) -> np.ndarray:
        """Return one unit row per text."""


def _unit(rows: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(rows, axis=1, keepdims=True)
    return rows / np.where(norms > 0, norms, 1.0)


class LabelMatcher:
    """Assign each crop the label of its most similar prompt, or None."""

    def __init__(self, embedder: Embedder, prompts: Dict[str, str], min_score: float,
                 min_margin: float):
        """
        Embed the prompts once.

        A crop is unknown (None) when its best similarity is below min_score,
        or within min_margin of the second best.
        """
        self.labels = list(prompts)
        self._embedder = embedder
        self._text = embedder.embed_texts([prompts[label] for label in self.labels])
        self._min_score = min_score
        self._min_margin = min_margin

    def scores(self, crops: Sequence[np.ndarray]) -> np.ndarray:
        """Return the similarity of every crop to every label, shape (crops, labels)."""
        if not crops:
            return np.zeros((0, len(self.labels)))
        return self._embedder.embed_images(crops) @ self._text.T

    def name(self, crops: Sequence[np.ndarray]) -> List[Tuple[Optional[str], float]]:
        """Return (label or None, best similarity) for each crop."""
        names = []
        for row in self.scores(crops):
            order = np.argsort(row)[::-1]
            best = float(row[order[0]])
            second = float(row[order[1]]) if len(order) > 1 else -np.inf
            confident = best >= self._min_score and best - second >= self._min_margin
            names.append((self.labels[order[0]] if confident else None, best))
        return names


class SiglipEmbedder:
    """SigLIP / SigLIP 2 image-text embeddings on the CPU (Hugging Face transformers)."""

    def __init__(self, model_id: str = DEFAULT_MODEL):
        """Load the model; downloads it into the Hugging Face cache on first use."""
        import torch
        from transformers import AutoModel, AutoProcessor
        self._torch = torch
        self._processor = AutoProcessor.from_pretrained(model_id)
        self._model = AutoModel.from_pretrained(model_id).eval()

    @staticmethod
    def _features(output):
        # transformers 5 returns a model output object; older versions a tensor.
        return getattr(output, 'pooler_output', output)

    def embed_images(self, crops: Sequence[np.ndarray]) -> np.ndarray:
        """Return one unit row per crop."""
        from PIL import Image
        with self._torch.no_grad():
            inputs = self._processor(images=[Image.fromarray(c) for c in crops],
                                     return_tensors='pt')
            features = self._features(self._model.get_image_features(**inputs))
        return _unit(features.numpy())

    def embed_texts(self, texts: Sequence[str]) -> np.ndarray:
        """Return one unit row per text."""
        with self._torch.no_grad():
            inputs = self._processor(text=list(texts), padding='max_length',
                                     return_tensors='pt')
            features = self._features(self._model.get_text_features(**inputs))
        return _unit(features.numpy())


class EmbeddingGemmaEmbedder:
    """EmbeddingGemma 2 text and image embeddings in one space (sentence-transformers, CPU)."""

    IMAGE_TEXT = '<|image|>'

    def __init__(self, model_id: str = EMBEDDINGGEMMA_MODEL, prompt_name: str = 'Document',
                 max_soft_tokens: int = 70):
        """
        Load the model in float32 (never float16); downloads it on first use.

        max_soft_tokens is the image token budget (70, 140, 280...): cost grows
        with it (about 1.3 s per crop at 70 on the CPU) and 70 already separates colours.
        The weights ship in bfloat16, which most CPUs only emulate: float32 is
        about four times faster.
        """
        import torch
        from sentence_transformers import SentenceTransformer
        self._model = SentenceTransformer(model_id, device='cpu',
                                          model_kwargs={'dtype': torch.float32})
        self._model[0].processor.image_processor.max_soft_tokens = max_soft_tokens
        self._prompt_name = prompt_name

    def embed_images(self, crops: Sequence[np.ndarray]) -> np.ndarray:
        """Return one unit row per crop."""
        from PIL import Image
        rows = [self._model.encode({'text': self.IMAGE_TEXT, 'image': Image.fromarray(c)})
                for c in crops]
        return _unit(np.asarray(rows, dtype=float))

    def embed_texts(self, texts: Sequence[str]) -> np.ndarray:
        """Return one unit row per text."""
        return _unit(np.asarray(self._model.encode(list(texts), prompt_name=self._prompt_name),
                                dtype=float))


class ColourEmbedder:
    """
    A deterministic stand-in for tests: embeds by colour alone.

    Images map to their mean colour with the grey removed; texts map to the
    colour word they contain. Not a model, just enough to exercise matching.
    """

    WORDS = {'red': (1.0, -0.5, -0.5), 'green': (-0.5, 1.0, -0.5), 'blue': (-0.5, -0.5, 1.0)}

    def embed_images(self, crops: Sequence[np.ndarray]) -> np.ndarray:
        """Return the grey-free mean colour of each crop, as unit rows."""
        means = np.array([c.reshape(-1, 3).mean(axis=0) for c in crops], dtype=float)
        return _unit(means - means.mean(axis=1, keepdims=True))

    def embed_texts(self, texts: Sequence[str]) -> np.ndarray:
        """Return the colour direction of the first colour word in each text."""
        rows = [next((v for w, v in self.WORDS.items() if w in t), (0.0, 0.0, 0.0))
                for t in texts]
        return _unit(np.array(rows, dtype=float))
