"""Speech to text with the fastest engine for this machine."""

import platform

MLX_MODELS = {'small': 'mlx-community/whisper-small-mlx',
              'turbo': 'mlx-community/whisper-large-v3-turbo'}
SAMPLE_RATE = 16000


def choose_engine(system: str = platform.system(), machine: str = platform.machine()) -> str:
    """Return 'mlx' on Apple Silicon (Metal GPU), else 'faster' (faster-whisper)."""
    return 'mlx' if system == 'Darwin' and machine == 'arm64' else 'faster'


class Transcriber:
    """Whisper speech recognition; model 'small' (default) or 'turbo'."""

    def __init__(self, engine: str, model: str = 'small'):
        self.engine = engine
        if engine == 'mlx':
            import mlx_whisper
            self._mlx, self._repo = mlx_whisper, MLX_MODELS.get(model, model)
        else:
            from faster_whisper import WhisperModel
            try:
                self._fw = WhisperModel(model, device='cuda', compute_type='float16')
            except Exception:                    # noqa: BLE001  no CUDA: CPU
                self._fw = WhisperModel(model, device='cpu', compute_type='int8')

    def transcribe(self, audio) -> str:
        """Return the text of audio (float32 16 kHz mono array, or a file path)."""
        if self.engine == 'mlx':
            return self._mlx.transcribe(audio, path_or_hf_repo=self._repo)['text'].strip()
        segments, _ = self._fw.transcribe(audio, vad_filter=True)
        return ' '.join(s.text.strip() for s in segments).strip()
