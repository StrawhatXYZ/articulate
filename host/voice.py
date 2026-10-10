"""
Talk to the robot: speech (or typed text) in, the robot's progress out.

Runs on the host (macOS or Windows), never in the container. Push to talk:
press Enter, speak, press Enter. The transcript is sent as a goal to the
/lab/instruct action in the lab container, and the robot's progress and
result are printed as they arrive.

  python host/voice.py                    # speak (Whisper picks the engine)
  python host/voice.py --text "put the red cube in its box"
  python host/voice.py --file request.wav
"""

import argparse
from datetime import datetime
import subprocess
import sys
from typing import Iterable, Iterator, List, Optional, Tuple

import stt

# Runs in the container: builds the goal from the LAB_TEXT environment entry,
# so the text never passes through a shell (quotes, newlines and any script
# arrive unchanged). JSON is valid YAML for ros2 action send_goal; characters
# stay raw (ensure_ascii=False), since YAML would split an escaped emoji's
# surrogate pair into two invalid halves.
_SEND = ('import json, os, subprocess, sys; '
         'sys.exit(subprocess.call(["ros2", "action", "send_goal", "--feedback", '
         '"/lab/instruct", "lab_interfaces/action/Instruct", '
         'json.dumps({"text": os.environ["LAB_TEXT"]}, ensure_ascii=False)]))')
_SETUP = 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash'


def log(*parts) -> None:
    """Print a line with a timestamp."""
    print(f'[{datetime.now():%H:%M:%S}]', *parts, flush=True)


def goal_command(text: str, container: str) -> List[str]:
    """Return the docker command that sends text as an Instruct goal (no shell quoting)."""
    return ['docker', 'exec', '-e', f'LAB_TEXT={text}', container, 'bash', '-c',
            f"{_SETUP} && python3 -c '{_SEND}'"]


FIELDS = ('stage', 'message', 'outcome', 'steps')


def parse_line(line: str) -> Optional[Tuple[str, str]]:
    """Return (field, value) for a 'field: value' line of the action output."""
    key, sep, value = line.strip().partition(':')
    if not sep or key not in FIELDS:
        return None
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in '\'"':
        value = value[1:-1]
    return key, value


def read_output(lines: Iterable[str]) -> Iterator[Tuple[str, str, str]]:
    """
    Yield (section, field, value) from ros2 action send_goal output.

    The CLI indents only the first field of each message, so fields are
    read in the Feedback and Result sections whatever their indentation;
    other lines are yielded as ('note', '', line) when worth showing.
    """
    section = None
    for raw in lines:
        line = raw.rstrip('\n')
        if line.startswith(('Feedback:', 'Result:')):
            section = line[:-1].lower()
            continue
        if not line.strip() or line.startswith(('Goal accepted', 'Goal finished', 'Waiting',
                                                'Sending goal')):
            section = None if line.startswith('Goal') else section
            continue
        field = parse_line(line) if section else None
        if field:
            yield (section, *field)
        elif 'rejected' in line:
            yield ('note', '', 'busy: another instruction is running')
        elif 'rror' in line or 'No such container' in line:
            yield ('note', '', line)


def send(text: str, args) -> int:
    """Send text to the robot and print its progress; return the exit code."""
    process = subprocess.Popen(goal_command(text, args.container), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                               errors='replace')
    for section, field, value in read_output(process.stdout):
        if section == 'note':
            log('[robot]', value)
        elif section == 'feedback' and field == 'message' and value:
            log('[robot]', value)
        elif section == 'result' and field in ('outcome', 'message'):
            log(f'[robot] {field}:', value)
    return process.wait()


def handle(text: str, args) -> None:
    """Send one transcript, unless nothing was heard."""
    text = text.strip()
    if not text:
        log('nothing heard')
        return
    log(f'you: "{text}"')
    send(text, args)


def record():
    """Record from the microphone between two presses of Enter (16 kHz mono)."""
    import numpy as np
    import sounddevice as sd

    chunks = []
    input('Press Enter, then speak ...')
    with sd.InputStream(samplerate=stt.SAMPLE_RATE, channels=1, dtype='float32',
                        callback=lambda data, *_: chunks.append(data.copy())):
        input('Listening ... press Enter to stop ')
    return np.concatenate(chunks)[:, 0] if chunks else np.zeros(0, dtype='float32')


def parser() -> argparse.ArgumentParser:
    """Return the command line parser."""
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--text', help='send this text instead of speaking')
    p.add_argument('--file', help='transcribe this audio file and send it')
    p.add_argument('--model', default='small',
                   help='Whisper model: small (default, fastest) or turbo (more accurate)')
    p.add_argument('--language', help='language spoken, e.g. en, hi, te (default: detect);'
                                      ' fixing it avoids mishearing')
    p.add_argument('--container', default='articulate', help='lab container name')
    return p


def main(argv=None) -> int:
    """Run the voice client."""
    args = parser().parse_args(argv)
    if args.text is not None:
        handle(args.text, args)
        return 0
    engine = stt.choose_engine()
    log(f'loading Whisper ({engine}, {args.model}) ...')
    transcriber = stt.Transcriber(engine, args.model, args.language)
    if args.file:
        handle(transcriber.transcribe(args.file), args)
        return 0
    while True:
        try:
            audio = record()
        except (EOFError, KeyboardInterrupt):
            return 0
        started = datetime.now()
        text = transcriber.transcribe(audio) if audio.size else ''
        log(f'[whisper] {(datetime.now() - started).total_seconds():.1f} s')
        handle(text, args)


if __name__ == '__main__':
    sys.exit(main())
