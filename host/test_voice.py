"""Voice client logic that runs without a microphone or a model."""

import stt
import voice


def test_engine_choice():
    assert stt.choose_engine('Darwin', 'arm64') == 'mlx'
    assert stt.choose_engine('Darwin', 'x86_64') == 'faster'
    assert stt.choose_engine('Windows', 'AMD64') == 'faster'
    assert stt.choose_engine('Linux', 'x86_64') == 'faster'


def test_goal_command_passes_text_without_shell_quoting():
    text = 'put the "red" cube\nin it\'s box; rm -rf / $(x) నీలం'
    cmd = voice.goal_command(text, 'ai-robotics-lab')
    assert cmd[:2] == ['docker', 'exec']
    assert f'LAB_TEXT={text}' in cmd           # passed as one argv entry, never through a shell
    assert all(text not in part for part in cmd if not part.startswith('LAB_TEXT='))


def test_feedback_and_result_lines_are_parsed():
    assert voice.parse_line('    stage: step') == ('stage', 'step')
    assert voice.parse_line("    message: 'Done: move red_cube to its box.'") == (
        'message', 'Done: move red_cube to its box.')
    assert voice.parse_line('Goal accepted with ID: 1234') is None
    assert voice.parse_line('     text: put it back') is None


def test_empty_transcript_is_not_sent(monkeypatch):
    sent = []
    monkeypatch.setattr(voice, 'send', lambda text, args: sent.append(text))
    voice.handle('   ', voice.parser().parse_args(['--text', ' ']))
    assert sent == []


REAL_OUTPUT = '''Waiting for an action server to become available...
Sending goal:
     text: which pieces are in their boxes?

Feedback:
    stage: heard
message: which pieces are in their boxes?

Goal accepted with ID: fbb62ee49c8b42b7a9cc0edadd39be90

Feedback:
    stage: planning
message: ''

Result:
    outcome: answered
message: The red_cube is in its box.
steps: []

Goal finished with status: SUCCEEDED
'''


def test_progress_and_result_are_read_from_real_action_output():
    events = list(voice.read_output(REAL_OUTPUT.splitlines()))
    assert ('feedback', 'message', 'which pieces are in their boxes?') in events
    assert ('result', 'outcome', 'answered') in events
    assert ('result', 'message', 'The red_cube is in its box.') in events
    assert ('feedback', 'stage', 'planning') in events and len(events) == 7


def test_the_goal_conversion_keeps_every_character(tmp_path):
    import json
    import os
    import stat
    import subprocess
    import sys
    fake = tmp_path / 'ros2'                        # prints the goal argument it gets
    fake.write_text(f'#!{sys.executable}\nimport sys\nsys.stdout.write(sys.argv[-1])\n')
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    text = 'put the 🟥 cube "here"\nనీలం 𠀀'
    env = dict(os.environ, LAB_TEXT=text, PATH=f'{tmp_path}{os.pathsep}{os.environ["PATH"]}')
    goal = subprocess.run([sys.executable, '-c', voice._SEND], env=env, capture_output=True,
                          text=True, encoding='utf-8').stdout
    assert '\\ud' not in goal                     # no surrogate escapes for YAML to split
    assert json.loads(goal)['text'] == text
