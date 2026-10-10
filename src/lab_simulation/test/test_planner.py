"""The planner's Ollama client, against a scripted fake server."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from lab_simulation.memory import BOX, PieceState, TABLE, UNKNOWN
from lab_simulation.planner import describe_scene, NUM_PREDICT, parse_calls, Planner
from lab_simulation.tasks import PiecePose

STATES = {
    'red_cube': PieceState('red_cube', BOX, PiecePose(0.135, 0.176, 0.765), (0.2, -0.08), True),
    'blue_cylinder': PieceState('blue_cylinder', TABLE, PiecePose(0.12, -0.15, 0.765),
                                (0.2, -0.10)),
    'green_block': PieceState('green_block', UNKNOWN, None, None),
}


class Fake:
    """Replies to /api/chat from a script; records requests."""

    def __init__(self, replies):
        self.replies, self.requests = list(replies), []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                body = json.dumps({'models': [{'name': 'gemma4:e2b'}]}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                length = int(self.headers['Content-Length'])
                fake.requests.append(json.loads(self.rfile.read(length)))
                reply = fake.replies.pop(0)
                if reply == 'hang':
                    threading.Event().wait(2.0)
                    return
                if reply == 'drop':                  # the server dies mid-request
                    self.close_connection = True
                    return
                body = json.dumps({'message': reply}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'


def tool(name, **args):
    return {'function': {'name': name, 'arguments': args}}


def test_scene_text_comes_from_memory_and_camera_only():
    text = describe_scene(STATES)
    assert 'red_cube (red cube): in its box (placed by me)' in text
    assert 'green_block (green block): not visible' in text
    assert 'blue_cylinder (blue cylinder): on the table' in text and 'moved from its start' in text


def test_native_tool_calls_are_parsed_and_limits_sent():
    fake = Fake([{'tool_calls': [tool('move_piece', piece='blue_cylinder', target='bin')]}])
    reply = Planner(fake.url).plan('put the blue one away', STATES)
    assert reply.error is None and reply.calls[0].name == 'move_piece'
    sent = fake.requests[0]
    assert sent['options']['num_predict'] == NUM_PREDICT and sent['options']['temperature'] == 0
    assert 'think' not in sent
    assert {t['function']['name'] for t in sent['tools']} >= {'refuse', 'answer'}


def test_text_written_calls_are_parsed():
    calls = parse_calls({'content': 'refuse{reason:<|"|>unsafe<|"|>} go_home{}'})
    assert [c.name for c in calls] == ['refuse', 'go_home']
    assert calls[0].args == {'reason': 'unsafe'}


def test_no_call_is_retried_once_then_not_understood():
    fake = Fake([{'content': 'hmm'}, {'content': 'still no'}])
    reply = Planner(fake.url).plan('blah', STATES)
    assert reply.error == 'no_call' and len(fake.requests) == 2


def test_a_timeout_is_retried_once():
    fake = Fake(['hang', {'tool_calls': [tool('go_home')]}])
    reply = Planner(fake.url, timeout_s=0.5).plan('go home', STATES)
    assert reply.error is None and reply.calls[0].name == 'go_home'


def test_server_down_is_unavailable():
    reply = Planner('http://127.0.0.1:9', timeout_s=0.5).plan('go home', STATES)
    assert reply.error == 'unavailable'


def test_check_ready_reports_a_missing_model():
    fake = Fake([])
    assert Planner(fake.url).check_ready() is None
    assert 'pull' in Planner(fake.url, model='other:1b').check_ready()


def test_a_server_dying_mid_request_is_unavailable_not_a_crash():
    fake = Fake(['drop'])
    reply = Planner(fake.url).plan('go home', STATES)
    assert reply.error == 'unavailable'


def test_malformed_replies_parse_to_nothing_rather_than_crash():
    for message in ({'tool_calls': [{'function': None}]}, {'tool_calls': [None, 'x']},
                    {'tool_calls': None, 'content': None}, {'content': 7}):
        calls = parse_calls(message)
        assert all(isinstance(c.args, dict) or c.args is not None for c in calls)
