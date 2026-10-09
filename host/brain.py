import json
import math
import re
import sys
import time
import urllib.request

OLLAMA = "http://localhost:11434"
MODEL = "gemma4:e2b"
SIDES = ("left", "right", "front", "back")
REGIONS = ("left", "right", "front", "back", "center")
COLOURS = ("red", "green", "blue")
SHAPES = ("cube", "cylinder", "block")
MAX_STEPS = 8

SAMPLE_SCENE = {
    "busy": False,
    "pieces": {
        "red_cube": {"xy": [0.22, -0.02], "start_xy": [0.22, -0.02], "in_bin": False, "colour": "red", "shape": "cube"},
        "blue_cylinder": {"xy": [0.14, -0.15], "start_xy": [0.20, -0.10], "in_bin": False, "colour": "blue", "shape": "cylinder"},
        "green_block": {"xy": [0.05, -0.19], "start_xy": [0.05, -0.19], "in_bin": False, "colour": "green", "shape": "block"},
    },
    "bins": {"green": [0.209, 0.105], "red": [0.135, 0.176], "blue": [0.035, 0.2]},
}

TARGET_ARGS = {
    "type": "object",
    "properties": {
        "piece": {"type": "string", "description": "piece to move, e.g. red_cube"},
        "target": {
            "type": "string",
            "enum": ["bin", "start", "near", "region"],
            "description": "bin = its own colour box; start = where it was at the beginning; "
                           "near = next to another piece; region = an area of the table",
        },
        "other_piece": {"type": "string", "description": "only for target=near"},
        "side": {"type": "string", "enum": list(SIDES), "description": "only for target=near: which side of other_piece"},
        "region": {"type": "string", "enum": list(REGIONS), "description": "only for target=region"},
    },
    "required": ["piece", "target"],
}
TOOLS = [
    {"type": "function", "function": {
        "name": "move_piece",
        "description": "Move one piece. Call it several times, in order, for several moves.",
        "parameters": TARGET_ARGS}},
    {"type": "function", "function": {
        "name": "swap_pieces",
        "description": "Swap the places of two pieces (one call; handles the temporary spot itself).",
        "parameters": {"type": "object", "properties": {
            "piece_a": {"type": "string"}, "piece_b": {"type": "string"}},
            "required": ["piece_a", "piece_b"]}}},
    {"type": "function", "function": {
        "name": "go_home", "description": "Move the arm to its home pose.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "sort_all",
        "description": "Put every piece into the box of its own colour (clean up / tidy / sort everything).",
        "parameters": {"type": "object", "properties": {}}}},
]

SYSTEM = """You control a robot arm over a table. You never answer in words: you always call tools.
Directions from the robot's point of view: front = farther away from the robot, back = closer to the robot ("closer to you", "nearer to you" = back), left and right as the robot sees them.
Pieces: {pieces}.
Each colour has a box (bin) at the side of the table.
Tools:
- move_piece(piece, target, ...): target "bin" = into the piece's own colour box; "start" = back to where it was at the beginning (use for "put back", "return", "where it was"); "near" = next to other_piece, on a side of it (left/right/front/back); "region" = an area of the table (left/right/front/back/center), e.g. "place the green block in front" = region front.
- go_home(): the arm goes home.
- sort_all(): every piece into its own colour box ("clean up the table", "tidy everything", "sort everything"). If the request names ONE piece ("tidy up the green thing", "put the red one away"), use move_piece with target "bin" for that piece instead.
Rules:
- Use piece names exactly as listed. "the blue one", "the cube" identify a piece by colour or shape.
- "put everything back", "reset the table", "restore", "undo everything" = one move_piece with target "start" for each piece (NOT sort_all).
- Compound requests = several move_piece calls in order.
- "swap A and B" / "exchange" = one swap_pieces call.
- For target "near" always give both other_piece and side.
- If the request is destructive or unsafe (throwing, dropping on the floor, breaking, hitting) or unrelated to moving pieces, call NO tool.
Current scene:
{scene}"""


def _post(path, body, timeout):
    req = urllib.request.Request(OLLAMA + path, json.dumps(body).encode(),
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def warm_up():
    try:
        _post("/api/chat", {"model": MODEL, "messages": [{"role": "user", "content": "hi"}],
                            "stream": False, "keep_alive": "30m", "options": {"num_predict": 1}}, 180)
        return True
    except Exception:
        return False


def region_of(xy):
    dx, dy = xy[0] - 0.0388, xy[1]
    bearing, radius = math.atan2(dy, dx), math.hypot(dx, dy)
    if abs(bearing) > 0.6:
        return "left" if bearing > 0 else "right"
    if radius > 0.195:
        return "front"
    return "back" if radius < 0.165 else "center"


def scene_summary(scene):
    lines = []
    for name, p in scene.get("pieces", {}).items():
        where = "in its box" if p.get("in_bin") else f"on the table, {region_of(p['xy'])} area, xy={[round(v, 2) for v in p['xy']]}"
        moved = "" if p.get("in_bin") or _close(p["xy"], p.get("start_xy", p["xy"])) else f"; started at xy={[round(v, 2) for v in p['start_xy']]} ({region_of(p['start_xy'])} area)"
        lines.append(f"- {name} ({p.get('colour')} {p.get('shape')}): {where}{moved}")
    return "\n".join(lines)


def _close(a, b):
    return abs(a[0] - b[0]) < 0.03 and abs(a[1] - b[1]) < 0.03


def resolve_piece(word, scene):
    names = list(scene.get("pieces", {}))
    w = str(word or "").lower().replace(" ", "_").strip()
    if w in names:
        return w
    tokens = set(re.split(r"[\s_]+", str(word or "").lower()))
    best = [n for n in names if tokens & {scene["pieces"][n].get("colour"), scene["pieces"][n].get("shape")}]
    for kind in (("colour",), ("shape",)):
        hits = [n for n in names if tokens & {scene["pieces"][n].get(k) for k in kind}]
        if len(hits) == 1:
            return hits[0]
    return best[0] if len(best) == 1 else None


def swap_steps(a, b, scene):
    pa, pb = scene["pieces"][a], scene["pieces"][b]
    if a == b or pa.get("in_bin") or pb.get("in_bin"):
        return None
    used = {region_of(p["xy"]) for p in scene["pieces"].values() if not p.get("in_bin")}
    free = [r for r in ("center", "front", "back", "left", "right") if r not in used]
    if not free:
        return None
    return [
        {"action": "move", "piece": a, "to": {"kind": "region", "region": free[0]}},
        {"action": "move", "piece": b, "to": {"kind": "xy", "xy": list(pa["xy"])}},
        {"action": "move", "piece": a, "to": {"kind": "xy", "xy": list(pb["xy"])}},
    ]


def _calls_from_text(content):
    calls = []
    for name, body in re.findall(r"(move_piece|go_home|sort_all|swap_pieces)\s*\{([^}]*)\}", content or ""):
        args = dict(re.findall(r'(\w+)\s*:\s*<\|"\|>(.*?)<\|"\|>', body))
        calls.append({"function": {"name": name, "arguments": args}})
    return calls


def _side_towards(piece, other):
    dx, dy = piece["xy"][0] - other["xy"][0], piece["xy"][1] - other["xy"][1]
    if abs(dy) >= abs(dx):
        return "left" if dy > 0 else "right"
    return "front" if dx > 0 else "back"


def _step_from_call(name, args, scene):
    if name == "swap_pieces":
        a, b = resolve_piece(args.get("piece_a"), scene), resolve_piece(args.get("piece_b"), scene)
        return swap_steps(a, b, scene) if a and b else None
    if name == "go_home":
        return {"action": "home"}
    if name == "sort_all":
        return {"action": "sort_all"}
    if name != "move_piece":
        return None
    piece = resolve_piece(args.get("piece"), scene)
    if not piece:
        return None
    kind = str(args.get("target") or args.get("target_kind") or "").lower()
    if kind in REGIONS and not args.get("region"):
        args = {**args, "region": kind}
        kind = "region"
    elif not kind:
        kind = "region" if args.get("region") else "near" if args.get("other_piece") else ""
    if kind == "bin":
        to = {"kind": "bin"}
    elif kind == "start":
        to = {"kind": "start"}
    elif kind == "near":
        other = resolve_piece(args.get("other_piece"), scene)
        side = str(args.get("side") or "").lower()
        if other and not side:
            side = _side_towards(scene["pieces"][piece], scene["pieces"][other])
        if not other or other == piece or side not in SIDES:
            return None
        to = {"kind": "near", "piece": other, "side": side}
    elif kind == "region":
        region = str(args.get("region") or "").lower()
        if region not in REGIONS:
            return None
        to = {"kind": "region", "region": region}
    else:
        return None
    return {"action": "move", "piece": piece, "to": to}


def describe(plan):
    out = []
    for s in plan["steps"]:
        if s["action"] == "home":
            out.append("go home")
        elif s["action"] == "sort_all":
            out.append("sort everything into its box")
        else:
            t = s["to"]
            where = {"bin": "its box", "start": "where it started"}.get(t["kind"])
            if t["kind"] == "near":
                where = f"the {t['side']} of {t['piece']}"
            elif t["kind"] == "region":
                where = f"the {t['region']} of the table"
            elif t["kind"] == "xy":
                where = f"xy {t['xy']}"
            out.append(f"move {s['piece']} to {where}")
    return "; ".join(out) or "(nothing)"


def plan(text, scene=None, timeout=60, verbose=False):
    scene = scene or SAMPLE_SCENE
    system = SYSTEM.format(pieces=", ".join(scene.get("pieces", {})), scene=scene_summary(scene))
    body = {
        "model": MODEL, "stream": False, "keep_alive": "30m", "tools": TOOLS,
        "messages": [{"role": "system", "content": system}, {"role": "user", "content": text}],
        "options": {"temperature": 0, "num_ctx": 4096},
    }
    try:
        msg = _post("/api/chat", body, timeout).get("message", {})
    except Exception as exc:
        if verbose:
            print("[brain] error:", exc)
        return None
    if verbose:
        print("[brain] raw:", json.dumps(msg.get("tool_calls"))[:600], "|", (msg.get("content") or "")[:200])
    steps = []
    for call in msg.get("tool_calls") or _calls_from_text(msg.get("content")):
        fn = call.get("function", {})
        args = fn.get("arguments") or {}
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                args = {}
        step = _step_from_call(fn.get("name"), args, scene)
        if step is None:
            return {"text": text, "steps": [], "refused": f"invalid step {fn.get('name')}"}
        steps.extend(step if isinstance(step, list) else [step])
    if not steps or len(steps) > MAX_STEPS:
        return {"text": text, "steps": [], "refused": "no safe plan for that request"}
    return {"text": text, "steps": steps}


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    t0 = time.time()
    result = plan(" ".join(sys.argv[1:]), SAMPLE_SCENE, verbose="-v" in sys.argv)
    print(json.dumps(result, indent=1) if result else "null (no valid plan)")
    if result:
        print(describe(result))
    print(f"[{time.time() - t0:.2f} s]")
