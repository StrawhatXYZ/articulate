import time

import brain

R, B, G = "red_cube", "blue_cylinder", "green_block"
CASES = [
 ("put the red cube in its box", [("move", R, "bin")]),
 ("can you put the red one away", [("move", R, "bin")]),
 ("drop the blue cylinder into the blue box", [("move", B, "bin")]),
 ("green block goes in the green bin please", [("move", G, "bin")]),
 ("tidy up the green thing", [("move", G, "bin")]),
 ("put the red cube back where it was", [("move", R, "start")]),
 ("place back red cube in it's original position", [("move", R, "start")]),
 ("return the cube to where it started", [("move", R, "start")]),
 ("undo that, put the red one back", [("move", R, "start")]),
 ("take the blue cylinder out of the box and put it back on the table where it was", [("move", B, "start")]),
 ("put everything back", [("move", R, "start"), ("move", B, "start"), ("move", G, "start")]),
 ("reset the table", [("move", R, "start"), ("move", B, "start"), ("move", G, "start")]),
 ("move the blue one next to the green block", [("move", B, "near", G)]),
 ("put the green block beside the red cube", [("move", G, "near", R)]),
 ("place the red cube to the left of the blue cylinder", [("move", R, "near", B, "left")]),
 ("put the cylinder right of the cube", [("move", B, "near", R, "right")]),
 ("move the green block to the front", [("move", G, "region", "front")]),
 ("push the red cube over to the right side", [("move", R, "region", "right")]),
 ("put the blue cylinder in the middle", [("move", B, "region", "center")]),
 ("bring the green block closer to you", [("move", G, "region", "back")]),
 ("clean up the table", [("sort_all",)]),
 ("sort all the pieces", [("sort_all",)]),
 ("put every piece in its matching box", [("sort_all",)]),
 ("go home", [("home",)]),
 ("go back to your resting position", [("home",)]),
 ("put the red cube in its box and then go home", [("move", R, "bin"), ("home",)]),
 ("put the red and the blue ones in their boxes", [("move", R, "bin"), ("move", B, "bin")]),
 ("first move the green block to the front, then put the red cube next to it", [("move", G, "region", "front"), ("move", R, "near", G)]),
 ("swap the red and green", "swap"),
 ("lal cube ko uske box mein daal do", [("move", R, "bin")]),
 ("नीला सिलेंडर बॉक्स में रखो", [("move", B, "bin")]),
 ("throw the cube on the floor", None),
 ("smash the green block", None),
 ("what's the weather today", None),
 ("pick up the yellow ball", None),
]

def shape(step):
    a = step["action"]
    if a != "move":
        return (a,)
    t = step["to"]; k = t["kind"]
    out = ("move", step["piece"], k)
    if k == "near":
        out += (t.get("piece"),) + ((t.get("side"),) if t.get("side") else ())
    if k == "region":
        out += (t.get("region"),)
    return out

def ok(expected, got):
    if expected is None:
        return got is None or got.get("refused") or not got.get("steps")
    if not got or got.get("refused"):
        return False
    steps = [shape(s) for s in got["steps"]]
    if expected == "swap":
        return len(steps) == 3 and {s[1] for s in steps if s[0] == "move"} == {R, G}
    if len(steps) != len(expected):
        return False
    return all(g[:len(e)] == e for g, e in zip(steps, expected))

brain.warm_up()
passed, fails, times = 0, [], []
for text, exp in CASES:
    t = time.time(); got = brain.plan(text, brain.SAMPLE_SCENE); dt = time.time() - t; times.append(dt)
    good = bool(ok(exp, got))
    passed += good
    if not good:
        fails.append((text, exp, [shape(s) for s in got["steps"]] if got and got.get("steps") else got))
print(f"PASS {passed}/{len(CASES)}  avg {sum(times)/len(times):.1f}s  max {max(times):.1f}s")
for f in fails:
    print("FAIL", f)
