import argparse
import json
import subprocess
import sys
import threading
import time
from datetime import datetime

CONTAINER = "ai-robotics-lab"
INSTRUCTION_TOPIC = "/lab/instruction"
STATUS_TOPIC = "/lab/instruction/status"
PLAN_TOPIC = "/lab/plan"
SCENE_TOPIC = "/lab/scene"
SAMPLE_RATE = 16000
ROS_SETUP = "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash"


def add_cuda_dlls():
    import importlib.util
    import os
    from pathlib import Path

    spec = importlib.util.find_spec("nvidia")
    if spec is None or not hasattr(os, "add_dll_directory"):
        return
    for root in spec.submodule_search_locations:
        for bin_dir in Path(root).glob("*/bin"):
            os.add_dll_directory(str(bin_dir))
            os.environ["PATH"] = f"{bin_dir}{os.pathsep}{os.environ['PATH']}"


def load_model(name):
    add_cuda_dlls()
    from faster_whisper import WhisperModel

    try:
        model = WhisperModel(name, device="cuda", compute_type="float16")
        import numpy as np

        list(model.transcribe(np.zeros(SAMPLE_RATE, dtype="float32"))[0])
        return model, "cuda/float16"
    except Exception as exc:
        log(f"[whisper] CUDA unavailable ({str(exc).splitlines()[0][:80]}); using CPU")
        return WhisperModel(name, device="cpu", compute_type="int8"), "cpu/int8"


def transcribe(model, audio, language=None):
    segments, info = model.transcribe(audio, language=language, vad_filter=True)
    return " ".join(s.text.strip() for s in segments).strip(), info.language


def record(seconds=None):
    import numpy as np
    import sounddevice as sd

    if seconds:
        log(f"Recording {seconds} s ...")
        data = sd.rec(int(seconds * SAMPLE_RATE), SAMPLE_RATE, channels=1, dtype="float32")
        sd.wait()
        return data[:, 0]
    chunks = []

    def callback(indata, frames, t, status):
        chunks.append(indata.copy())

    input("Press Enter to start recording ...")
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32", callback=callback):
        input("Recording... press Enter to stop ")
    return np.concatenate(chunks)[:, 0] if chunks else np.zeros(0, dtype="float32")


def docker_ros(command):
    return ["docker", "exec", CONTAINER, "bash", "-c", f"{ROS_SETUP} && {command}"]


def send(text, topic=INSTRUCTION_TOPIC):
    msg = "{data: %s}" % json.dumps(text)
    quoted = "'" + msg.replace("'", "'\\''") + "'"
    cmd = f"ros2 topic pub --once {topic} std_msgs/msg/String {quoted}"
    result = subprocess.run(docker_ros(cmd), capture_output=True, text=True)
    if result.returncode != 0:
        log("Send failed:", result.stderr.strip() or result.stdout.strip())
    return result.returncode == 0


def get_scene(timeout=10):
    cmd = f"ros2 topic echo --once {SCENE_TOPIC} --field data --full-length"
    try:
        r = subprocess.run(docker_ros(cmd), capture_output=True, text=True, timeout=timeout)
        out = r.stdout.strip()
        if out.endswith("---"):
            out = out[:-3].strip()
        if out[:1] in "'\"" and out[-1:] == out[:1]:
            import yaml
            out = yaml.safe_load(out)
        return json.loads(out)
    except Exception:
        return None


FINAL_STATUS = ("plan done", "done:", "failed", "refused", "did not understand", "busy")


def log(*parts):
    print(f"[{datetime.now():%H:%M:%S}] " + " ".join(str(p) for p in parts), flush=True)


def watch(seconds=180, plan_steps=0):
    log(f"[robot] watching {STATUS_TOPIC} (Ctrl+C to stop) ...")
    cmd = f"timeout {seconds} ros2 topic echo {STATUS_TOPIC} --field data"
    proc = subprocess.Popen(docker_ros(cmd), stdout=subprocess.PIPE, text=True,
                            encoding="utf-8", errors="replace")
    try:
        parts = []
        for raw in proc.stdout:
            if raw.strip() != "---":
                parts.append(raw.strip())
                continue
            line, parts = " ".join(p for p in parts if p), []
            if not line:
                continue
            log(f"[robot] {line}")
            if line.startswith("done: step") and plan_steps:
                continue
            if line.startswith(FINAL_STATUS):
                break
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()


def handle(text, args):
    log(f'Command: "{text}"')
    if not text:
        log("Nothing heard.")
        return
    plan = None
    if not args.no_brain:
        import brain

        scene = get_scene() if not args.dry_run else None
        if scene is None:
            log("[brain] no live scene (robot not reachable); using the sample scene")
            scene = brain.SAMPLE_SCENE
        t = time.time()
        plan = brain.plan(text, scene)
        log(f"[brain] {time.time() - t:.2f} s")
        if plan is None:
            log("[brain] not reachable; " + ("not sending." if args.dry_run else "falling back to /lab/instruction"))
        elif plan.get("refused"):
            log(f"[brain] refused: {plan['refused']} (nothing sent)")
            return
        else:
            log("Plan: " + brain.describe(plan))
            log(json.dumps(plan))
    if args.dry_run:
        log("(dry run, not sent)")
    elif plan is not None:
        ok = send(json.dumps(plan), PLAN_TOPIC)
        if ok and args.watch:
            watch(plan_steps=len(plan["steps"]))
    elif send(text) and args.watch:
        watch()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--text", help="send this text instead of recording")
    p.add_argument("--file", help="transcribe this audio file")
    p.add_argument("--seconds", type=float, help="record a fixed number of seconds")
    p.add_argument("--model", default="small", help="Whisper model size (default: small)")
    p.add_argument("--language", help="language code, e.g. en, hi, te (default: auto-detect)")
    p.add_argument("--dry-run", action="store_true", help="do not send to the robot")
    p.add_argument("--watch", action=argparse.BooleanOptionalAction, default=True,
                   help="log the robot's progress after sending (default on; --no-watch to skip)")
    p.add_argument("--no-brain", action="store_true", help="skip the Gemma planner, send plain text to /lab/instruction")
    args = p.parse_args()

    if not args.no_brain:
        import brain

        threading.Thread(target=brain.warm_up, daemon=True).start()

    if args.text:
        handle(args.text, args)
        return

    t0 = time.time()
    model, mode = load_model(args.model)
    log(f"[whisper] {args.model} on {mode}, loaded in {time.time() - t0:.1f} s")

    def run(audio):
        t = time.time()
        text, lang = transcribe(model, audio, args.language)
        log(f"[whisper] language={lang}, {time.time() - t:.2f} s")
        handle(text, args)

    if args.file:
        run(args.file)
        return
    while True:
        try:
            audio = record(args.seconds)
        except (EOFError, KeyboardInterrupt):
            break
        run(audio)
        if args.seconds is None:
            continue
        if input("Again? [Enter = yes, q = quit] ").strip().lower() == "q":
            break


if __name__ == "__main__":
    sys.exit(main())
