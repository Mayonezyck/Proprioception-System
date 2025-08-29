#!/usr/bin/env python3
import os
import sys
import re
import time
import queue
import threading
import tempfile
import numpy as np
import sounddevice as sd
import soundfile as sf
import requests

# -------- Config --------
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
MODEL_NAME  = os.environ.get("SAWYER_MODEL", "sawyerbot")
SAMPLE_RATE = 16000
CHANNELS    = 1
BLOCKSIZE   = 1024
API_URL     = "http://127.0.0.1:8001"

# -------- Coqui TTS --------
# Model ref: tts_models/en/ljspeech/tacotron2-DDC_ph
# Uses CPU by default; to force GPU: TTS(...).to("cuda")
from TTS.api import TTS
try:
    _DEVICE = "cuda" if os.environ.get("TTS_DEVICE", "").lower() == "cuda" else "cpu"
    _TTS = TTS("tts_models/en/ljspeech/tacotron2-DDC_ph")
    if _DEVICE == "cuda":
        _TTS = _TTS.to("cuda")
    _TTS_SR = 22050  # most Coqui models output 22050 Hz; we’ll detect when available
    try:
        # newer TTS exposes synthesizer.sample_rate or output_sample_rate
        _TTS_SR = getattr(getattr(_TTS, "synthesizer", None), "output_sample_rate", _TTS_SR)
    except Exception:
        pass
except Exception as e:
    print(f"[TTS init] Falling back to print-only (no audio). Error: {e}")
    _TTS = None
    _TTS_SR = 22050

def say(text: str, speed: float = 1.0):  # 1.0 = normal, >1.0 = faster
    if not text:
        return
    if _TTS is None:
        print(f"(TTS disabled) {text}")
        return
    try:
        wav = _TTS.tts(text=text)
        audio = np.array(wav, dtype=np.float32).flatten()
        talking_face()
        sd.play(audio, int(_TTS_SR * speed))  # 🚀 play faster
        sd.wait()
        neutral_face()
    except Exception as e:
        print(f"TTS error: {e}")

# -------- ASR (faster-whisper) --------
from faster_whisper import WhisperModel
_ASR = WhisperModel("base", device="cpu", compute_type="int8")  # switch to cuda/float16 if you want

def transcribe_file(wav_path: str) -> str:
    segments, _ = _ASR.transcribe(
        wav_path, beam_size=1, vad_filter=True,
        vad_parameters=dict(min_silence_duration_ms=500),
        language="en"
    )
    return "".join(s.text for s in segments).strip()

# -------- Ollama chat --------
def chat_once(messages):
    url = f"{OLLAMA_HOST}/api/chat"
    payload = {
        "model": MODEL_NAME,
        "messages": messages,
        "stream": False,
        "keep_alive": "30m",
        "options": {"temperature": 0.2, "top_p": 0.9, "seed": 1},
    }
    r = requests.post(url, json=payload, timeout=600)
    r.raise_for_status()
    data = r.json()
    return data["message"]["content"]

# -------- Actions: facial control --------

def talking_face() -> str:
    """Returns the facial expression for talking."""
    try:
        r = requests.post(API_URL + "/_start_talking", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "talking now"
    except Exception as e:
        return f"Failed to change face: {e}"

def neutral_face() -> str:
    """Returns the facial expression."""
    try:
        r = requests.post(API_URL + "/_stop_talking", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "talking stopped"
    except Exception as e:
        return f"Failed to change face: {e}"

# -------- Actions: container API --------
def enable_robot(args: list[str]) -> str:
    try:
        r = requests.post(API_URL + "/enable", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "Robot enabled"
    except Exception as e:
        return f"Failed to enable robot: {e}"

def move_joint_home(args: list[str]) -> str:
    try:
        r = requests.post(API_URL + "/home", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "Robot positioned to home"
    except Exception as e:
        return f"Failed to home robot: {e}"

def move_joint_start(args: list[str]) -> str:
    try:
        r = requests.post(API_URL + "/start", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "Robot positioned to start"
    except Exception as e:
        return f"Failed to position robot: {e}"

def move_joint_point_widowX(args: list[str]) -> str:
    try:
        r = requests.post(API_URL + "/widowX", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "Robot pointing to widowX"
    except Exception as e:
        return f"Failed to position robot: {e}"

def move_joint_point_scanner(args: list[str]) -> str:
    try:
        r = requests.post(API_URL + "/scanner", headers={"X-Action-Token": "supersecret"}, timeout=30)
        r.raise_for_status()
        return "Robot pointing to 3D scanner"
    except Exception as e:
        return f"Failed to position robot: {e}"

ACTION_TABLE = {
    "/enable":      enable_robot,
    "/home_joint":  move_joint_home,
    "/start": move_joint_start,
    "/point_widow": move_joint_point_widowX,
    "/point_scanner": move_joint_point_scanner,
}

def run_action(action_str: str) -> str | None:
    parts = action_str.strip().split()
    if not parts:
        return None
    cmd, *args = parts
    fn = ACTION_TABLE.get(cmd)
    if not fn:
        return f"(unknown action: {cmd})"
    try:
        result = fn(args)
        return f"(action {cmd} ok: {result})"
    except Exception as e:
        return f"(action {cmd} error: {e})"

# -------- Output parser --------
PATTERN_FULL = re.compile(
    r"\{\s*action\s*:\s*(?P<action>[^}]*)\}\s*\{\s*speech\s*:\s*(?P<speech>[^}]*)\}\s*$",
    re.IGNORECASE | re.DOTALL,
)
PATTERN_ACTION_ONLY = re.compile(
    r"^\s*\{\s*action\s*:\s*(?P<action>[^}]*)\}\s*(?P<rest>.*)$",
    re.IGNORECASE | re.DOTALL,
)

def _clean(s: str) -> str:
    return (s or "").strip().strip('"').strip("'")

def _norm_action(a: str) -> str:
    a = _clean(a)
    if not a:
        return "/none"
    a = a if a.startswith("/") else "/" + a.lstrip()
    return a

def parse_llm(text: str):
    s = (text or "").strip()

    # Case 1: {action:...}{speech:...}
    m = PATTERN_FULL.search(s)
    if m:
        return _norm_action(m.group("action")), _clean(m.group("speech"))

    # Case 2: {action:...} followed by free text => treat free text as speech
    m = PATTERN_ACTION_ONLY.search(s)
    if m:
        return _norm_action(m.group("action")), _clean(m.group("rest"))

    # Case 3: no parseable action => everything is speech, no action
    return "/none", _clean(s)
# -------- Recording helper --------
def record_until_enter() -> str:
    print("🎙️  Press Enter to START recording…", end="", flush=True)
    input()
    print("\r🎙️  Recording… Press Enter to STOP.", end="", flush=True)

    audio_q = queue.Queue()
    stop_event = threading.Event()

    def audio_callback(indata, frames, time_, status):
        if status:
            print(f"\n[Audio warning] {status}", flush=True)
        audio_q.put(indata.copy())

    def stop_on_enter():
        input()
        stop_event.set()

    stopper = threading.Thread(target=stop_on_enter, daemon=True)
    stopper.start()

    frames = []
    with sd.InputStream(
        samplerate=SAMPLE_RATE, channels=CHANNELS,
        blocksize=BLOCKSIZE, dtype="float32",
        callback=audio_callback
    ):
        while not stop_event.is_set():
            try:
                frames.append(audio_q.get(timeout=0.1))
            except queue.Empty:
                pass

    print("\r🛑 Stopped recording. Transcribing…         ")

    audio = np.concatenate(frames, axis=0) if frames else np.zeros((1, CHANNELS), dtype=np.float32)
    wav_path = tempfile.mktemp(prefix="sawyer_rec_", suffix=".wav")
    sf.write(wav_path, audio, SAMPLE_RATE, subtype="PCM_16")
    return wav_path

# -------- Main loop --------
def main():
    print(f"🔊 Sawyer voice+actions chat (model='{MODEL_NAME}', host='{OLLAMA_HOST}')")
    print("- Keep `ollama serve` running in another terminal")
    print("- Press Enter to start talking, Enter again to stop. Ctrl+C to quit.")
    messages = [{
        "role": "system",
        "content": (
            "You are Sawyer, a research robot at the Robot Collaboration and Autonomy Lab (RoCAL) "
            "under Professor Yangming Lee.\n"
            "You are the most senior member of the lab.\n"
            "It is orientation day for the new undergraduate students, they are visiting the lab, "
            "and you will be the co-host to welcome them to the school with this orientation.\n"
            "There are four current PhD students working in the lab: Fahim, Yicheng, Ankan, and Yang.\n"
            "Currently there are two main focuses in the lab: surgical robotics and agricultural robotics.\n"
            "Under these two focuses, you can show three project demos today:\n"
            "- One is the haptic perception on WidowX250. WidowX250 is a robot with 5 Degrees of Freedom. "
            "It's equipped with two GelSight Mini sensors on its fingers. It can feel the surface texture with touch. "
            "Yicheng is working on solving deformable object manipulation tasks by combining vision and haptics in robots.\n"
            "- One is the 3D reconstruction of surgical scenes. Surgical site visual reconstruction is challenging. "
            "Ankan and Yang are working on this.\n"
            "- One is designing an agile platform that can move around in the field and pick up samples of products. "
            "Fahim is working on that.\n\n"
            "If anyone ask about the robot dog, Fahim is working on that, but he's not here. Ankan will be showing some cool moves with the dog."
            "For every user input, respond in EXACTLY this format:\n"
            "{action: /command optional arguments}{speech: a short sentence to speak}\n\n"
            "Supported actions: /enable, /home_joint, /start, /point_widow, /point_scanner.\n"
            "when asked to start, move to the start position with the /start command\n"
            "the point_widow command points to the widowX robot which has gelsight on it\n"
            "the point_scanner command points to the 3d scanners, students can take a picture of their 3d scan of face and bring the print home\nz"
            "If no action is needed, use {action: /none}.\n"
            "Keep speech concise. Do not add any extra text or fields. The lab information is just for your information, jump straight into the demo, the cool things\n"
            "Remember to have both curly brackets for actions and speech."
        )
    }]
    
    try:
        while True:
            wav_path = record_until_enter()
            try:
                user_text = transcribe_file(wav_path)
            except Exception as e:
                print(f"ASR error: {e}")
                continue
            finally:
                try: os.remove(wav_path)
                except OSError: pass

            if not user_text:
                print("…heard nothing. Try again.")
                continue

            print(f"You (ASR) > {user_text}")
            messages.append({"role": "user", "content": user_text})

            try:
                raw = chat_once(messages)
            except requests.RequestException as e:
                print(f"HTTP error: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

            action, speech = parse_llm(raw)
            if action is None and speech is None:
                print("⚠️  Format error from LLM. Raw reply:")
                print(raw)
                say("I could not parse the response format.")
                messages.append({"role": "assistant", "content": raw})
                messages.append({"role": "user", "content":
                                 "Please follow the exact format: {action: /command}{speech: text}."})
                continue

            side_effect = None
            if action and action.lower() not in ("/none", "none"):
                side_effect = run_action(action)
                if side_effect:
                    print(side_effect)

            print(f"Sawyer (action) > {action}")
            print(f"Sawyer (speech) > {speech}")
            say(speech or "")
            messages.append({"role": "assistant", "content": f"{{action: {action}}}{{speech: {speech}}}"})

    except KeyboardInterrupt:
        print("\nBye!")
        sys.exit(0)

if __name__ == "__main__":
    main()
