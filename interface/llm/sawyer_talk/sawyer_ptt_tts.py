#!/usr/bin/env python3
import os
import sys
import time
import queue
import threading
import tempfile
import numpy as np
import sounddevice as sd
import soundfile as sf
import requests
import pyttsx3

# ---- Config ----
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
MODEL_NAME  = os.environ.get("SAWYER_MODEL", "sawyerbot")
SAMPLE_RATE = 16000     # 16 kHz for ASR
CHANNELS    = 1
BLOCKSIZE   = 1024

# ---- TTS (pyttsx3) ----
engine = pyttsx3.init()
engine.setProperty("rate", 185)
# To choose a specific voice, run a small script to list them and set by id/name:
# engine.setProperty("voice", "<voice id>")

def say(text: str):
    engine.say(text)
    engine.runAndWait()

# ---- Ollama chat ----
def chat_once(messages):
    url = f"{OLLAMA_HOST}/api/chat"
    payload = {
        "model": MODEL_NAME,
        "messages": messages,
        "stream": False,
        "keep_alive": "30m",
        "options": {"temperature": 0.3, "top_p": 0.9, "seed": 1},
    }
    r = requests.post(url, json=payload, timeout=600)
    r.raise_for_status()
    data = r.json()
    return data["message"]["content"]

# ---- Speech-to-text (faster-whisper) ----
# Loads a small model; change to "small", "medium", etc. if you want higher quality.
from faster_whisper import WhisperModel
_ASR = WhisperModel("base", device="cpu", compute_type="int8")  # CPU-friendly

def transcribe_file(wav_path: str) -> str:
    segments, info = _ASR.transcribe(
        wav_path,
        beam_size=1,
        vad_filter=True,
        vad_parameters=dict(min_silence_duration_ms=500),
    )
    text = "".join(seg.text for seg in segments).strip()
    return text

# ---- Audio recording (press Enter to start/stop) ----
def record_until_enter() -> str:
    """
    Records 16kHz mono audio until user presses Enter again.
    Returns path to temp WAV file.
    """
    print("🎙️  Press Enter to START recording…", end="", flush=True)
    input()  # wait for Enter to start
    print("\r🎙️  Recording… Press Enter to STOP.", end="", flush=True)

    audio_q = queue.Queue()
    stop_event = threading.Event()

    def audio_callback(indata, frames, time_, status):
        if status:
            # just print once to avoid spam
            print(f"\n[Audio warning] {status}", flush=True)
        # copy to avoid referencing the same memory
        audio_q.put(indata.copy())

    # Thread to wait for stop Enter
    def stop_on_enter():
        input()           # wait for Enter to stop
        stop_event.set()

    stopper = threading.Thread(target=stop_on_enter, daemon=True)
    stopper.start()

    frames_collected = []

    with sd.InputStream(samplerate=SAMPLE_RATE,
                        channels=CHANNELS,
                        blocksize=BLOCKSIZE,
                        dtype="float32",
                        callback=audio_callback):
        while not stop_event.is_set():
            try:
                block = audio_q.get(timeout=0.1)
                frames_collected.append(block)
            except queue.Empty:
                pass

    print("\r🛑 Stopped recording. Transcribing…         ")

    # Concatenate and save to temp WAV (16-bit PCM)
    if frames_collected:
        audio = np.concatenate(frames_collected, axis=0)
    else:
        audio = np.zeros((1, CHANNELS), dtype=np.float32)

    wav_path = tempfile.mktemp(prefix="sawyer_rec_", suffix=".wav")
    sf.write(wav_path, audio, SAMPLE_RATE, subtype="PCM_16")
    return wav_path

# ---- Main loop ----
def main():
    print(f"🔊 Sawyer voice chat (model='{MODEL_NAME}', host='{OLLAMA_HOST}')")
    print("Tips:")
    print("- Keep another terminal running: `ollama serve`")
    print("- Press Enter to start talking, Enter again to stop.")
    print("- Ctrl+C to exit.")

    messages = []  # your Modelfile bakes Sawyer's identity

    try:
        while True:
            # 1) Capture voice -> text
            wav_path = record_until_enter()
            try:
                user_text = transcribe_file(wav_path)
            except Exception as e:
                print(f"ASR error: {e}")
                continue
            finally:
                try:
                    os.remove(wav_path)
                except OSError:
                    pass

            if not user_text:
                print("…heard nothing. Try again.")
                continue

            print(f"You (ASR) > {user_text}")
            messages.append({"role": "user", "content": user_text})

            # 2) LLM -> reply
            try:
                reply = chat_once(messages)
            except requests.RequestException as e:
                print(f"HTTP error: {e}")
                # do not append assistant turn on failure
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

            print(f"Sawyer > {reply}")
            # 3) TTS
            try:
                say(reply)
            except Exception as e:
                print(f"TTS error: {e}")

            messages.append({"role": "assistant", "content": reply})

    except KeyboardInterrupt:
        print("\nBye!")
        sys.exit(0)

if __name__ == "__main__":
    main()

