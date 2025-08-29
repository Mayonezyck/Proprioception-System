#!/usr/bin/env python3
import os
import sys
import json
import requests
import pyttsx3

OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
MODEL_NAME  = os.environ.get("SAWYER_MODEL", "sawyerbot")

# Initialize offline TTS
engine = pyttsx3.init()
engine.setProperty("rate", 150)      # tweak voice speed if you like
engine.setProperty("voice", "24")  # choose a specific voice if desired

def say(text: str):
    engine.say(text)
    engine.runAndWait()

def chat_once(messages):
    url = f"{OLLAMA_HOST}/api/chat"
    payload = {
        "model": MODEL_NAME,
        "messages": messages,
        "stream": False,
        # keep the model warm in VRAM so replies start faster
        "keep_alive": "30m",
        # deterministic-ish output; adjust if you want more flair
        "options": {"temperature": 0.3, "top_p": 0.9, "seed": 1},
    }
    r = requests.post(url, json=payload, timeout=600)
    r.raise_for_status()
    data = r.json()
    # Ollama returns a single assistant message at data["message"]["content"]
    return data["message"]["content"]

def main():
    print(f"🔊 Sawyer voice chat (model='{MODEL_NAME}', host='{OLLAMA_HOST}')")
    print("Type your message and press Enter. Ctrl+C to exit.")
    # Keep a running conversation so the agent has context
    messages = []  # Your Modelfile already bakes Sawyer’s identity, so no system msg needed

    try:
        while True:
            user = input("\nYou > ").strip()
            if not user:
                continue
            messages.append({"role": "user", "content": user})

            try:
                reply = chat_once(messages)
            except requests.RequestException as e:
                print(f"HTTP error: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

            # Print and speak
            print(f"Sawyer > {reply}")
            say(reply)

            # Append assistant turn to preserve context
            messages.append({"role": "assistant", "content": reply})

    except KeyboardInterrupt:
        print("\nBye!")
        sys.exit(0)

if __name__ == "__main__":
    main()

