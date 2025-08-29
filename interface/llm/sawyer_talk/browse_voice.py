import pyttsx3

engine = pyttsx3.init()
voices = engine.getProperty("voices")
for idx, v in enumerate(voices):
    print(f"[{idx}] id={v.id}  name={v.name}  lang={v.languages}  gender={getattr(v, 'gender', 'n/a')}")

