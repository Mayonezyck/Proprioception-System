from openai import OpenAI

client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")
r = client.chat.completions.create(
    model="llama3.2",
    messages=[{"role":"user","content":"Say hi and tell me the time complexity of binary search."}],
)
print(r.choices[0].message.content)
