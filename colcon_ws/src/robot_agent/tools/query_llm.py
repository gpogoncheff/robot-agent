import sys
import json
import requests
from pathlib import Path

prompt_path = Path("prompts/tool_prompt.txt")
local_llama_query_path = "http://localhost:8080/completion"

def main():
    if len(sys.argv) < 2:
        print("Usage: python tools.query_llm.py \"your command here\"")
        sys.exit(1)
    user_command = sys.argv[1]

    system_prompt = prompt_path.read_text()
    full_prompt = f"{system_prompt}\n\nUser command: {user_command}"
    payload = {
        "prompt": full_prompt,
        "n_predict": 256,
        "temperature": 0.0,
        "stop": ["User command:"]
    }

    response = requests.post(local_llama_query_path, json=payload)
    response.raise_for_status()

    data = response.json()
    text = str(data["content"]).strip()
    text_lines = text.split()
    if text_lines[0].startswith("```json"):
        text_lines.pop(0)
    if text_lines[-1].endswith("```"):
        text_lines.pop(-1)
    text = "\n".join(text_lines)

    print(text)


if __name__ == "__main__":
    main()