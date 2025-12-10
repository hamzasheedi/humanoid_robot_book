import requests
import json

# Test the chatbot API
API_URL = "http://localhost:8000/chat/ask"
question = "What is Physical AI?"
selected_text = ""
session_id = "test_session_123"

payload = {
    "question": question,
    "selected_text": selected_text,
    "session_id": session_id
}

print(f"Sending request to {API_URL}")
print(f"Payload: {json.dumps(payload, indent=2)}")

try:
    response = requests.post(API_URL, json=payload)
    print(f"Response Status Code: {response.status_code}")
    print(f"Response JSON: {response.json()}")
except Exception as e:
    print(f"Error: {e}")