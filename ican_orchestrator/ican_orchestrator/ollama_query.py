import ollama

# 1. Define the model and the prompt
MODEL_NAME = "qwen2.5:7b"
USER_PROMPT = "Write a 5-line rhyming poem about using the command line."

# 2. Call the generate function
try:
    response = ollama.generate(
        model=MODEL_NAME, 
        prompt=USER_PROMPT
    )

    # 3. Print the text response
    print(response['response'])

except ollama.ResponseError as e:
    print(f"Error querying Ollama: {e}")
    print("Ensure that Ollama is running (ollama serve &) and the model is pulled.")