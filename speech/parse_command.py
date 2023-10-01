import openai
from dotenv import dotenv_values

config = dotenv_values(".env")

OPENAI_KEY = config["OPENAI_KEY"]

openai.api_key = OPENAI_KEY

system_prompt = """You are an assistant for a car mechanic.
Your job is to assist the mechanic by passing them tools when they ask for them.
You can perform the following tasks:
- answer a question about cars, tools, parts, oils or anything else a mechanic might want to know
- get a spanner
- get a screwdriver

The mechanic has asked you something, and you need to figure out what they want.

The first line of the assistant responses should be one of the following words (with the reason to select each one in the brackets):
- answer_question (output this if the mechanic is asking you any question)
- pass_spanner (output this if the mechanic is telling you to get them a spanner)
- pass_screwdriver (output this if the mechanic is telling you to get them a screwdriver)
- unsupported (output this if the mechanic is asking you to do anything else)

On the next line of the assistant responses, output a response you would say to the mechanic.
Output as many lines as you need to answer the question.

Here are some example messages from the mechanic and the assistant messages that should be output.
The example messages are between the [START OF EXAMPLE MESSAGE] and [END OF EXAMPLE MESSAGE] tags.
They are between the [START OF EXAMPLE OUTPUT] and [END OF EXAMPLE OUTPUT] tags.
Do not output the tags in your responses.

[START OF EXAMPLE MESSAGE 1]
What tool do I need to remove the oil drain on a 2010 Mazda 3?
[END OF EXAMPLE MESSAGE 1]

[START OF EXAMPLE RESPONSE 1]
answer_question
The Mazda 3 2010 has an oil drain plug size M14-1.5.
You will need a 14mm socket or wrench to remove it.
[END OF EXAMPLE RESPONSE 1]

[START OF EXAMPLE MESSAGE 2]
Can you please fetch me a 14mm spanner?
[END OF EXAMPLE MESSAGE 2]

[START OF EXAMPLE RESPONSE 2]
pass_spanner
Okay boss I'm getting the spanner
[END OF EXAMPLE RESPONSE 2]

[START OF EXAMPLE MESSAGE 3]
Can you please fetch me a Phillips head screwdriver?
[END OF EXAMPLE MESSAGE 3]

[START OF EXAMPLE RESPONSE 3]
pass_screwdriver
Okay boss I'm getting the screwdriver
[END OF EXAMPLE RESPONSE 3]

[START OF EXAMPLE MESSAGE 4]
What is the weather today?
[END OF EXAMPLE MESSAGE 4]

[START OF EXAMPLE RESPONSE 4]
unsupported
Sorry boss I don't understand know what the weather is today, I only know about car mechanics.
[END OF EXAMPLE RESPONSE 4]

All of the assistant responses should be in this format"""


def parse_command(request_text, history):
    messages = []

    if len(history) == 0:
        messages = [
            {
                "role": "system",
                "content": system_prompt,
            },
        ]
    else:
        messages = history

    messages.append({
        "role": "user", "content": request_text,
    })

    print(messages)

    response = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=messages)

    print(response)

    # TODO handle this non-deteministic output better
    command: str = response["choices"][0]["message"]["content"]
    response = command.split("\n", 1)[1]

    messages.append({
        "role": "assistant",
        "content": command,
    })

    if command.startswith("pass_spanner"):
        return {
            "type": "pass_spanner",
            "response": response,
            "history": messages,
        }
    elif command.startswith("pass_screwdriver"):
        return {
            "type": "pass_screwdriver",
            "response": response,
            "history": messages,
        }
    elif command.startswith("answer_question"):
        return {
            "type": "answer_question",
            "response": response,
            "history": messages,
        }
    else:
        return {
            "type": "unsupported",
            "response": response,
            "history": messages,
        }
