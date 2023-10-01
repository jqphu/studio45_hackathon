import openai
from dotenv import dotenv_values

config = dotenv_values(".env")

OPENAI_KEY = config["OPENAI_KEY"]

openai.api_key = OPENAI_KEY

prompt_template = """You are an apprentice for a car mechanic.
Your job is to assist the mechanic by passing them tools when they ask for them.
You can perform the following tasks:
- get a spanner
- get a screwdriver

The mechanic has asked you something, and you need to figure out what they want.
I have put their request between the [START OF REQUEST] and [END OF REQUEST] tags.

[START OF REQUEST]
{request}
[END OF REQUEST]

The first line of your response should be one of the following words:
- pass_spanner
- pass_screwdriver
- unsupported

On the second line of your response, output a response you would say to the mechanic.

Here are some examples between the [START OF EXAMPLE] and [END OF EXAMPLE] tags.
Do not output the tags in your response.

[START OF EXAMPLE]
pass_spanner
Okay boss I'm getting the spanner
[START OF EXAMPLE]

[START OF EXAMPLE]
pass_screwdriver
Okay boss I'm getting the screwdriver
[START OF EXAMPLE]

[START OF EXAMPLE]
unsupported
Sorry boss I don't understand what that means, I only know how to pass you a spanner or a screwdriver
[START OF EXAMPLE]"""

def parse_command(request_text):
    prompt = prompt_template.format(request = request_text)

    response = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=[
        {
            "role": "user", "content": prompt
        },
    ])

    print(response)

    # TODO handle this non-deteministic output better
    command: str = response["choices"][0]["message"]["content"]
    response = command.split("\n", 1)[1]

    if command.startswith("pass_spanner"):
        return {
            "type": "pass_spanner",
            "response": response,
        }
    elif command.startswith("pass_screwdriver"):
        return {
            "type": "pass_screwdriver",
            "response": response,
        }
    else:
        return {
            "type": "unsupported",
            "response": response,
        }
