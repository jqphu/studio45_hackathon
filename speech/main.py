from wake_word_listener import wait_for_wake_word
from record_command import record_command
from transcribe_audio import transcribe_audio
from parse_command import parse_command
from play_audio import play_audio

RECORD_DURATION_SECONDS = 8

history = []

while True:
    print("listening for the wake word...")
    wait_for_wake_word()

    print("recording command...")
    audio_buffer = record_command(RECORD_DURATION_SECONDS)

    print("transcribing audio...")
    command_text = transcribe_audio(audio_buffer)
    print(command_text)

    print("parsing command...")
    command = parse_command(command_text, history)
    print(command)

    print("playing audio...")

    command_type = command["type"]
    response = command["response"]
    history = command["history"]

    # play_audio(response)

    if command_type == "pass_spanner":
       pass
    elif command_type == "pass_screwdriver":
        pass
    elif command_type == "unsupported":
        pass
    else:
        pass
