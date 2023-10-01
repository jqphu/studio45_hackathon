import requests
from dotenv import dotenv_values
import pygame
import io

config = dotenv_values(".env")

XI_API_KEY = config["XI_API_KEY"]
CHUNK_SIZE = 1024
VOICE_ID = "21m00Tcm4TlvDq8ikWAM"
STREAM_URL = f"https://api.elevenlabs.io/v1/text-to-speech/{VOICE_ID}/stream"
USE_ELEVENLABS = config["USE_ELEVENLABS"] == "True"


headers = {
    "Accept": "audio/mpeg",
    "Content-Type": "application/json",
    "xi-api-key": XI_API_KEY,
}


def play_audio(input_text):
    if USE_ELEVENLABS:
        play_audio_from_elevenlabs(input_text)
    else:
        play_audio_from_file()


def play_audio_from_elevenlabs(input_text):
    print("Playing audio from Eleven Labs")

    data = {
        "text": input_text,
        "model_id": "eleven_monolingual_v1",
        "voice_settings": {"stability": 0.5, "similarity_boost": 0.5},
    }
    response = requests.post(STREAM_URL, json=data, headers=headers, stream=True)

    pygame.mixer.init()
    pygame.mixer.music.load(io.BytesIO(response.content))
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        pass


def play_audio_from_file():
    pygame.mixer.init()
    pygame.mixer.music.load("output.mp3")
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        pass
