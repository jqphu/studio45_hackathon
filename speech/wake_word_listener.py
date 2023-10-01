import pyaudio
import numpy as np
from openwakeword.model import Model

MODEL="hey_jarvis"
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1280

def wait_for_wake_word():
    audio = pyaudio.PyAudio()

    mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    owwModel = Model(wakeword_models=[MODEL])

    while True:
        audio = np.frombuffer(mic_stream.read(CHUNK), dtype=np.int16)

        owwModel.predict(audio)

        scores = list(owwModel.prediction_buffer[MODEL])

        curr_score = format(scores[-1], '.20f').replace("-", "")

        detected = "" if scores[-1] <= 0.5 else "Wakeword Detected!"
        output = f"{curr_score[0:5]}{detected}"
        
        print(output)

        if scores[-1] > 0.4:
            return

if __name__ == "__main__":
    wait_for_wake_word()
