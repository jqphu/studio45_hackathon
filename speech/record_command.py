import pyaudio
import wave
import io


def record_command(record_seconds) -> io.BytesIO:

    # set the chunk size of 1024 samples
    chunk = 1024
    # sample format
    FORMAT = pyaudio.paInt16
    # mono, change to 2 if you want stereo
    channels = 1
    # 44100 samples per second
    sample_rate = 44100
    # initialize PyAudio object
    p = pyaudio.PyAudio()
    # open stream object as input & output
    stream = p.open(
        format=FORMAT,
        channels=channels,
        rate=sample_rate,
        input=True,
        output=True,
        frames_per_buffer=chunk,
    )
    frames = []

    print("Recording...")
    
    for i in range(int(sample_rate / chunk * record_seconds)):
        data = stream.read(chunk)
        # if you want to hear your voice while recording
        # stream.write(data)
        frames.append(data)
    
    print("Finished recording")

    stream.stop_stream()
    stream.close()    
    p.terminate()

    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))

    wav_buffer.seek(0)
    wav_buffer.name = "file.wav"

    return wav_buffer

if __name__ == "__main__":
    duration_seconds = 3
    record_command(duration_seconds)
