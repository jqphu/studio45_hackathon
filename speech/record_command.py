import pyaudio
import wave
import numpy as np
import struct
import io

def rms(data):
    """Calculate root mean square of data."""
    count = len(data)/2
    format = "<h" * int(count)
    shorts = np.array(list(struct.unpack(format, data)))
    return np.sqrt(np.mean(np.square(shorts)))

def record_command(record_seconds) -> str:

    chunk = 1024
    FORMAT = pyaudio.paInt16
    channels = 1
    sample_rate = 44100
    p = pyaudio.PyAudio()

    # Initialize streams for both microphones
    stream_left = p.open(format=FORMAT, channels=channels, rate=sample_rate, input=True, input_device_index=2)
    stream_right = p.open(format=FORMAT, channels=channels, rate=sample_rate, input=True, input_device_index=3)

    frames_left = []
    frames_right = []
    rms_values_left = []
    rms_values_right = []

    print("Recording...")

    for i in range(int(sample_rate / chunk * record_seconds)):
        data_left = stream_left.read(chunk, exception_on_overflow=False)
        data_right = stream_right.read(chunk, exception_on_overflow=False)

        frames_left.append(data_left)
        frames_right.append(data_right)

        # Calculate RMS for each chunk and store
        rms_values_left.append(rms(data_left))
        rms_values_right.append(rms(data_right))

    print("Finished recording")

    stream_left.stop_stream()
    stream_right.stop_stream()
    stream_left.close()
    stream_right.close()
    p.terminate()

    # Decide which side was louder on average
    avg_rms_left = np.mean(rms_values_left)
    avg_rms_right = np.mean(rms_values_right)

    if avg_rms_left > avg_rms_right:
        dominant_side = "Left"
        frames_to_save = frames_left
    elif avg_rms_right > avg_rms_left:
        dominant_side = "Right"
        frames_to_save = frames_right
    else:
        dominant_side = "Equal volume"
        frames_to_save = frames_left  # default to left if equal volume

    # Save the recording from the dominant microphone
    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames_to_save))

    wav_buffer.seek(0)
    with open("dominant_recording.wav", "wb") as f:
        f.write(wav_buffer.read())

    return dominant_side

if __name__ == "__main__":
    duration_seconds = 3
    direction = record_command(duration_seconds)
    print(f"The dominant side during the recording was: {direction}")
