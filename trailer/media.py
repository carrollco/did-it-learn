"""Google media models for the trailer: Gemini text-to-speech for the narrator, Lyria for the music."""
import base64, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
from kit import client

VOICE = "Fenrir"   # a prebuilt Gemini voice; the style annotation asks for the movie-trailer delivery


def speak(c, text, out, style="deep, gravelly movie-trailer narrator, dramatic pauses, deadpan comic timing"):
    it = c.interactions.create(model="gemini-3.8-flash-tts", response_format={"type": "audio"},
                               input=[{"type": "user_input", "content": [{"type": "text", "text": text,
                                       "annotations": [{"type": "speech_metadata", "style": style}]}]}],
                               generation_config={"speech_config": [{"voice": VOICE}]})
    pathlib.Path(out).write_bytes(base64.b64decode(it.output_audio.data))
    return out


def music(c, prompt, out):
    it = c.interactions.create(model="lyria-3-clip-preview", input=prompt)
    pathlib.Path(out).write_bytes(base64.b64decode(it.output_audio.data))
    return out


if __name__ == "__main__":
    c = client()
    d = pathlib.Path("runs/trailer/test"); d.mkdir(parents=True, exist_ok=True)
    print(speak(c, "In a world where AI loops grade their own homework... one drone knew the answers in advance.", d / "voice.wav"))
    print(music(c, "Instrumental only, no vocals. Over-the-top action movie trailer: pounding drums, brass stabs, a comic pause, then a huge hit. 30 seconds.", d / "music.mp3"))
