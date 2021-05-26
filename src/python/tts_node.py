from easybot.node import Node
from espeak import espeak


class TTSNode(Node):
    def __init__(self) -> None:
        super().__init__('TTS', 5, worker_type='thread')

        # 150 is the default, I think
        espeak.set_parameter(espeak.Parameter.Rate, 130)
        # 0 is the default, I think
        espeak.set_parameter(espeak.Parameter.Wordgap, 1)

        self.subscribe('tts', self._handle_tts)

    def loop(self) -> None:
        pass

    def _handle_tts(self, message) -> None:
        message = message.data
        if not espeak.is_playing():
            self.log(f'Speaking: "{message}"')
            espeak.synth(message)

