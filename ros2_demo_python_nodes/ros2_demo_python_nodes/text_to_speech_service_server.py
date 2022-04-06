import rclpy.node
import rclpy.qos
import rclpy
import rclpy.executors

from ros2_demo_custom_msgs.srv._phrase import Phrase, Phrase_Request, Phrase_Response
from std_srvs.srv._trigger import Trigger, Trigger_Request, Trigger_Response

from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
from pydub.playback import play
import random
import time


class ROS2Text2SpeechNode(rclpy.node.Node):
    def __init__(self, *args):
        super(ROS2Text2SpeechNode, self).__init__(
            "ros2_imrt_text2_speech_node")

        self._phrases = []
        self.declare_parameter("phrases",  ['Nothing'])
        self._phrases = self.get_parameter("phrases")
        self.get_logger().info("Loaded following sentences from yaml")
        self._phrases = self._phrases.get_parameter_value().string_array_value

        for curr_phrase in self._phrases:
            # print the loaded phrarses
            self.get_logger().info(curr_phrase)

        self.create_service(srv_type=Trigger,
                            srv_name='speak_random',
                            callback=self._speak_random_callback,
                            qos_profile=rclpy.qos.qos_profile_services_default)

        self.create_service(srv_type=Phrase,
                            srv_name='speak_phrase',
                            callback=self._speak_phrase_callback,
                            qos_profile=rclpy.qos.qos_profile_services_default)

    def _play_phrase(self, phrase):
        tts = gTTS(text=phrase, lang="en", slow=False)
        mp3_audio = BytesIO()
        tts.write_to_fp(mp3_audio)
        mp3_audio.seek(0)
        play(AudioSegment.from_mp3(mp3_audio))

    def _speak_random_callback(self, req, res):
        phrase = random.choice(self._phrases)
        self._play_phrase(phrase)
        res = Trigger_Response()
        res.success = True
        res.message = ('Added [{}] to buffer').format(phrase)
        return res

    def _speak_phrase_callback(self, req, res):
        self._play_phrase(req.phrase)
        res = Phrase_Response()
        res.result = True
        return res


def main():
    rclpy.init()
    node = ROS2Text2SpeechNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
