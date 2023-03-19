#! /usr/bin/env python3

import os
import cv2
import math
import pyaudio
import numpy as np
import copy as copy_module

from io import BytesIO
from gtts import gTTS
from pydub import AudioSegment

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from speaking_face.msg import FaceConfig, FaceParam, ShapeConfig

from cv_bridge import CvBridge

cvb = CvBridge()
pack = rospkg.RosPack()
path = pack.get_path('speaking_face')
face_path = os.path.join(path, "faces")
phoneme_path = os.path.join(path, "phonemes")


class FaceManager:
    def __init__(self) -> None:

        self._DOWN_SAMPLE_SIZE = 64
        self.last_observation = None
        self.attention = np.asarray((0.0, 0.0))

        # face configurations
        self.face_config = FaceConfig()
        self.face_param = FaceParam(mouth_sensitivity=0.75, eye_left_sensitivity=0.75, eye_right_sensitivity=0.75)

        # eye sets
        self.eye_status = 0
        self.norm = ShapeConfig(width=1.0, height=1.0, size=0.3, curve=-0.25, rotation=0.0)
        self.wink = ShapeConfig(width=1.3, height=0.2, size=0.2, curve=0.0, rotation=0.0)
        self.happy = ShapeConfig(width=1.3, height=0.5, size=0.3, curve=-0.45, rotation=0.0)
        self.confused = ShapeConfig(width=1.0, height=1.0, size=0.15, curve=-0.2, rotation=0.0)
        self.eyes = [self.norm, self.wink, self.happy, self.confused]
        self.lasting = [10, 0.2, 5, 3]
        self.duration = rospy.Duration(5)
        self.last_change = rospy.Time.now()
        self.wink_count = 0.0

        
        # mouth sets
        self.none = ShapeConfig(width=1.0, height=0.2, size=0.1, curve=0.3, rotation=0.0)
        self.a = ShapeConfig(width=1.3, height=1.0, size=0.2, curve=-0.3, rotation=0.0)
        self.i = ShapeConfig(width=1.3, height=0.3, size=0.2, curve=0.1, rotation=0.0)
        self.o = ShapeConfig(width=1.0, height=1.0, size=0.2, curve=-0.1, rotation=0.0)
        self.u = ShapeConfig(width=1.0, height=1.3, size=0.1, curve=-0.3, rotation=0.0)

        # this need to be the same as defined in Phoneme
        self.mouthes = [self.none, self.a, self.i, self.o, self.u]

        # init the face
        self.face_config.mouth = copy_module.deepcopy(self.none)
        self.face_config.eye_left = copy_module.deepcopy(self.norm)
        self.face_config.eye_right = copy_module.deepcopy(self.norm)
        self.face_config.eye_left.offset_x = 0.6
        self.face_config.eye_left.offset_y = 0.2
        self.face_config.eye_right.offset_x = -0.6
        self.face_config.eye_right.offset_y = 0.2
        self.face_config.mouth.offset_y = -0.3

        self.pub_config = rospy.Publisher('/speaking_face/face_config', FaceConfig, queue_size=10)
        self.pub_param = rospy.Publisher('/speaking_face/face_param', FaceParam, queue_size=10)

        rospy.Rate(1).sleep()
        self.pub_param.publish(self.face_param)

        # self.sub_rgb = rospy.Subscriber('camera', Image, queue_size=1, callback=self.rgb_cb)
        # self.sub_depth = rospy.Subscriber('depth', Image, queue_size=1, callback=self.depth_cb)

    def rgb_cb(self, msg):
        img = cvb.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.resize(img, (64, 64))
        img = cv2.GaussianBlur(img, ksize=(7,7), sigmaX=7)

        if self.last_observation is not None:
            diff = np.abs(img-self.last_observation).max(axis=2)>50
            self.attention = np.asarray((diff).nonzero()).mean(axis=1)/self._DOWN_SAMPLE_SIZE-0.5
            print(self.attention)
            cv2.imshow('diff', diff.astype(float))
            cv2.waitKey(1)

        self.last_observation = img

    def depth_cb(self, msg):
        depth = cvb.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth = cv2.resize(depth, (self._DOWN_SAMPLE_SIZE, self._DOWN_SAMPLE_SIZE))

    def copyShape(self, dst: ShapeConfig, src: ShapeConfig, copy_size=False):
        dst.width = src.width
        dst.height = src.height
        dst.curve = src.curve
        if copy_size:
            dst.size=src.size

    def PickMouth(self, id, strength):

        self.copyShape(self.face_config.mouth, self.mouthes[id])
        self.face_config.mouth.size = math.log10(max(strength-600, 1.0))/60+0.1

        if np.random.randn(1)>5.0:
            self.face_config.mouth.rotation = np.random.randn(1)/20.0
            self.face_config.mouth.offset_x = np.random.randn(1)/30.0
            self.face_config.mouth.offset_y = -0.3+np.random.randn(1)/30.0

    def EyeMovement(self):

        # switch eye
        if rospy.Time.now() - self.last_change > self.duration:
            self.eye_status = np.random.randint(0, len(self.eyes))
            self.duration = rospy.Duration(self.lasting[self.eye_status] + np.random.randn()*self.lasting[self.eye_status]/5.0)
            self.last_change = rospy.Time.now()
            self.copyShape(self.face_config.eye_left, self.eyes[self.eye_status], copy_size=True)
            self.copyShape(self.face_config.eye_right, self.eyes[self.eye_status], copy_size=True)

        shift_x = 0.0
        shift_y = 0.0
        rot = 0.0
        # eye random movement
        if np.random.randn(1)>5.0:
            shift_x = np.random.randn(1)/10.0
            shift_y = np.random.randn(1)/10.0
            rot = np.random.randn(1)/10.0

            self.face_config.eye_left.offset_x = self.attention[0]-0.6+shift_x
            self.face_config.eye_left.offset_y = self.attention[1]+0.2+shift_y
            self.face_config.eye_left.rotation = rot
            self.face_config.eye_right.offset_x = self.attention[0]+0.6+shift_x
            self.face_config.eye_right.offset_y = self.attention[1]+0.2+shift_y
            self.face_config.eye_right.rotation = -rot

    def publish(self):

        # wink wink
        wink=False
        if np.random.randn(1)>self.wink_count:
            self.copyShape(self.face_config.eye_left, self.wink, copy_size=True)
            self.copyShape(self.face_config.eye_right, self.wink, copy_size=True)
            wink=True
            self.wink_count = 10.0

        self.pub_config.publish(self.face_config)
        self.wink_count -=0.1

        if wink:
            self.copyShape(self.face_config.eye_left, self.eyes[self.eye_status], copy_size=True)
            self.copyShape(self.face_config.eye_right, self.eyes[self.eye_status], copy_size=True)



class Phoneme:
    """
    Phoneme class is responsible for parsing audio and estimate the mouth.
    """

    def __init__(self, frame_size=1024, phonemes = ["a","i","o","u"]):

        self.p = np.asarray([np.load(os.path.join(phoneme_path,f"{p}.npy")) for p in phonemes])
        self.mask = np.fft.fftfreq(frame_size, 1.0/44100.0)<5000
        self.spectrums = self.p[:, self.mask]

        self.spectrums = self.SoundFeature(self.spectrums)
        self.spectrums /= np.linalg.norm(self.spectrums, axis=1, keepdims=True)

        self.ResetVote()
    
    def GetMouth(self):

        id = self.vote.argmax()
        strength = self.strength
        self.ResetVote()
        return id, strength
    
    def ResetVote(self):

        self.vote = np.zeros(self.p.shape[0]+1, dtype=int)
        self.strength = 0

    def Vote(self, id, strength):

        if strength<600:
            self.vote[0]+=1
        else:
            self.vote[id]+=1
            self.strength = max(strength, self.strength)
    
    def WaveToSpec(self, raw):

        return np.fft.fft(np.fromstring(raw, dtype=np.int16))[self.mask]

    def SoundFeature(self, spectrum, segment=17):

        if len(spectrum.shape)==2:
            seg = np.absolute(spectrum).reshape(spectrum.shape[0],segment,-1)
        elif len(spectrum.shape)==1:
            seg = np.absolute(spectrum).reshape(segment,-1)
        return np.max(seg, axis=-1)*0.5 + np.average(seg, axis=-1)
    
    def MatchPhoneme(self, frame):

        spectrum = self.WaveToSpec(frame)
        feature = self.SoundFeature(spectrum)
        strength = np.average(feature)

        if strength>0.0:
            feature = feature/np.linalg.norm(feature)

        feature = feature[:, np.newaxis]
        mouth = np.argmax(np.matmul(self.spectrums, feature))+1

        self.Vote(mouth, strength)


class Speaker:
    """
    Speaker class is responsible for subscribing text, TTS and audio.
    """
    
    def __init__(self, fap: Phoneme):
        
        self.start = 0
        self.p = pyaudio.PyAudio()
        self.stream = None

        self.sample_width = 0
        self.channels = 0
        self.sample_rate = 0
        self.frame_size = 1024

        self.sounds = []
        self.current_frame = None

        self.fap = fap

        self.text_sub = rospy.Subscriber("speaking_face/text", String, callback=self.text_sub, queue_size=10)
        self.state_pub = rospy.Publisher("Speaking_face/status", String, queue_size=10)

    def text_sub(self, msg: String):
        text = msg.data
        sound = self.TextToSpeech(text)
        self.sounds.append(sound)

        print(f"Receieve {text}, we have {len(self.sounds)} sounds pending...")

    def stream_callback(self, in_data, frame_size, time_info, status):

        # get the frame for playing
        self.current_frame = self.GetFrame(frame_size*2)
        self.start+=frame_size*2

        # prepare a leading frame to parse the mouth
        parse_frame = self.GetFrame(frame_size=frame_size*2, leading=4096)
        if parse_frame is not None and len(parse_frame)>=self.frame_size*2:
            self.fap.MatchPhoneme(parse_frame)
        
        return (self.current_frame, pyaudio.paContinue)
    
    def GetFrame(self, frame_size, leading=0):
        beg = self.start+leading
        end = self.start+leading+frame_size
        return self.current_sound._data[beg:end] if beg < len(self.current_sound._data) else None

    def Play(self):
        if (self.stream is not None) and self.stream.is_active():
            return

        if len(self.sounds)>0:

            print(f"Pop the last audio and speak...")

            # release the resources
            if self.stream is not None:
                self.stream.close()

            self.current_sound = self.sounds.pop(0)
            self.start = 0

            self.sample_width = self.current_sound.sample_width
            self.channels = self.current_sound.channels
            self.sample_rate = self.current_sound.frame_rate

            self.stream = self.p.open(format=self.p.get_format_from_width(self.sample_width),
                        channels=self.channels,
                        rate=self.sample_rate,
                        output=True,
                        stream_callback=self.stream_callback)

    def Terminate(self):
        if self.stream is not None:
            self.stream.close()
        
        if self.p is not None:
            self.p.terminate()

    def TextToSpeech(self, text):
        # prepare sound
        tts = gTTS(text=text, lang='en')
        tts.stream()
        audio = BytesIO()
        tts.write_to_fp(audio)
        audio.seek(0)

        sound = AudioSegment.from_file(audio, format='mp3')
        sound = sound.set_frame_rate(44100)
        return sound
    

if __name__ == "__main__":
    rospy.init_node("SpeakingFace", anonymous=True)

    phoneme = Phoneme(frame_size=1024, phonemes=["a","i","u","o"])
    speaker = Speaker(phoneme)
    face = FaceManager()

    while not rospy.is_shutdown():

        speaker.Play()
        id, strength = phoneme.GetMouth()
        face.PickMouth(id, strength=strength)
        face.EyeMovement()
        face.publish()

        rospy.Rate(20).sleep()