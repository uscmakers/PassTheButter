import argparse
import os
import struct
from datetime import datetime
from threading import Thread

import numpy as np
import pvporcupine
import pyaudio
import soundfile


class ButterVoice(Thread):

    def __init__(
            self,
            library_path,
            model_path,
            keyword_paths,
            sensitivities,
            input_device_index=None,
            output_path=None):
        super(ButterVoice, self).__init__()

        self._library_path = library_path
        self._model_path = model_path
        self._keyword_paths = keyword_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index

        self._output_path = output_path
        if self._output_path is not None:
            self._recorded_frames = []

    def run(self):
        keywords = list()
        for x in self._keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                keywords.append(' '.join(keyword_phrase_part[0:-6]))
        else:
            keywords.append(keyword_phrase_part[0])
        porcupine = None
        pa = None
        audio_stream = None
        try:
            porcupine = pvporcupine.create(
                library_path=self._library_path,
                model_path=self._model_path,
                keyword_paths=self._keyword_paths,
                sensitivities=self._sensitivities)

            pa = pyaudio.PyAudio()
            audio_stream = pa.open(
                rate=porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=porcupine.frame_length,
                input_device_index=self._input_device_index)

            print('Listening')
            for keyword, sensitivity in zip(keywords, self._sensitivities):
                print('  %s (%.2f)' % (keyword, sensitivity))
            print('}')

            while True:
                pcm = audio_stream.read(porcupine.frame_length)
                pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

                if self._output_path is not None:
                    self._recorded_frames.append(pcm)

                result = porcupine.process(pcm)
                if result >= 0:
                    "When this if statement is triggered," \
                    "PicoVoice is confirming that the wake word" \
                    "has been heard.  TODO: implement STT service"


        except KeyboardInterrupt:
            print('Stopping ...')

        finally:
            if porcupine is not None:
                porcupine.delete()

            if audio_stream is not None:
                audio_stream.close()

            if pa is not None:
                pa.terminate()

            if self._output_path is not None and len(self._recorded_frames) > 0:
                recorded_audio = np.concatenate(self._recorded_frames, axis=0).astype(np.int16)
                soundfile.write(self._output_path, recorded_audio, samplerate=porcupine.sample_rate, subtype='PCM_16')

    @classmethod
    def show_audio_devices(cls):
        fields = ('index', 'name', 'defaultSampleRate', 'maxInputChannels')

        pa = pyaudio.PyAudio()

        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            print(', '.join("'%s': '%s'" % (k, str(info[k])) for k in fields))

        pa.terminate()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--keywords',
        nargs='+',
        help='List of default keywords for detection. Available keywords: %s' % ', '.join(sorted(pvporcupine.KEYWORDS)),
        choices=sorted(pvporcupine.KEYWORDS),
        metavar='')

    parser.add_argument(
        '--keyword_paths',
        nargs='+',
        help="Absolute paths to keyword model files. If not set it will be populated from `--keywords` argument")

    parser.add_argument('--library_path', help='Absolute path to dynamic library.', default=pvporcupine.LIBRARY_PATH)

    parser.add_argument(
        '--model_path',
        help='Absolute path to the file containing model parameters.',
        default=pvporcupine.MODEL_PATH)

    parser.add_argument(
        '--sensitivities',
        nargs='+',
        help="Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher " +
             "sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 " +
             "will be used.",
        type=float,
        default=None)

    parser.add_argument('--audio_device_index', help='Index of input audio device.', type=int, default=None)

    parser.add_argument('--output_path', help='Absolute path to recorded audio for debugging.', default=None)

    parser.add_argument('--show_audio_devices', action='store_true')

    args = parser.parse_args()

    if args.show_audio_devices:
        ButterVoice.show_audio_devices()
    else:
        if args.keyword_paths is None:
            if args.keywords is None:
                raise ValueError("Either `--keywords` or `--keyword_paths` must be set.")

            keyword_paths = [pvporcupine.KEYWORD_PATHS[x] for x in args.keywords]
        else:
            keyword_paths = args.keyword_paths

        if args.sensitivities is None:
            args.sensitivities = [0.5] * len(keyword_paths)

        if len(keyword_paths) != len(args.sensitivities):
            raise ValueError('Number of keywords does not match the number of sensitivities.')

        ButterVoice(
            library_path=args.library_path,
            model_path=args.model_path,
            keyword_paths=keyword_paths,
            sensitivities=args.sensitivities,
            output_path=args.output_path,
            input_device_index=args.audio_device_index).run()


if __name__ == '__main__':
    main()
