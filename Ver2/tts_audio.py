#!/usr/bin/env python3
"""
TTS Audio Module — MAX98357A Stereo I2S Amplifier on Jetson Nano
================================================================

Hardware topology
-----------------
Two MAX98357A boards share a single I2S bus wired to the Jetson Nano
40-pin header:

  Jetson pin 12  (I2S0_CLK / BCLK)  ──►  BCLK  of BOTH amplifiers
  Jetson pin 35  (I2S0_FS  / LRCLK) ──►  LRC   of BOTH amplifiers
  Jetson pin 40  (I2S0_DOUT / DIN)  ──►  DIN   of BOTH amplifiers
  Jetson pin 17  (3.3 V)             ──►  SD_MODE of LEFT  amp  (pulls HIGH → left ch)
  Jetson GND                         ──►  SD_MODE of RIGHT amp  (pulls LOW  → right ch)
  Jetson pin 2   (5 V)               ──►  VIN   of BOTH amplifiers

SD_MODE channel mapping (per MAX98357A datasheet):
  SD high (>1.4 V)   → outputs LEFT  channel
  SD low  (<0.4 V)   → outputs RIGHT channel
  SD floating (~0.7V) → outputs (L + R) / 2 mono mix

Software requirements (Jetson Nano)
-------------------------------------
1.  I2S device-tree overlay — run once after OS install:
        sudo apt install -y espeak-ng alsa-utils
        # Confirm I2S card appears:
        aplay -l
        # Expected: "card N: tegrasndt210ref [tegra-snd-t210ref-mobile-rt565x]"

2.  Optional high-quality TTS (needs network):
        pip install gtts
        # ffmpeg for MP3 → WAV conversion:
        sudo apt install -y ffmpeg

3.  Optional Python TTS fallback:
        pip install pyttsx3

Usage
-----
    from tts_audio import TTSAudio, Priority

    tts = TTSAudio(config)
    tts.start()
    tts.speak("Robot is ready.")
    tts.speak("OBSTACLE DETECTED — stopping.", priority=Priority.CRITICAL, interrupt=True)
    tts.stop()
"""

import os
import logging
import threading
import queue
import subprocess
import tempfile
import time
import hashlib
from pathlib import Path
from typing import Optional
from enum import IntEnum

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Priority levels
# ---------------------------------------------------------------------------

class Priority(IntEnum):
    """Speech priority — higher value = spoken sooner."""
    LOW      = 0   # background info, exploration status
    NORMAL   = 1   # general status updates
    HIGH     = 2   # important events (mapping complete, low battery)
    CRITICAL = 3   # safety-critical (obstacle, emergency stop)


# ---------------------------------------------------------------------------
# Main TTS class
# ---------------------------------------------------------------------------

class TTSAudio:
    """
    Non-blocking text-to-speech via dual MAX98357A stereo I2S amplifiers.

    Synthesis backends (tried in preference order):
      1. espeak-ng  — offline, always available after apt install
      2. gTTS       — online, higher quality (requires network + pip install gtts)
      3. pyttsx3    — offline Python fallback

    Playback goes through ALSA ``aplay`` directly to the I2S card so that
    Linux audio routing is bypassed and latency is predictable.
    """

    # ------------------------------------------------------------------
    # Sensible defaults (all overridable via config['audio'])
    # ------------------------------------------------------------------
    _DEFAULT_ALSA_DEVICE  = "plughw:tegrasndt210ref,0"  # auto-rate conversion
    _DEFAULT_SAMPLE_RATE  = 22050
    _DEFAULT_CHANNELS     = 2        # stereo → drives both MAX98357A chips
    _DEFAULT_VOLUME       = 80       # 0–100 %
    _DEFAULT_ESPEAK_VOICE = "en+m3"  # male voice variant 3
    _DEFAULT_ESPEAK_SPEED = 150      # words per minute

    def __init__(self, config: dict):
        cfg = config.get('audio', {})

        self.alsa_device   : str  = cfg.get('alsa_device',   self._DEFAULT_ALSA_DEVICE)
        self.sample_rate   : int  = cfg.get('sample_rate',   self._DEFAULT_SAMPLE_RATE)
        self.channels      : int  = cfg.get('channels',      self._DEFAULT_CHANNELS)
        self.volume        : int  = cfg.get('volume',        self._DEFAULT_VOLUME)
        self.language      : str  = cfg.get('language',      'en')
        self.use_gtts      : bool = cfg.get('use_gtts',      False)
        self.espeak_voice  : str  = cfg.get('espeak_voice',  self._DEFAULT_ESPEAK_VOICE)
        self.espeak_speed  : int  = cfg.get('espeak_speed',  self._DEFAULT_ESPEAK_SPEED)
        self._enabled      : bool = cfg.get('enabled',       True)

        # Cache directory for gTTS (persists between runs)
        cache_path: str = cfg.get('cache_dir', '/tmp/robot_tts_cache')
        self._cache_dir = Path(cache_path)
        self._cache_dir.mkdir(parents=True, exist_ok=True)

        # Internal state
        self._queue: queue.PriorityQueue = queue.PriorityQueue()
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._current_proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()
        self._seq = 0           # tie-breaker so equal-priority items keep FIFO order
        self._seq_lock = threading.Lock()

        # Probe available backends
        self._espeak_ok  = self._check_espeak()
        self._gtts_ok    = self._check_gtts()   if self.use_gtts else False
        self._pyttsx3_ok = self._check_pyttsx3()
        self._aplay_ok   = self._check_aplay()

        if not self._aplay_ok:
            logger.warning(
                "TTS: 'aplay' not found — audio playback disabled. "
                "Install with: sudo apt install -y alsa-utils"
            )
            self._enabled = False
        elif not (self._espeak_ok or self._gtts_ok or self._pyttsx3_ok):
            logger.warning(
                "TTS: no synthesis engine found — audio disabled. "
                "Install with: sudo apt install -y espeak-ng"
            )
            self._enabled = False

        if self._enabled:
            engines = []
            if self._espeak_ok:   engines.append('espeak-ng')
            if self._gtts_ok:     engines.append('gTTS')
            if self._pyttsx3_ok:  engines.append('pyttsx3')
            logger.info(
                f"TTS audio ready — device={self.alsa_device}, "
                f"engines={engines}, stereo={self.channels == 2}"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Start background speech worker.  Call once after construction."""
        if not self._enabled:
            logger.info("TTS audio disabled — worker not started")
            return
        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._worker_loop, name="tts-worker", daemon=True
        )
        self._worker.start()
        logger.info("TTS worker started")

    def stop(self):
        """Gracefully stop worker and cancel any in-progress utterance."""
        self._stop_event.set()
        self._interrupt_playback()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=3.0)
        logger.info("TTS worker stopped")

    def speak(
        self,
        text: str,
        priority: Priority = Priority.NORMAL,
        interrupt: bool = False,
    ) -> None:
        """
        Queue *text* for spoken output.

        :param text:      The sentence(s) to speak.
        :param priority:  Queuing priority (CRITICAL messages jump the queue).
        :param interrupt: When True, cancel the ongoing utterance immediately
                          and flush items with lower priority from the queue.
        """
        if not self._enabled or not text:
            return

        if interrupt or priority >= Priority.CRITICAL:
            self._interrupt_playback()
            self._drain_queue_below(priority)

        with self._seq_lock:
            self._seq += 1
            seq = self._seq

        # PriorityQueue is a min-heap → negate priority for max-heap behaviour.
        # seq ensures FIFO ordering within the same priority level.
        self._queue.put((-int(priority), seq, text))

    def speak_status(self, text: str) -> None:
        """Convenience wrapper: LOW-priority background status."""
        self.speak(text, Priority.LOW)

    def speak_event(self, text: str) -> None:
        """Convenience wrapper: NORMAL-priority event."""
        self.speak(text, Priority.NORMAL)

    def speak_warning(self, text: str) -> None:
        """Convenience wrapper: HIGH-priority warning."""
        self.speak(text, Priority.HIGH, interrupt=False)

    def speak_critical(self, text: str) -> None:
        """Convenience wrapper: CRITICAL — interrupts everything."""
        self.speak(text, Priority.CRITICAL, interrupt=True)

    def set_volume(self, volume: int) -> None:
        """Set playback volume (0–100 %)."""
        self.volume = max(0, min(100, int(volume)))
        self._apply_alsa_volume()

    def set_speed(self, wpm: int) -> None:
        """Set espeak-ng speech rate in words-per-minute."""
        self.espeak_speed = max(80, min(400, int(wpm)))

    @property
    def is_speaking(self) -> bool:
        """True while an utterance is actively being played back."""
        with self._proc_lock:
            return (
                self._current_proc is not None
                and self._current_proc.poll() is None
            )

    @property
    def queue_size(self) -> int:
        """Number of utterances waiting in the queue."""
        return self._queue.qsize()

    @property
    def enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    # Internal — worker
    # ------------------------------------------------------------------

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                _, _, text = item
                self._speak_now(text)
            except Exception as exc:
                logger.error(f"TTS worker error: {exc}", exc_info=True)
            finally:
                self._queue.task_done()

    # ------------------------------------------------------------------
    # Internal — synthesis + playback
    # ------------------------------------------------------------------

    def _speak_now(self, text: str):
        """Synthesise *text* and play it synchronously on the worker thread."""
        wav_path = self._synthesise(text)
        if wav_path:
            self._play_wav(wav_path)

    def _synthesise(self, text: str) -> Optional[str]:
        """Return path to a 16-bit PCM WAV for *text* (None on failure)."""
        # Prefer online gTTS when enabled (better quality)
        if self._gtts_ok and self.use_gtts:
            path = self._synthesise_gtts(text)
            if path:
                return path
        # Fall back to espeak-ng (offline, always fast)
        if self._espeak_ok:
            path = self._synthesise_espeak(text)
            if path:
                return path
        # Last resort: pyttsx3
        if self._pyttsx3_ok:
            return self._synthesise_pyttsx3(text)
        return None

    def _synthesise_espeak(self, text: str) -> Optional[str]:
        """Generate WAV with espeak-ng (offline)."""
        fd, out_path = tempfile.mkstemp(suffix='.wav', dir=str(self._cache_dir))
        os.close(fd)
        cmd = [
            'espeak-ng',
            '-v', self.espeak_voice,
            '-s', str(self.espeak_speed),
            '-a', str(self.volume),       # amplitude 0-200
            '--stdout',
        ]
        try:
            result = subprocess.run(
                cmd + ['--', text],
                capture_output=True,
                timeout=10,
            )
            if result.returncode != 0:
                raise RuntimeError(result.stderr.decode(errors='replace'))
            with open(out_path, 'wb') as f:
                f.write(result.stdout)
            return out_path
        except Exception as exc:
            logger.error(f"espeak-ng synthesis failed: {exc}")
            try:
                os.unlink(out_path)
            except OSError:
                pass
            return None

    def _synthesise_gtts(self, text: str) -> Optional[str]:
        """Generate WAV via gTTS (requires network + ffmpeg)."""
        cache_key = hashlib.sha256(
            f"{self.language}:{text}".encode()
        ).hexdigest()[:16]
        wav_path = str(self._cache_dir / f"gtts_{cache_key}.wav")

        if os.path.exists(wav_path):
            return wav_path   # cache hit

        mp3_path = str(self._cache_dir / f"gtts_{cache_key}.mp3")
        try:
            from gtts import gTTS  # type: ignore[import]
            tts_obj = gTTS(text=text, lang=self.language, slow=False)
            tts_obj.save(mp3_path)
            # Convert MP3 → signed-16-bit WAV with ffmpeg
            cmd = [
                'ffmpeg', '-y', '-i', mp3_path,
                '-ar', str(self.sample_rate),
                '-ac', str(self.channels),
                '-f', 's16le',
                wav_path,
            ]
            subprocess.run(cmd, check=True, capture_output=True, timeout=15)
            return wav_path
        except Exception as exc:
            logger.error(f"gTTS synthesis failed: {exc}")
            return None
        finally:
            try:
                os.unlink(mp3_path)
            except OSError:
                pass

    def _synthesise_pyttsx3(self, text: str) -> Optional[str]:
        """Generate WAV via pyttsx3 (offline Python fallback)."""
        fd, out_path = tempfile.mkstemp(suffix='.wav', dir=str(self._cache_dir))
        os.close(fd)
        try:
            import pyttsx3  # type: ignore[import]
            engine = pyttsx3.init()
            engine.setProperty('rate', self.espeak_speed)
            engine.setProperty('volume', self.volume / 100.0)
            engine.save_to_file(text, out_path)
            engine.runAndWait()
            return out_path
        except Exception as exc:
            logger.error(f"pyttsx3 synthesis failed: {exc}")
            try:
                os.unlink(out_path)
            except OSError:
                pass
            return None

    def _play_wav(self, wav_path: str):
        """
        Play a WAV file through the I2S ALSA device.

        Uses ``aplay`` with explicit format flags so ALSA plug layer
        handles any sample-rate mismatch between synthesis engine and
        the hardware clock.
        """
        cmd = [
            'aplay',
            '-D', self.alsa_device,
            '--rate',     str(self.sample_rate),
            '--channels', str(self.channels),
            '--format',   'S16_LE',
            wav_path,
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            with self._proc_lock:
                self._current_proc = proc
            proc.wait()
        except Exception as exc:
            logger.error(f"aplay playback failed: {exc}")
        finally:
            with self._proc_lock:
                self._current_proc = None
            # Remove temp files (espeak-ng, pyttsx3); keep gTTS cache
            if os.path.basename(wav_path).startswith('tmp') or \
               not os.path.basename(wav_path).startswith('gtts_'):
                try:
                    os.unlink(wav_path)
                except OSError:
                    pass

    # ------------------------------------------------------------------
    # Internal — queue / playback management
    # ------------------------------------------------------------------

    def _interrupt_playback(self):
        """Terminate the currently playing aplay process."""
        with self._proc_lock:
            proc = self._current_proc
        if proc and proc.poll() is None:
            proc.terminate()
            # Give it a short moment to exit before forcing
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _drain_queue_below(self, min_priority: Priority):
        """
        Remove queued items whose priority is strictly below *min_priority*.
        Items at or above *min_priority* are re-inserted.
        """
        keep = []
        while True:
            try:
                item = self._queue.get_nowait()
                if -item[0] >= int(min_priority):
                    keep.append(item)
                self._queue.task_done()
            except queue.Empty:
                break
        for item in keep:
            self._queue.put(item)

    def _apply_alsa_volume(self):
        """
        Attempt to set ALSA Master volume on the I2S card.
        Non-critical: some I2S drivers expose no mixer controls.
        """
        try:
            subprocess.run(
                ['amixer', '-D', self.alsa_device,
                 'sset', 'Master', f'{self.volume}%'],
                capture_output=True,
                timeout=3,
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Backend availability probes
    # ------------------------------------------------------------------

    @staticmethod
    def _check_espeak() -> bool:
        try:
            subprocess.run(
                ['espeak-ng', '--version'],
                capture_output=True, timeout=3,
            )
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    @staticmethod
    def _check_gtts() -> bool:
        try:
            import gtts  # noqa: F401  # type: ignore[import]
            return True
        except ImportError:
            return False

    @staticmethod
    def _check_pyttsx3() -> bool:
        try:
            import pyttsx3  # noqa: F401  # type: ignore[import]
            return True
        except ImportError:
            return False

    @staticmethod
    def _check_aplay() -> bool:
        try:
            subprocess.run(
                ['aplay', '--version'],
                capture_output=True, timeout=3,
            )
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False
