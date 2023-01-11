from collections.abc import Generator

class Animation:
    class AnimationFinished(Exception):
        def __init__(self, message='Animation finished') -> None:
            self.message = message
            super().__init__(self.message)

    def __init__(self, animation_description: dict, num_led: int, controller_freq: float) -> None:
        self._brightness = 100
        self._num_led = num_led
        self._current_cycle = 1
        self._finished = False
        self._anim_iteration = 0
        self._progress = 0.0

        # Check for obligatory keys
        if not 'duration' in animation_description:
            raise KeyError('No duration in animation description')

        self._duration = animation_description['duration']
        if self._duration <= 0:
            raise KeyError('Duration has to be positive')

        if 'repeat' in animation_description:
            self._loops = animation_description['repeat']
            if self._loops <= 0 or isinstance(self._loops, float):
                raise KeyError('Repeat has to be a positive integer')
        else:
            self._loops = 1

        if self._duration * self._loops > 10:
            raise KeyError('Animation display duration (duration * repeat) exceeds 10 seconds')

        if 'brightness' in animation_description:
            self._brightness = float(animation_description['brightness'])
            if not (0 < self._brightness <= 1):
                raise KeyError('Brightness has to match boundaries 0 < brightness <= 1')
            self._brightness = int(round(self._brightness * 100))

        # evaluate number of animation frames
        self._anim_len = int(round(self._duration * controller_freq))
        self._full_anim_len = self._anim_len * self._loops

        if self._anim_len < 1:
            raise KeyError('Animation duration is too short to display with the current frequency')

    def __call__(self) -> Generator:
        if self._current_cycle <= self._loops:
            frame = self._update_frame()
            self._anim_iteration += 1
            self._progress = (
                self._anim_iteration + self._anim_len * (self._current_cycle - 1)
            ) / self._full_anim_len

            if self._anim_iteration >= self._anim_len:
                self._anim_iteration = 0
                self._current_cycle += 1

            if self._current_cycle > self._loops:
                self._finished = True

            for led in frame:
                yield led
        else:
            raise Animation.AnimationFinished

    def reset(self) -> None:
        self._anim_iteration = 0
        self._current_cycle = 1
        self._finished = False
        self._progress = 0.0

    def _update_frame(self) -> list:
        raise NotImplementedError

    @property
    def brightness(self) -> int:
        return self._brightness

    @property
    def num_led(self) -> int:
        return self._num_led

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def progress(self) -> bool:
        return self._progress
