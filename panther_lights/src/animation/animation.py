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

    def __call__(self) -> None:
        '''returns new frame'''
        raise NotImplementedError

    def reset(self) -> None:
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
