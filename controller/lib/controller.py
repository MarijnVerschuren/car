from abc import abstractmethod
from inputs import *
from math import *


__all__ = [
    "Component",
    "Button",
    "Trigger",
    "Joystick",
    "Controller",
    "PS3"
]


class Component:
    def __init__(self, event_name: str) -> None:
        self.event_name_len = len(event_name)
        self.event_name = event_name

    @abstractmethod
    def load(self, event: InputEvent): pass


class Button(Component):
    def __init__(self, event_name: str) -> None:
        super(Button, self).__init__(event_name)
        self._pressed = False

    @property
    def pressed(self) -> bool: return self._pressed
    def __bool__(self) -> bool: return self._pressed

    def load(self, event: InputEvent) -> None:
        if event.code == self.event_name: self._pressed = bool(event.state)


class Trigger(Component):
    def __init__(self, event_name: str) -> None:
        super(Trigger, self).__init__(event_name)
        self._x = 0
    
    @property
    def raw(self) -> int: return self._x

    @property
    def x(self) -> float: return self._x / 255

    def load(self, event: InputEvent) -> None:
        if event.code == self.event_name: self._x = event.state


class Joystick(Component):
    def __init__(self, event_name: str) -> None:
        super(Joystick, self).__init__(event_name)
        self._x = 0
        self._y = 0
        
    @property
    def raw_x(self) -> int: return self._x
    @property
    def raw_y(self) -> int: return self._y

    @property
    def x(self) -> float: return (self._x - 127.5) / 127.5
    @property
    def y(self) -> float: return (self._y - 127.5) / 127.5
    @property
    def angle(self) -> float: return atan(self.y/self.x)

    def load(self, event: InputEvent) -> None:
        if not event.code.startswith(self.event_name): return
        if event.code == (self.event_name + "X"): self._x = event.state
        if event.code == (self.event_name + "Y"): self._y = event.state


class Controller:
    def __init__(self, device: GamePad, parts: list) -> None:
        self.device = device
        self.parts = parts

    def update(self) -> None:
        for event in self.device.read():
            if event.ev_type in ["Sync", "Misc"]: continue
            for part in self.parts: part.load(event)


class PS3(Controller):
    def __init__(self, device: GamePad) -> None:
        self.joystick_L = Joystick("ABS_")
        self.joystick_R = Joystick("ABS_R")
        self.trigger_L = Trigger("ABS_Z")
        self.trigger_R = Trigger("ABS_RZ")
        self.y = Button("BTN_NORTH")    # triangle
        self.b = Button("BTN_EAST")     # circle
        self.a = Button("BTN_SOUTH")    # x_button
        self.x = Button("BTN_WEST")     # square
        self.up = Button("BTN_DPAD_UP")
        self.right = Button("BTN_DPAD_RIGHT")
        self.down = Button("BTN_DPAD_DOWN")
        self.left = Button("BTN_DPAD_LEFT")
        self.thumb_L = Button("BTN_THUMBL")
        self.thumb_R = Button("BTN_THUMBR")
        self.bumper_L = Button("BTN_TL")
        self.bumper_R = Button("BTN_TR")
        self.start = Button("BTN_START")
        self.select = Button("BTN_SELECT")
        self.mode = Button("BTN_MODE")
        super(PS3, self).__init__(device, [
            self.joystick_L,    self.joystick_R,
            self.trigger_L,     self.trigger_R,
            self.y, self.b, self.a, self.x,
            self.up, self.right, self.down, self.left,
            self.thumb_L, self.thumb_R,
            self.bumper_L, self.bumper_R,
            self.start, self.select, self.mode
        ])

    @property
    def triangle(self) -> Button: return self.y
    @property
    def circle(self) -> Button: return self.b
    @property
    def x_button(self) -> Button: return self.a
    @property
    def square(self) -> Button: return self.x

    def __str__(self) -> str:
        return f"""
        LJ: {self.joystick_L.x}, {self.joystick_L.y}
        RJ: {self.joystick_R.x}, {self.joystick_R.y}
        LT: {self.trigger_L.x}
        RT: {self.trigger_R.x}
        TRIANGLE: {self.y.pressed}
        CIRCLE:   {self.b.pressed}
        X_BUTTON: {self.a.pressed}
        SQUARE:   {self.x.pressed}
        UP:       {self.up.pressed}
        RIGHT:    {self.right.pressed}
        DOWN:     {self.down.pressed}
        LEFT:     {self.left.pressed}
        THUMB_L:  {self.thumb_L.pressed}
        THUMB_R:  {self.thumb_R.pressed}
        BUMPER_L: {self.bumper_L.pressed}
        BUMPER_R: {self.bumper_R.pressed}
        START:    {self.start.pressed}
        SELECT:   {self.select.pressed}
        MODE:     {self.mode.pressed}
        """
