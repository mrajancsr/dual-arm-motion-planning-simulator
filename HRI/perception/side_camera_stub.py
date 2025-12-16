import numpy as np
from typing import Optional, Tuple

class SideCameraStub:
    def __init__(self, mode: str = "transform", front_camera_width: int = 640, front_camera_height: int = 480):
        self.mode = mode
        self.front_width = front_camera_width
        self.front_height = front_camera_height
        self.fixed_u2 = front_camera_width * 0.6
        self.fixed_v2 = front_camera_height * 0.5
        self.rng = np.random.RandomState(42)
        self.transform_scale = 0.8
        self.transform_offset_u = front_camera_width * 0.1
        self.transform_offset_v = front_camera_height * 0.05

    def get_hand_pixels(self, front_u: Optional[float] = None, front_v: Optional[float] = None) -> Tuple[float, float]:
        if self.mode == "fixed":
            return (self.fixed_u2, self.fixed_v2)
        elif self.mode == "random":
            u2 = self.rng.uniform(0.2 * self.front_width, 0.8 * self.front_width)
            v2 = self.rng.uniform(0.2 * self.front_height, 0.8 * self.front_height)
            return (u2, v2)
        elif self.mode == "transform":
            if front_u is None or front_v is None:
                front_u = self.front_width / 2
                front_v = self.front_height / 2
            u2 = front_u * self.transform_scale + self.transform_offset_u
            v2 = front_v * self.transform_scale + self.transform_offset_v
            u2 = np.clip(u2, 0, self.front_width - 1)
            v2 = np.clip(v2, 0, self.front_height - 1)
            return (u2, v2)
        else:
            raise ValueError(f"Unknown mode: {self.mode}. Use 'fixed', 'random', or 'transform'")

    def set_mode(self, mode: str):
        if mode not in ['fixed', 'random', 'transform']:
            raise ValueError(f"Invalid mode: {mode}")
        self.mode = mode

    def get_camera_info(self) -> dict:
        return {
            'width': self.front_width,
            'height': self.front_height,
            'mode': self.mode,
            'is_stub': True
        }

if __name__ == "__main__":
    stub = SideCameraStub(mode="transform")
    print("Testing Side Camera Stub")
    print("=" * 50)
    test_cases = [
        (320, 240),
        (100, 100),
        (540, 380),
        (None, None)
    ]
    for front_u, front_v in test_cases:
        u2, v2 = stub.get_hand_pixels(front_u, front_v)
        print(f"Front: ({front_u}, {front_v}) -> Side: ({u2:.1f}, {v2:.1f})")
    print("\nTesting different modes:")
    for mode in ['fixed', 'random', 'transform']:
        stub.set_mode(mode)
        u2, v2 = stub.get_hand_pixels(320, 240)
        print(f"{mode:10s}: ({u2:.1f}, {v2:.1f})")
