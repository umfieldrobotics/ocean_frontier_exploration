from isaacsim.oceansim.sensors.UW_Camera import UW_Camera

class UW_StereoCamera:
    def __init__(self, prim_path, baseline=0.12, resolution=(1280,720), translation=None):
        if translation is None:
            translation = [0.0, 0.0, 0.0]

        # IMPORTANT: use translation (local) instead of position (world) if attaching to robot
        self.left = UW_Camera(
            prim_path=f"{prim_path}/left",
            name="uw_left",
            resolution=list(resolution),
            translation=[translation[0], translation[1] - baseline/2, translation[2]],
        )
        self.right = UW_Camera(
            prim_path=f"{prim_path}/right",
            name="uw_right",
            resolution=list(resolution),
            translation=[translation[0], translation[1] + baseline/2, translation[2]],
        )

    def initialize(self, **kwargs):
        self.left.initialize(**kwargs)
        self.right.initialize(**kwargs)

    def render(self):
        # Debug proof: annotators produce data
        raw_l = self.left._rgba_annot.get_data()
        raw_r = self.right._rgba_annot.get_data()
        print(f"[LEFT {self.left._prim_path}] rgba.size={raw_l.size}")
        print(f"[RIGHT {self.right._prim_path}] rgba.size={raw_r.size}")

        self.left.render()
        self.right.render()

    def close(self):
        self.left.close()
        self.right.close()
