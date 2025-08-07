from .baseController import *

class OpenLoopController(BaseController):
    def __init__(self, legs, motors, markers, load, motorsInit, motorsMin, motorsMax, cutoffFreq):
        super().__init__(legs, motors, markers, load, motorsInit, motorsMin, motorsMax, cutoffFreq)

        # Specific gui setup
        group = "Mechanical Parameters"
        self.guiNode.addData(name="loadMass", type="float", value=self.load.UniformMass.totalMass.value)
        MyGui.MyRobotWindow.addSettingInGroup("Load Mass", self.guiNode.loadMass, 0.0, 5., group)

        self.guiNode.addData(name="noise", type="float", value=0.)
        MyGui.MyRobotWindow.addSettingInGroup("Noise\n(% angle max)", self.guiNode.noise, 0, 100, "Motors Movement")


    def execute_control_at_camera_frame(self):
        desiredMotorPos = self.currentMotorsPos.copy()
        if self.guiNode.active.value:
            for i, motor in enumerate(self.motors):
                noise = self.guiNode.noise.value / 100 * np.pi / 4
                desiredMotorPos[i] = motor.position.value + np.random.normal(0, noise, 1)
        self.command = self.filter(
            desiredMotorPos, self.command,
            cutoffFreq=self.cutoffFreq, samplingFreq=self.samplingFreq)


    def execute_control_at_simu_frame(self):
        super().execute_control_at_simu_frame()
        self.load.UniformMass.totalMass.value = self.guiNode.loadMass.value


    def save(self):
        print("Saving data...")
        np.savez(
            os.path.join(data_path, "sofa", "openLoop.npz"),
            legsVel=np.array(self.legsVelList).reshape(len(self.legsVelList), self.legsVelList[0].shape[0]),
            legsPos=np.array(self.legsPosList).reshape(len(self.legsPosList), self.legsPosList[0].shape[0]),
            markersPos=np.array(self.markersPosList).reshape(len(self.markersPosList), self.markersPosList[0].shape[0]),
            motorsPos=np.array(self.motorsPosList).reshape(len(self.motorsPosList), self.motorsPosList[0].shape[0]),
            fps=1 / self.root.dt.value,
        )
