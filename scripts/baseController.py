import os
from math import pi
import numpy as np
import Sofa
import Sofa.ImGui as MyGui

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")

class BaseController(Sofa.Core.Controller):
    def __init__(self, legs, motors, markers, load, motorsInit, motorsMin, motorsMax, cutoffFreq):
        Sofa.Core.Controller.__init__(self)
        self.name = "Control Strategy"
        self.root = legs[0].getRoot()
        self.legs = legs
        self.motors = motors
        self.markers = markers
        self.load = load

        # Motors setup
        speedLimit = 75.57
        self.speedLimit = speedLimit * 2 * pi / 60 * self.root.dt.value
        self.motorsInit = np.array(motorsInit)
        self.motorsMin = np.array(motorsMin)
        self.motorsMax = np.array(motorsMax)
        self.cutoffFreq = cutoffFreq
        self.samplingFreq = 60.
        for i, motor in enumerate(self.motors):
            motor.addData(name="position", type="float", value=0.)

        # Forces setup
        self.root.VisualStyle.displayFlags.value = "showVisualModels showForceFields"
        self.force = self.load.addObject('ConstantForceField', name="User", indices=[0], showArrowSize=0.03)
        self.force.forces.value = [7 * [0.]]
        self.maxforce = 100

        # GUI setup
        self.guiNode = self.root.addChild("guiNode")
        self.setup_gui()

        # Initialize simulation variables
        self.setup_variables()


    def setup_gui(self):
        for i, motor in enumerate(self.motors):
            MyGui.MyRobotWindow.addSettingInGroup(f"Motor {i+1}", motor.position, self.motorsMin[i], self.motorsMax[i], "Motors Movement")

        self.guiNode.addData(name="forceY", type="float", value=0.)
        self.guiNode.addData(name="forceZ", type="float", value=0.)
        self.guiNode.addData(name="record", type="bool", value=False)
        self.guiNode.addData(name="saving", type="bool", value=False)
        self.guiNode.addData(name="active", type="bool", value=False)

        MyGui.MyRobotWindow.addSettingInGroup("Vertical", self.guiNode.forceY, -self.maxforce, self.maxforce, "User Actions")
        MyGui.MyRobotWindow.addSettingInGroup("Horizontal", self.guiNode.forceZ, -self.maxforce, self.maxforce, "User Actions")
        MyGui.MyRobotWindow.addSettingInGroup("Active", self.guiNode.active, 0, 1, "Buttons")
        MyGui.MyRobotWindow.addSettingInGroup("Record", self.guiNode.record, 0, 1, "Buttons")
        MyGui.MyRobotWindow.addSettingInGroup("Saving", self.guiNode.saving, 0, 1, "Buttons")


    def setup_variables(self):

        # constants
        self.initThreshold = 5e-2
        self.step = 0
        self.dtRatio = int(round(1 / (60 * self.root.dt.value)))
        self.mos = [leg.getChild(f"Leg{i}DeformablePart").MechanicalObject for i, leg in enumerate(self.legs)]

        # measures
        self.legsVel = np.zeros_like(np.vstack([mo.velocity.value.copy() for mo in self.mos]))
        self.legsPos = np.zeros_like(np.vstack([mo.position.value.copy() for mo in self.mos]))
        self.lastLegsPos = np.zeros_like(np.vstack([mo.position.value.copy() for mo in self.mos]))
        self.markersPos = np.zeros_like(np.vstack([marker.position.value.copy()[:, [1, 2]].reshape(-1, 1) for marker in self.markers]))
        self.currentMotorsPos = np.zeros((len(self.motors), ))

        # states
        self.command = np.zeros((len(self.motors), ))

        # boolean
        self.start = False
        self.hasSaved = False

        # data storage
        self.legsPosList = []
        self.legsVelList = []
        self.markersPosList = []
        self.motorsPosList = []


    def onAnimateBeginEvent(self, e):
        if self.start:
            self.measure_data()

            if self.step % self.dtRatio == 0:
                self.step = 0
                self.execute_control_at_camera_frame()

            self.execute_control_at_simu_frame()

            if self.step % self.dtRatio == 0 and self.guiNode.record.value:
                self.record_data()

            self.step += 1
        else:
            self.initialize_simulation()

        if self.hasSaved:
            self.hasSaved = False
            self.guiNode.saving.value = False
        if self.guiNode.saving.value and not self.hasSaved:
            self.hasSaved = True
            self.save()
        if self.guiNode.record.value and not self.guiNode.active.value:
            self.guiNode.active.value = True


    def initialize_simulation(self):
        legsVel = np.vstack([mo.velocity.value.copy() for mo in self.mos])
        legsPos = np.vstack([mo.position.value.copy() for mo in self.mos])
        if np.linalg.norm(legsPos - self.lastLegsPos) < self.initThreshold:
            self.start = True
            self.guiNode.active.value = True
            print("Simulation started")
        self.lastLegsPos = legsPos
        self.initialLegsVel = legsVel[:, [1, 2]].reshape(-1, 1)
        self.initialLegsPos = legsPos[:, [1, 2]].reshape(-1, 1)
        self.initialMarkersPos = np.vstack([marker.position.value.copy()[:, [1, 2]].reshape(-1, 1) for marker in self.markers])


    def measure_data(self):
        self.legsVel = np.vstack([mo.velocity.value.copy() for mo in self.mos])
        self.legsVel = self.legsVel[:, [1, 2]].reshape(-1, 1) - self.initialLegsVel
        self.legsPos = np.vstack([mo.position.value.copy() for mo in self.mos])
        self.legsPos = self.legsPos[:, [1, 2]].reshape(-1, 1) - self.initialLegsPos
        markersPos = np.vstack([marker.position.value.copy()[:, [1, 2]].reshape(-1, 1) for marker in self.markers])
        self.markersPos = markersPos - self.initialMarkersPos
        self.currentMotorsPos = np.array([motor.JointActuator.value.value for motor in self.motors]) - self.motorsInit


    def execute_control_at_simu_frame(self):
        motorDisplacement = np.clip(self.command - self.currentMotorsPos, -self.speedLimit, self.speedLimit)
        command = self.currentMotorsPos + motorDisplacement
        command = np.clip(command, self.motorsMin, self.motorsMax)
        for i, motor in enumerate(self.motors):
            motor.JointActuator.value.value = command[i] + self.motorsInit[i]

        self.force.forces = [[ 0, self.guiNode.forceY.value*1e2, self.guiNode.forceZ.value*1e2, 0, 0, 0, 0]]
        self.guiNode.forceY.value = 0
        self.guiNode.forceZ.value = 0

    def execute_control_at_camera_frame(self):
       raise NotImplementedError("execute_control_at_camera_frame method must be implemented in the derived class.")


    def record_data(self):
        self.legsVelList.append(self.legsVel.copy())
        self.legsPosList.append(self.legsPos.copy())
        self.markersPosList.append(self.markersPos.copy())
        self.motorsPosList.append(np.array([self.command]).reshape(-1, 1))


    def save(self):
        raise NotImplementedError("save method must be implemented in the derived class.")


    def filter(self, signal, signalFilter, cutoffFreq=1., samplingFreq=60.):
        samplingPeriod = 1 / samplingFreq
        tau = 1 / (2 * np.pi * cutoffFreq)
        alpha = samplingPeriod / (tau + samplingPeriod)
        return alpha * signal + (1 - alpha) * signalFilter
