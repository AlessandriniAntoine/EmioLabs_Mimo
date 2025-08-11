import os
import time
import json
import numpy as np
import tkinter as tk
import multiprocessing
import cv2 as cv
from emioapi import EmioMotors
from emioapi._depthcamera import DepthCamera

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")


nbMarkers = 3
ControlMode = {"Open Loop": False, "Closed Loop": True}

###########################################################################
# TKINTER APP
###########################################################################
class ClosedLoopLinearApp:
    def __init__(self, root, sharedMotorsPos, sharedRefPos, sharedStart, sharedControlMode, sharedRecord, sharedSave):
        self.root = root
        self.root.title('Emio Linear Closed Loop Control')
        self.root.geometry("500x250")

        # shared Variables
        self.sharedMotorsPos = sharedMotorsPos
        self.sharedRefPos = sharedRefPos
        self.sharedStart = sharedStart
        self.sharedControlMode = sharedControlMode
        self.sharedRecord = sharedRecord
        self.sharedSave = sharedSave

        # app variable
        self.start = False
        self.active = True

        # Information Frame
        infoFrame = tk.Frame(root)
        infoFrame.pack()
        info = ["Start", "Closed Loop", "Active"]
        infoValue = ["False", "False", "True"]
        self.labelInfo = {}
        for i, item in enumerate(info):
            self.labelInfo[item] = tk.Label(infoFrame, text=f"{item}: {infoValue[i]}", font=("Arial", 12))
            self.labelInfo[item].pack(side="left", padx=5, pady=5)

        # Motor Sliders
        motorLabelFrame = tk.Frame(root)
        motorLabelFrame.pack()
        motorSliderFrame = tk.Frame(root)
        motorSliderFrame.pack()

        self.labelMotor1 = tk.Label(motorLabelFrame, text='Motor 1: 0.00 (rad)')
        self.labelMotor1.pack(side="left", padx=3, pady=5)
        self.sliderMotor1 = tk.Scale(motorSliderFrame, from_=-100, to=100, orient='horizontal', command=self.sliderMotor1Action)
        self.sliderMotor1.pack(side="left", padx=3, pady=5)
        self.sliderMotor1.set(0)

        self.labelMotor2 = tk.Label(motorLabelFrame, text='Motor 2: 0.00 (rad)')
        self.labelMotor2.pack(side="left", padx=3, pady=5)
        self.sliderMotor2 = tk.Scale(motorSliderFrame, from_=-100, to=100, orient='horizontal', command=self.sliderMotor2Action)
        self.sliderMotor2.pack(side="left", padx=3, pady=5)
        self.sliderMotor2.set(0)

        # Reference Sliders
        refLabelFrame = tk.Frame(root)
        refLabelFrame.pack()
        refSliderFrame = tk.Frame(root)
        refSliderFrame.pack()

        self.labelRef1 = tk.Label(refLabelFrame, text='Ref 1: 0.00 (mm)')
        self.labelRef1.pack(side="left", padx=3, pady=5)
        self.sliderRef1 = tk.Scale(refSliderFrame, from_=-100, to=100, orient='horizontal', command=self.sliderRef1Action)
        self.sliderRef1.pack(side="left", padx=3, pady=5)
        self.sliderRef1.set(0)

        self.labelRef2 = tk.Label(refLabelFrame, text='Ref 2: 0.00 (mm)')
        self.labelRef2.pack(side="left", padx=3, pady=5)
        self.sliderRef2 = tk.Scale(refSliderFrame, from_=-100, to=100, orient='horizontal', command=self.sliderRef2Action)
        self.sliderRef2.pack(side="left", padx=3, pady=5)
        self.sliderRef2.set(0)

        # Buttons
        buttonFrame = tk.Frame(root)
        buttonFrame.pack(pady=10)
        self.startButton = tk.Button(buttonFrame, text="Start", command=self.startAction)
        self.startButton.pack(side='left', padx=3)

        self.controlModeButton = tk.Button(buttonFrame, text="Control Mode", command=self.controlModeAction)
        self.controlModeButton.pack(side='right', padx=3)

        self.activeButton = tk.Button(buttonFrame, text="Active", command=self.activeAction)
        self.activeButton.pack(side='right', padx=3)

        self.recordingButton = tk.Button(buttonFrame, text="Record", command=self.recordAction)
        self.recordingButton.pack(side='right', padx=3)

        self.saveButton = tk.Button(buttonFrame, text="Save", command=self.saveAction)
        self.saveButton.pack(side='right', padx=3)


    def sliderMotor1Action(self, val):
        motorPos = int(val) * np.pi / 2 * 1 / 100
        self.labelMotor1.config(text=f'Motor 1: {motorPos:.2f} (rad)')
        with self.sharedMotorsPos.get_lock():
            if self.active:
                self.sharedMotorsPos[0] = motorPos

    def sliderMotor2Action(self, val):
        motorPos = int(val) * np.pi / 2 * 1 / 100
        self.labelMotor2.config(text=f'Motor 2: {motorPos:.2f} (rad)')
        with self.sharedMotorsPos.get_lock():
            if self.active:
                self.sharedMotorsPos[1] = motorPos

    def sliderRef1Action(self, val):
        refPos = float(val)
        self.labelRef1.config(text=f'Ref 1: {refPos:.2f} (mm)')
        with self.sharedRefPos.get_lock():
            if self.active:
                self.sharedRefPos[0] = refPos

    def sliderRef2Action(self, val):
        refPos = float(val)
        self.labelRef2.config(text=f'Ref 2: {refPos:.2f} (mm)')
        with self.sharedRefPos.get_lock():
            if self.active:
                self.sharedRefPos[1] = refPos

    def startAction(self):
        self.start = True
        with self.sharedStart.get_lock():
            self.sharedStart.value = True
            self.labelInfo["Start"].config(text="Start: True")

    def controlModeAction(self):
        with self.sharedControlMode.get_lock():
            if self.sharedControlMode.value == ControlMode["Open Loop"]:
                self.sharedControlMode.value = ControlMode["Closed Loop"]
                self.labelInfo["Closed Loop"].config(text="Closed Loop: True")
            else:
                self.sharedControlMode.value = ControlMode["Open Loop"]
                self.labelInfo["Closed Loop"].config(text="Closed Loop: False")

    def activeAction(self):
        self.active = not self.active
        if self.active:
            self.labelInfo["Active"].config(text="Active: True")
            if self.sharedControlMode.value == ControlMode["Open Loop"]:
                self.sharedMotorsPos[0] = self.sliderMotor1.get() * np.pi / 2 * 1 / 100
                self.sharedMotorsPos[1] = self.sliderMotor2.get() * np.pi / 2 * 1 / 100
            else:
                self.sharedRefPos[0] = self.sliderRef1.get()
                self.sharedRefPos[1] = self.sliderRef2.get()
        else:
            self.labelInfo["Active"].config(text="Active: False")


    def recordAction(self):
        with self.sharedRecord.get_lock():
            self.sharedRecord.value = not self.sharedRecord.value
            if self.sharedRecord.value:
                self.active = True
                if self.sharedControlMode.value == ControlMode["Open Loop"]:
                    self.sharedMotorsPos[0] = self.sliderMotor1.get() * np.pi / 2 * 1 / 100
                    self.sharedMotorsPos[1] = self.sliderMotor2.get() * np.pi / 2 * 1 / 100
                else:
                    self.sharedRefPos[0] = self.sliderRef1.get()
                    self.sharedRefPos[1] = self.sliderRef2.get()
                print("Recording started...")

    def saveAction(self):
        with self.sharedSave.get_lock():
            self.sharedSave.value = True

    def close_app(self):
        self.root.destroy()


############################################################################
# Low-pass filter function
#############################################################################
def filterFirstOder(signal, signalFiltered, cutoffFreq=1., samplingFreq=60.):
    """Apply a first-order low-pass filter to the signal."""
    timeConstant = 1 / (2 * np.pi * cutoffFreq)
    samplingPeriod = 1 / samplingFreq
    alpha = samplingPeriod / (timeConstant + samplingPeriod)
    signalFiltered = alpha * signal + (1 - alpha) * signalFiltered
    return signalFiltered

############################################################################
# Process
#############################################################################
def processCamera(trackerPos, event):
    """Update tracker position."""
    json_path = os.path.join(data_path, "hardware", "cameraparameter.json")
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Camera parameter file not found at {json_path}")
    with open(json_path, 'r') as fp:
        json_parameters = json.load(fp)

    camera = DepthCamera(
        show_video_feed=True,
        tracking=True,
        compute_point_cloud=False,
        parameter=json_parameters)
    while True:
        camera.update()
        print(f"Size of trackers_pos: {len(camera.trackers_pos)}")
        if len(camera.trackers_pos) == nbMarkers:
            with trackerPos.get_lock():
                trackerPos[:] = np.array(camera.trackers_pos).flatten()
        event.set()

        k = cv.waitKey(1)
        if k == ord('q'):
            camera.quit()
            break

def processSlider(sharedMotorsPos, sharedRefPos,
                  sharedStart, sharedControlMode, sharedRecord, sharedSave):
    root = tk.Tk()
    app = ClosedLoopLinearApp(root, sharedMotorsPos, sharedRefPos,
        sharedStart, sharedControlMode, sharedRecord, sharedSave)
    root.protocol("WM_DELETE_WINDOW", app.close_app)
    root.mainloop()

def processController(trackerPos, sharedMotorsPos, sharedRefPos, sharedStart, sharedControlMode, sharedRecord, sharedSave, event):

    motors = EmioMotors()
    while not motors.open():
        print("Waiting for motors to open...")
        time.sleep(1)
    print("Motors opened successfully.")

    # Control mode
    counterRecord = 0
    maxRecord = 400
    start = False
    cutoffFreqMeasure = 90.  # Hz
    cutoffFreqMotor = 10.  # Hz
    samplingFreq = 60.  # Hz

    # Motors Limits
    initAngle = 0.25 * np.pi  # rad
    upperAngle = 0.8 * np.pi / 2
    lowerAngle = - np.pi / 4

    # control data
    order = 4
    modelPath = os.path.join(data_path, "models", f"order{order}.npz")
    controlPath = os.path.join(data_path, "control", f"order{order}.npz")
    observerPath = os.path.join(data_path, "control", f"order{order}_obs.npz")
    if not os.path.exists(modelPath):
        raise FileNotFoundError(f"Model data file {modelPath} does not exist. Please run the identification script first.")
    if not os.path.exists(controlPath):
        raise FileNotFoundError(f"Closed loop data file {controlPath} does not exist. Please run the controller script first.")
    if not os.path.exists(observerPath):
        raise FileNotFoundError(f"Observer data file {observerPath} does not exist. Please run the observer script first.")

    dataModel = np.load(modelPath)
    A = dataModel["stateMatrix"]
    B = dataModel["inputMatrix"]
    C = dataModel["outputMatrix"]

    dataCl = np.load(controlPath)
    K = dataCl["feedbackGain"]
    G = dataCl["feedForwardGain"]

    dataObs = np.load(observerPath)
    L = dataObs["observerGain"]
    observerState = np.zeros((2*order, 1))

    # transformation matrix
    tfMatrix = np.load(os.path.join(data_path, "hardware", "camera2sofa.npz"))["transform"]

    # Initialize variables
    initialMarkersPos = np.zeros((3*nbMarkers, ))
    measureFiltered = np.zeros((3*nbMarkers, ))
    markersPosPrev = np.zeros((3*nbMarkers, ))
    motorsPos = np.zeros((2,))
    motorsPosPrev = np.zeros((2,))
    currentMotorsPos = np.zeros((2,))

    # recorded data
    markersPositions = []
    motorsPositions = []
    referenceList = []
    outputList = []
    observerStatesList = []
    observerOutputList = []
    controlModeList = []

    motors.angles = [initAngle, 0, initAngle, 0]

    while True:
        event.wait()
        event.clear()

        # Measurements
        with trackerPos.get_lock():
            measure = np.array(trackerPos[:])

        # order the markers
        pos = measure.reshape(nbMarkers, 3)
        i_xmax = np.argmax(pos[:, 0])
        i_rest = [i for i in range(nbMarkers) if i != i_xmax]
        i_sorted_y = sorted(i_rest, key=lambda i: pos[i, 1])
        new_order = [i_sorted_y[1], i_xmax, i_sorted_y[0]]
        pos_sorted = pos[new_order]
        measure = pos_sorted.flatten()

        # Filter the measurements
        measureFiltered = filterFirstOder(measure, measureFiltered,
                                          cutoffFreq=cutoffFreqMeasure, samplingFreq=samplingFreq)

        angles = motors.angles
        if angles is not None:
            currentMotorsPos = np.array([angles[0], angles[2]]) - initAngle
            # print(f"Current motors position: {currentMotorsPos}")

        if start:
            markersPos = measureFiltered - initialMarkersPos
            if np.linalg.norm(markersPos - markersPosPrev) > 70:
                print("Filtering markers ...")
                markersPos = markersPosPrev.copy()

            output = (tfMatrix @ markersPos.reshape(-1,1))[[x for i in range(nbMarkers) for x in [3*i+1, 3*i+2]]]
            outputPrev = (tfMatrix @ markersPosPrev.reshape(-1,1))[[x for i in range(nbMarkers) for x in [3*i+1, 3*i+2]]]

            # observer update
            cmd = np.array([currentMotorsPos]).reshape(-1, 1)
            observerOutput = C @ observerState
            observerState = A @ observerState + B @ cmd + L @ (outputPrev - observerOutput)

            ref = np.array([sharedRefPos[:]]).reshape(-1, 1)

            if sharedControlMode.value == ControlMode["Open Loop"]:
                with sharedMotorsPos.get_lock():
                    command = np.array(sharedMotorsPos[:])
            else:
                command = G @ ref - K @ observerState
                command = command.flatten()



            # apply the motor position
            motorsPos = filterFirstOder(command, motorsPos, cutoffFreq=cutoffFreqMotor, samplingFreq=samplingFreq)
            motorsPos = np.clip(motorsPos, lowerAngle, upperAngle)
            dxlCommand = motorsPos + initAngle
            if (motorsPosPrev - motorsPos).any():
                motors.angles = [dxlCommand[0], 0, dxlCommand[1], 0]
                motorsPosPrev = motorsPos

            # save the data
            with sharedRecord.get_lock():
                if sharedRecord.value and counterRecord < maxRecord:
                    markersPositions.append(markersPos.flatten())
                    motorsPositions.append(currentMotorsPos)
                    referenceList.append(ref.flatten())
                    outputList.append(output.flatten())
                    observerStatesList.append(observerState.flatten())
                    observerOutputList.append(observerOutput.flatten())
                    controlModeList.append(sharedControlMode.value)
                    counterRecord +=1

            if counterRecord == maxRecord:
                print("Recording finished")
                sharedRecord.value = False
                counterRecord = 0

            markersPosPrev = markersPos.copy()
        else:
            motorsPosPrev = currentMotorsPos
            initialMarkersPos = measureFiltered.copy()


        if sharedSave.value:
            print("Data saved...")
            np.savez(os.path.join(data_path, "hardware", "closedLoop.npz"),
                markersPos=markersPositions,
                motorsPos=motorsPositions,
                reference=referenceList,
                outputsPos=outputList,
                observerState=observerStatesList,
                observerOutput=observerOutputList,
                controlMode=controlModeList,
                initialMarkersPos=initialMarkersPos,
            )
            sharedSave.value = False
        if sharedStart.value and not start:
            start = True
            markersPosPrev = measureFiltered - initialMarkersPos


def main():
    import time

    # shared variables
    trackerPos = multiprocessing.Array("d", 3 * nbMarkers * [0.])
    sharedMotorsPos = multiprocessing.Array("d", 2 * [0.0])
    sharedRefPos = multiprocessing.Array("d", 2 * [0.0])
    sharedStart = multiprocessing.Value("i", False)
    sharedControlMode = multiprocessing.Value("i", False)
    sharedRecord = multiprocessing.Value("i", False)
    sharedSave = multiprocessing.Value("i", False)
    event = multiprocessing.Event()

    # Create the GUI application


    # Create processes
    p1 = multiprocessing.Process(target=processCamera, args=(trackerPos, event))
    p2 = multiprocessing.Process(target=processSlider, args=(sharedMotorsPos, sharedRefPos,
        sharedStart, sharedControlMode, sharedRecord, sharedSave))
    p3 = multiprocessing.Process(target=processController, args=(
        trackerPos, sharedMotorsPos, sharedRefPos,
        sharedStart, sharedControlMode, sharedRecord, sharedSave, event
    ))

    p1.start()
    p2.start()
    p3.start()

    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        p1.terminate()
        p2.terminate()
        p3.terminate()


def createScene(root):
    """Create the scene for the direct control application."""
    main()

if __name__ == "__main__":
    main()
