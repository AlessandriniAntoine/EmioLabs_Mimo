import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../../")

import Sofa
from math import pi, sin, cos

from utils.header import addHeader, addSolvers

from splib3.animation import AnimationManager, animate
from splib3.loaders import getLoadingLocation
from splib3.numerics import to_radians

from parts.motor import Motor
from parts.leg import Leg
from parts.camera import Camera
from parts.centerpart import CenterPart
from utils.topology import applyTranslation, applyRotation
from utils import getColorFromFilename, getListFromArgs
from utils import RGBAColor


def arg_parse():
    import sys
    import argparse
    parser = argparse.ArgumentParser(prog=sys.argv[0],
                                     description='Simulate two legs.')

    parser.add_argument('-ln', '--legsName', type=str, nargs='*',
                        help="name of the leg (ex: blueleg, greenleg, greyleg)",
                        default='magentaleg magentaleg', dest="legsName")
    parser.add_argument('-lm', '--legsModel', type=str, nargs='*',
                        help="name of the model (beam, cosserat, or tetra)",
                        default='tetra', dest="legsModel")
    parser.add_argument('-lp', '--legsPositionOnMotor', type=str, nargs='*',
                        help="position on motor (clockwiseup, counterclockwiseup, clockwisedown, or counterclockwisedown)",
                        default='clockwisedown clockwisedown', dest="legsPositionOnMotor")

    parser.add_argument('-fps', "--framerate", type=str,
                        help="Scene frame rate",
                        default='120', dest="framerate")


    parser.add_argument('-mi', '--motorsInit', type=str, nargs='*',
                        help="Initial values for the motors (in radians). ",
                        default='0.', dest="motorsInit")
    parser.add_argument('-mx', '--motorsMax', type=str, nargs='*',
                        help="Maximum values for the motors (in radians). ",
                        default='1.57', dest="motorsMax")
    parser.add_argument('-mn', '--motorsMin', type=str, nargs='*',
                        help="Minimum values for the motors (in radians). ",
                        default='-1.57', dest="motorsMin")
    parser.add_argument("--motorCutoffFreq", type=str,
                        help="Motor cutoff frequency",
                        default='20.', dest="motorCutoffFreq")

    parser.add_argument('-ctr', "--controller", type=str,
                        help="Select the controller that will be used.",
                        default="openloop", dest="controller")
    parser.add_argument('--order', type=str,
                        help="Specify the order of the reduction",
                        default='7', dest="order")
    parser.add_argument('--useObserver', type=str,
                        help="Specify the order of the reduction",
                        default='1', dest="useObserver")
    parser.add_argument('--no-connection', dest="connection", action='store_false',
                        help="use when you want to run the simulation without the robot")


    try:
        args = parser.parse_args()
    except SystemExit:
        Sofa.msg_error(sys.argv[0], "Invalid arguments, get defaults instead.")
        args = parser.parse_args([])

    return args


def attachLoad(centerpart):
    load = centerpart.addChild("Load")
    load.addObject("MechanicalObject", template="Rigid3", position=[0, 33, 0, 0., 0, 0., 1],
                   showObject=False, showObjectScale=20)
    load.addObject("UniformMass", totalMass=0.315)
    visual = load.addChild("Visual")
    visual.addObject("MeshSTLLoader", filename="../../data/meshes/centerparts/greymass.stl",
                     translation=[0, -15, 0],
                     rotation=[-90, 90, 0])
    visual.addObject("OglModel", src=visual.MeshSTLLoader.linkpath, color=RGBAColor.grey)
    visual.addObject("RigidMapping")

    load.addObject('RigidMapping', name="load-center-rigid-mapping", input="@..", output="@.")


def createScene(rootnode):
    import myparameters

    args = arg_parse()

    ##############################################################################
    # Parameters
    ##############################################################################
    legsName = getListFromArgs(args.legsName)
    legsModel = getListFromArgs(args.legsModel)
    legsPositionOnMotor = getListFromArgs(args.legsPositionOnMotor)
    dt = 1 / float(args.framerate)
    motorsInit = [float(x) for x in getListFromArgs(args.motorsInit)]
    motorsMin = [-float(x) for x in getListFromArgs(args.motorsMin)]
    motorsMax = [float(x) for x in getListFromArgs(args.motorsMax)]

    if len(motorsInit) != 2:
        motorsInit = [motorsInit[0], motorsInit[0]]
    if len(motorsMax) != 2:
        motorsMax = [motorsMax[0], motorsMax[0]]
    if len(motorsMin) != 2:
        motorsMin = [motorsMin[0], motorsMin[0]]

    # Header of the simulation
    settings, modelling, simulation = addHeader(rootnode, withCollision=False)
    rootnode.addObject(AnimationManager(rootnode))
    rootnode.dt = dt
    rootnode.gravity = [0., -9810, 0.]
    addSolvers(simulation)
    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
    rootnode.Simulation.EulerImplicitSolver.rayleighStiffness = 1e-6

    ##############################################################################
    # Legs + Motor
    ##############################################################################
    legs = []
    motors = []
    centerPartPositions = []
    translations = []

    for i in range(2):
        angle = 2. * pi / 4. * (2. * i)
        radius = 97.5
        translation = [radius * sin(angle), 0, radius * cos(angle)]
        rotation = [0, [180, 0][i], 0]
        translations.append(translation)

    ##############################################################################
    # Motor
        motor = Motor(name="Motor" + str(i),
                    translation=translation,
                    rotation=rotation,
                    tempvisurotation=[-90, 180, 0])
        motor.Parts.MotorVisual.activated.value = False
        simulation.addChild(motor)
        motors.append(motor)

    ##############################################################################
    # Leg
        leg = Leg(name="Leg" + str(i),
                legName=legsName[i] if i < len(legsName) else legsName[0],
                positionOnMotor=legsPositionOnMotor[i] if i < len(legsPositionOnMotor) else legsPositionOnMotor[0],
                model=legsModel[i] if i < len(legsModel) else legsModel[0],
                translation=translation,
                rotation=rotation,
                youngModulus=myparameters.youngModulus,
                poissonRatio=myparameters.poissonRatio,
                )
        legs.append(leg)
        simulation.addChild(leg)

        # Attach the leg to motor
        leg.attachBase(motor.Parts, 1)

        position = list(leg.extremity.getMechanicalState().position.value[0])
        applyRotation([position], to_radians(rotation))
        applyTranslation([position], translation)
        centerPartPositions += [position]

    ##############################################################################
    # Camera
    ##############################################################################
    camera = modelling.addChild(Camera(rotation=[180, -135, 0], translation=[0, 10, 0]))

    ##############################################################################
    ## Center Part
    ##############################################################################
    centerpart = CenterPart(name="CenterPart",
                                        positions=centerPartPositions,
                                        partName="bluepart",
                                        model="beam",
                                        type="rigid",
                                        shape="circular",
                                        color=RGBAColor.blue,
                                        rotation=[0, 0, 0] if "down" in legsPositionOnMotor[0] else [180, 180, 0]
                                        )
    centerpart.addChild("Effector")
    simulation.addChild(centerpart)

    for i, leg in enumerate(legs):
        leg.attachExtremity(centerpart.attach, 2*i)

    attachLoad(centerpart)

    ##############################################################################
    # Base + Platform
    ##############################################################################
    box = modelling.addChild("Base")
    box.addObject("MeshSTLLoader",
                filename=getLoadingLocation("../../data/meshes/base-extended.stl", __file__))
    # Platform
    platform = modelling.addChild("Platform")
    platform.addObject("MeshSTLLoader",
                filename=getLoadingLocation("../../data/meshes/supports/platform.stl", __file__))
    platform.addObject("OglModel", src=platform.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.1])
    box.addObject("OglModel", src=box.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.1])

    ##############################################################################
    # Drums
    ##############################################################################
    for i in range(3):
        drum = modelling.addChild("Drum" + str(i+1))
        drum.addObject("MeshSTLLoader",
                      filename=getLoadingLocation("../../data/meshes/legmotorattachbase.stl", __file__),
                       rotation=[0, [90, 180, -90][i], 0],
                       translation=[[-100, 0, 0],[0, 0, -100],[100, 0, 0]][i])
        drum.addObject("OglModel", src=drum.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.1])

    ##############################################################################
    # Control torques
    ##############################################################################
    for i, motor in enumerate(motors):
        motor.addObject("JointConstraint", name="JointActuator", index=0, value=0, valueType="displacement", minDisplacement=-pi, maxDisplacement=pi)
        motor.JointActuator.value = motorsInit[i]

    ##############################################################################
    # Markers
    ##############################################################################
    markers = []
    legsPositionOnMotor = legsPositionOnMotor if len(legsPositionOnMotor) == 2 else legsPositionOnMotor * 2
    for i, leg in enumerate(legs):
        legmarkers = leg.leg.addChild("Markers")
        legPositionOnMotor = legsPositionOnMotor[i] if i < len(legsPositionOnMotor) else legsPositionOnMotor[0]
        signy = 1 if "up" in legPositionOnMotor else -1
        signz = -1 if i % 2 == 0 else 1
        y = 260 if i % 2  else 260
        z = -22.5 if "counter" in legPositionOnMotor else 22.5
        if i == 0:
            position = [[-5, signy * y, signz * z], [-5, 0.5 * signy * y, signz * z]]
        else:
            position = [[-5, signy * y, signz * z]]
        legmarkers.addObject("MechanicalObject",
                      position=position,
                      translation=translations[i],
                      showObject=True, showObjectScale=7, drawMode=1, showColor=[1, 0, 0, 1])
        legModel = legsModel[i] if i < len(legsModel) else legsModel[0]
        legmarkers.addObject("BarycentricMapping" if legModel == "tetra" else "SkinningMapping")
        markers.append(legmarkers.MechanicalObject)

    ##############################################################################
    # Emio Gui
    ##############################################################################


    # Open Loop
    if args.controller == "openloop":
        from scripts.openLoopController import OpenLoopController
        rootnode.addObject(OpenLoopController(legs, motors, markers, centerpart.Load,
            motorsInit, motorsMin, motorsMax, float(args.motorCutoffFreq)))
    elif args.controller == "closedloop":
        from scripts.closedLoopController import ClosedLoopController
        rootnode.addObject(ClosedLoopController(
            legs, motors, markers, centerpart.Load,
            motorsInit, motorsMin, motorsMax, float(args.motorCutoffFreq),
            int(args.order), int(args.useObserver)))

    ##############################################################################
    # Real Emio Connection
    ##############################################################################
    if not args.connection:
        print("Running simulation with connection to the real Emio robot.")
        from parts.controllers.trackercontroller import DotTracker
        rootnode.addObject(DotTracker(name="DotTracker",
                                      root=rootnode,
                                      nb_tracker=3,
                                      show_video_feed=False,
                                      track_colors=True,
                                      comp_point_cloud=False,
                                      scale=1,
                                      rotation=camera.torealrotation,
                                      translation=camera.torealtranslation))

        from parts.controllers.motorcontroller import MotorController
        rootnode.addObject(MotorController([motors[0].JointActuator.displacement, None, motors[1].JointActuator.displacement, None],
                                           name="MotorController"))

    return rootnode
