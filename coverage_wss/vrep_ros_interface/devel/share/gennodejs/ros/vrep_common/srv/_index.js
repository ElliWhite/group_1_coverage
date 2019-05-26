
"use strict";

let simRosGetObjectSelection = require('./simRosGetObjectSelection.js')
let simRosDisableSubscriber = require('./simRosDisableSubscriber.js')
let simRosReadDistance = require('./simRosReadDistance.js')
let simRosClearFloatSignal = require('./simRosClearFloatSignal.js')
let simRosSetBooleanParameter = require('./simRosSetBooleanParameter.js')
let simRosSetStringSignal = require('./simRosSetStringSignal.js')
let simRosGetUISlider = require('./simRosGetUISlider.js')
let simRosEndDialog = require('./simRosEndDialog.js')
let simRosRemoveModel = require('./simRosRemoveModel.js')
let simRosSetVisionSensorImage = require('./simRosSetVisionSensorImage.js')
let simRosGetFloatingParameter = require('./simRosGetFloatingParameter.js')
let simRosGetStringSignal = require('./simRosGetStringSignal.js')
let simRosSetJointForce = require('./simRosSetJointForce.js')
let simRosRemoveUI = require('./simRosRemoveUI.js')
let simRosReadCollision = require('./simRosReadCollision.js')
let simRosGetObjects = require('./simRosGetObjects.js')
let simRosDisablePublisher = require('./simRosDisablePublisher.js')
let simRosClearStringSignal = require('./simRosClearStringSignal.js')
let simRosSynchronousTrigger = require('./simRosSynchronousTrigger.js')
let simRosGetObjectFloatParameter = require('./simRosGetObjectFloatParameter.js')
let simRosGetIntegerParameter = require('./simRosGetIntegerParameter.js')
let simRosSetObjectIntParameter = require('./simRosSetObjectIntParameter.js')
let simRosSetUISlider = require('./simRosSetUISlider.js')
let simRosSetIntegerParameter = require('./simRosSetIntegerParameter.js')
let simRosSetIntegerSignal = require('./simRosSetIntegerSignal.js')
let simRosSetObjectParent = require('./simRosSetObjectParent.js')
let simRosSetJointTargetPosition = require('./simRosSetJointTargetPosition.js')
let simRosSetArrayParameter = require('./simRosSetArrayParameter.js')
let simRosGetObjectChild = require('./simRosGetObjectChild.js')
let simRosSetSphericalJointMatrix = require('./simRosSetSphericalJointMatrix.js')
let simRosStartSimulation = require('./simRosStartSimulation.js')
let simRosSetJointPosition = require('./simRosSetJointPosition.js')
let simRosGetModelProperty = require('./simRosGetModelProperty.js')
let simRosPauseSimulation = require('./simRosPauseSimulation.js')
let simRosSetUIButtonLabel = require('./simRosSetUIButtonLabel.js')
let simRosLoadUI = require('./simRosLoadUI.js')
let simRosGetUIHandle = require('./simRosGetUIHandle.js')
let simRosSetModelProperty = require('./simRosSetModelProperty.js')
let simRosAddStatusbarMessage = require('./simRosAddStatusbarMessage.js')
let simRosGetUIEventButton = require('./simRosGetUIEventButton.js')
let simRosSetObjectFloatParameter = require('./simRosSetObjectFloatParameter.js')
let simRosClearIntegerSignal = require('./simRosClearIntegerSignal.js')
let simRosAuxiliaryConsoleOpen = require('./simRosAuxiliaryConsoleOpen.js')
let simRosLoadModel = require('./simRosLoadModel.js')
let simRosLoadScene = require('./simRosLoadScene.js')
let simRosGetObjectPose = require('./simRosGetObjectPose.js')
let simRosGetArrayParameter = require('./simRosGetArrayParameter.js')
let simRosGetObjectGroupData = require('./simRosGetObjectGroupData.js')
let simRosGetCollisionHandle = require('./simRosGetCollisionHandle.js')
let simRosGetAndClearStringSignal = require('./simRosGetAndClearStringSignal.js')
let simRosGetDialogResult = require('./simRosGetDialogResult.js')
let simRosGetJointMatrix = require('./simRosGetJointMatrix.js')
let simRosTransferFile = require('./simRosTransferFile.js')
let simRosGetBooleanParameter = require('./simRosGetBooleanParameter.js')
let simRosGetObjectHandle = require('./simRosGetObjectHandle.js')
let simRosGetStringParameter = require('./simRosGetStringParameter.js')
let simRosGetDialogInput = require('./simRosGetDialogInput.js')
let simRosAppendStringSignal = require('./simRosAppendStringSignal.js')
let simRosDisplayDialog = require('./simRosDisplayDialog.js')
let simRosSetUIButtonProperty = require('./simRosSetUIButtonProperty.js')
let simRosSetJointState = require('./simRosSetJointState.js')
let simRosRemoveObject = require('./simRosRemoveObject.js')
let simRosGetInfo = require('./simRosGetInfo.js')
let simRosGetVisionSensorImage = require('./simRosGetVisionSensorImage.js')
let simRosCopyPasteObjects = require('./simRosCopyPasteObjects.js')
let simRosEnableSubscriber = require('./simRosEnableSubscriber.js')
let simRosAuxiliaryConsoleClose = require('./simRosAuxiliaryConsoleClose.js')
let simRosCloseScene = require('./simRosCloseScene.js')
let simRosAuxiliaryConsoleShow = require('./simRosAuxiliaryConsoleShow.js')
let simRosGetIntegerSignal = require('./simRosGetIntegerSignal.js')
let simRosEnablePublisher = require('./simRosEnablePublisher.js')
let simRosReadVisionSensor = require('./simRosReadVisionSensor.js')
let simRosGetFloatSignal = require('./simRosGetFloatSignal.js')
let simRosGetJointState = require('./simRosGetJointState.js')
let simRosGetLastErrors = require('./simRosGetLastErrors.js')
let simRosSetFloatSignal = require('./simRosSetFloatSignal.js')
let simRosCallScriptFunction = require('./simRosCallScriptFunction.js')
let simRosAuxiliaryConsolePrint = require('./simRosAuxiliaryConsolePrint.js')
let simRosSetObjectSelection = require('./simRosSetObjectSelection.js')
let simRosSynchronous = require('./simRosSynchronous.js')
let simRosReadForceSensor = require('./simRosReadForceSensor.js')
let simRosSetObjectPose = require('./simRosSetObjectPose.js')
let simRosSetObjectQuaternion = require('./simRosSetObjectQuaternion.js')
let simRosGetVisionSensorDepthBuffer = require('./simRosGetVisionSensorDepthBuffer.js')
let simRosSetObjectPosition = require('./simRosSetObjectPosition.js')
let simRosStopSimulation = require('./simRosStopSimulation.js')
let simRosReadProximitySensor = require('./simRosReadProximitySensor.js')
let simRosCreateDummy = require('./simRosCreateDummy.js')
let simRosGetDistanceHandle = require('./simRosGetDistanceHandle.js')
let simRosBreakForceSensor = require('./simRosBreakForceSensor.js')
let simRosGetObjectIntParameter = require('./simRosGetObjectIntParameter.js')
let simRosGetObjectParent = require('./simRosGetObjectParent.js')
let simRosSetJointTargetVelocity = require('./simRosSetJointTargetVelocity.js')
let simRosEraseFile = require('./simRosEraseFile.js')
let simRosGetCollectionHandle = require('./simRosGetCollectionHandle.js')
let simRosGetUIButtonProperty = require('./simRosGetUIButtonProperty.js')
let simRosSetFloatingParameter = require('./simRosSetFloatingParameter.js')

module.exports = {
  simRosGetObjectSelection: simRosGetObjectSelection,
  simRosDisableSubscriber: simRosDisableSubscriber,
  simRosReadDistance: simRosReadDistance,
  simRosClearFloatSignal: simRosClearFloatSignal,
  simRosSetBooleanParameter: simRosSetBooleanParameter,
  simRosSetStringSignal: simRosSetStringSignal,
  simRosGetUISlider: simRosGetUISlider,
  simRosEndDialog: simRosEndDialog,
  simRosRemoveModel: simRosRemoveModel,
  simRosSetVisionSensorImage: simRosSetVisionSensorImage,
  simRosGetFloatingParameter: simRosGetFloatingParameter,
  simRosGetStringSignal: simRosGetStringSignal,
  simRosSetJointForce: simRosSetJointForce,
  simRosRemoveUI: simRosRemoveUI,
  simRosReadCollision: simRosReadCollision,
  simRosGetObjects: simRosGetObjects,
  simRosDisablePublisher: simRosDisablePublisher,
  simRosClearStringSignal: simRosClearStringSignal,
  simRosSynchronousTrigger: simRosSynchronousTrigger,
  simRosGetObjectFloatParameter: simRosGetObjectFloatParameter,
  simRosGetIntegerParameter: simRosGetIntegerParameter,
  simRosSetObjectIntParameter: simRosSetObjectIntParameter,
  simRosSetUISlider: simRosSetUISlider,
  simRosSetIntegerParameter: simRosSetIntegerParameter,
  simRosSetIntegerSignal: simRosSetIntegerSignal,
  simRosSetObjectParent: simRosSetObjectParent,
  simRosSetJointTargetPosition: simRosSetJointTargetPosition,
  simRosSetArrayParameter: simRosSetArrayParameter,
  simRosGetObjectChild: simRosGetObjectChild,
  simRosSetSphericalJointMatrix: simRosSetSphericalJointMatrix,
  simRosStartSimulation: simRosStartSimulation,
  simRosSetJointPosition: simRosSetJointPosition,
  simRosGetModelProperty: simRosGetModelProperty,
  simRosPauseSimulation: simRosPauseSimulation,
  simRosSetUIButtonLabel: simRosSetUIButtonLabel,
  simRosLoadUI: simRosLoadUI,
  simRosGetUIHandle: simRosGetUIHandle,
  simRosSetModelProperty: simRosSetModelProperty,
  simRosAddStatusbarMessage: simRosAddStatusbarMessage,
  simRosGetUIEventButton: simRosGetUIEventButton,
  simRosSetObjectFloatParameter: simRosSetObjectFloatParameter,
  simRosClearIntegerSignal: simRosClearIntegerSignal,
  simRosAuxiliaryConsoleOpen: simRosAuxiliaryConsoleOpen,
  simRosLoadModel: simRosLoadModel,
  simRosLoadScene: simRosLoadScene,
  simRosGetObjectPose: simRosGetObjectPose,
  simRosGetArrayParameter: simRosGetArrayParameter,
  simRosGetObjectGroupData: simRosGetObjectGroupData,
  simRosGetCollisionHandle: simRosGetCollisionHandle,
  simRosGetAndClearStringSignal: simRosGetAndClearStringSignal,
  simRosGetDialogResult: simRosGetDialogResult,
  simRosGetJointMatrix: simRosGetJointMatrix,
  simRosTransferFile: simRosTransferFile,
  simRosGetBooleanParameter: simRosGetBooleanParameter,
  simRosGetObjectHandle: simRosGetObjectHandle,
  simRosGetStringParameter: simRosGetStringParameter,
  simRosGetDialogInput: simRosGetDialogInput,
  simRosAppendStringSignal: simRosAppendStringSignal,
  simRosDisplayDialog: simRosDisplayDialog,
  simRosSetUIButtonProperty: simRosSetUIButtonProperty,
  simRosSetJointState: simRosSetJointState,
  simRosRemoveObject: simRosRemoveObject,
  simRosGetInfo: simRosGetInfo,
  simRosGetVisionSensorImage: simRosGetVisionSensorImage,
  simRosCopyPasteObjects: simRosCopyPasteObjects,
  simRosEnableSubscriber: simRosEnableSubscriber,
  simRosAuxiliaryConsoleClose: simRosAuxiliaryConsoleClose,
  simRosCloseScene: simRosCloseScene,
  simRosAuxiliaryConsoleShow: simRosAuxiliaryConsoleShow,
  simRosGetIntegerSignal: simRosGetIntegerSignal,
  simRosEnablePublisher: simRosEnablePublisher,
  simRosReadVisionSensor: simRosReadVisionSensor,
  simRosGetFloatSignal: simRosGetFloatSignal,
  simRosGetJointState: simRosGetJointState,
  simRosGetLastErrors: simRosGetLastErrors,
  simRosSetFloatSignal: simRosSetFloatSignal,
  simRosCallScriptFunction: simRosCallScriptFunction,
  simRosAuxiliaryConsolePrint: simRosAuxiliaryConsolePrint,
  simRosSetObjectSelection: simRosSetObjectSelection,
  simRosSynchronous: simRosSynchronous,
  simRosReadForceSensor: simRosReadForceSensor,
  simRosSetObjectPose: simRosSetObjectPose,
  simRosSetObjectQuaternion: simRosSetObjectQuaternion,
  simRosGetVisionSensorDepthBuffer: simRosGetVisionSensorDepthBuffer,
  simRosSetObjectPosition: simRosSetObjectPosition,
  simRosStopSimulation: simRosStopSimulation,
  simRosReadProximitySensor: simRosReadProximitySensor,
  simRosCreateDummy: simRosCreateDummy,
  simRosGetDistanceHandle: simRosGetDistanceHandle,
  simRosBreakForceSensor: simRosBreakForceSensor,
  simRosGetObjectIntParameter: simRosGetObjectIntParameter,
  simRosGetObjectParent: simRosGetObjectParent,
  simRosSetJointTargetVelocity: simRosSetJointTargetVelocity,
  simRosEraseFile: simRosEraseFile,
  simRosGetCollectionHandle: simRosGetCollectionHandle,
  simRosGetUIButtonProperty: simRosGetUIButtonProperty,
  simRosSetFloatingParameter: simRosSetFloatingParameter,
};
