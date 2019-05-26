
"use strict";

let ProximitySensorData = require('./ProximitySensorData.js');
let ScriptFunctionCallData = require('./ScriptFunctionCallData.js');
let VisionSensorData = require('./VisionSensorData.js');
let VisionSensorDepthBuff = require('./VisionSensorDepthBuff.js');
let VrepInfo = require('./VrepInfo.js');
let JointSetStateData = require('./JointSetStateData.js');
let ForceSensorData = require('./ForceSensorData.js');
let ObjectGroupData = require('./ObjectGroupData.js');

module.exports = {
  ProximitySensorData: ProximitySensorData,
  ScriptFunctionCallData: ScriptFunctionCallData,
  VisionSensorData: VisionSensorData,
  VisionSensorDepthBuff: VisionSensorDepthBuff,
  VrepInfo: VrepInfo,
  JointSetStateData: JointSetStateData,
  ForceSensorData: ForceSensorData,
  ObjectGroupData: ObjectGroupData,
};
