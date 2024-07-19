import React from 'react';
import * as THREE from 'three';
import { Typography } from '@mui/material';

const DroneAttitude = ({ quatX, quatY, quatZ, quatW }) => {
  // Create a quaternion using the provided values
  const quaternion = new THREE.Quaternion(quatX, quatY, quatZ, quatW);

  // Convert the quaternion to Euler angles
  const euler = new THREE.Euler().setFromQuaternion(quaternion, 'XYZ');

  // Extract roll, pitch, and yaw from the Euler angles
  const roll = euler.x * (180 / Math.PI); // Convert from radians to degrees
  const pitch = euler.y * (180 / Math.PI); // Convert from radians to degrees
  const yaw = euler.z * (180 / Math.PI); // Convert from radians to degrees

  return (
    <div>
      <Typography>Roll: {roll.toFixed(2)}°</Typography>
      <Typography>Pitch: {pitch.toFixed(2)}°</Typography>
      <Typography>Yaw: {yaw.toFixed(2)}°</Typography>
    </div>
  );
};

export default DroneAttitude;
