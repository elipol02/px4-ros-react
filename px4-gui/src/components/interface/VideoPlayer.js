// src/RTSPStreamViewer.js
import React, { useEffect, useState } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { Typography, Box } from '@mui/material';

const VideoPlayer = () => {
  const { ros, connected } = useROS();
  const [image, setImage] = useState('');

  useEffect(() => {
    if (connected && ros) {
      const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/rtsp_stream',
        messageType: 'sensor_msgs/Image',
      });

      imageTopic.subscribe((message) => {
        // Convert ROS image message to base64
        const uint8Array = new Uint8Array(message.data);
        const binaryString = uint8Array.reduce((data, byte) => {
          return data + String.fromCharCode(byte);
        }, '');
        const base64String = window.btoa(binaryString);
        const imgData = `data:image/jpeg;base64,${base64String}`;
        setImage(imgData);
      });

      return () => {
        imageTopic.unsubscribe();
      };
    }
  }, [connected, ros]);

  return (
    <Box>
      <Typography color="secondary" >RTSP Stream Viewer</Typography>
      {image ? ( 
        <img src={image} alt="RTSP Stream" style={{ width: '200px' }} /> 
      ) : ( 
        <Typography color="secondary">Loading...</Typography>
      )}
    </Box>
  );
};

export default VideoPlayer;
