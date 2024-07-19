import React, { useEffect, useState } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { Button } from '@mui/material';

const ArmDisarmPublisher = ({ armed, isSmallScreen }) => {
  const { ros, connected } = useROS();
  const [armDisarmPublisher, setArmDisarmPublisher] = useState(null);

  useEffect(() => {
    if (connected && ros) {
      const publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/arm',
        messageType: 'std_msgs/String',
      });
      setArmDisarmPublisher(publisher);
      console.log('Topic initialized: /arm');
    }
  }, [connected, ros]);

  const handlePublish = (command) => {
    if (!connected) {
      console.error('Not connected to ROS');
      return;
    }

    if (!armDisarmPublisher) {
      console.error('Topic armDisarmPublisher not initialized');
      return;
    }

    const message = new ROSLIB.Message({
      data: command,
    });

    armDisarmPublisher.publish(message);
    console.log('Published to /arm:', command);
  };


  return (
    <div style={{ margin:"5px" }}>
      {armed ? (
        <Button variant="contained" color="error" sx={{ paddingX: isSmallScreen ? "8px" : "16px" }} onClick={() => handlePublish('disarm')}>Disarm</Button>
      ):(
        <Button variant="contained" color="primary" sx={{ paddingX: isSmallScreen ? "8px" : "16px" }} onClick={() => handlePublish('arm')}>Arm</Button>
      )}
    </div>
  );
};

export default ArmDisarmPublisher;
