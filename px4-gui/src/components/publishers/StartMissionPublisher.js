import React, { useEffect, useState } from 'react';
import { useROS } from '../ROSConnection';
import ROSLIB from 'roslib';
import { Button } from '@mui/material';

const StartMissionPublisher = () => {
  const { ros, connected } = useROS();
  const [startMissionPublisher, setStartMissionPublisher] = useState(null);
  const [isMissionStarted, setIsMissionStarted] = useState(false);

  useEffect(() => {
    if (connected && ros) {
      const publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/startmission',
        messageType: 'std_msgs/String',
      });
      setStartMissionPublisher(publisher);
      console.log('Topic initialized: /startmission');
    }
  }, [connected, ros]);

  const handlePublish = () => {
    if (!connected) {
      console.error('Not connected to ROS');
      return;
    }

    if (!startMissionPublisher) {
      console.error('Topic startMissionPublisher not initialized');
      return;
    }

    const messageData = isMissionStarted ? 'stop' : 'start';

    const startMessage = new ROSLIB.Message({
      data: messageData,
    });

    startMissionPublisher.publish(startMessage);
    console.log(`Published to /startmission: ${messageData}`);

    // Toggle the state
    setIsMissionStarted(!isMissionStarted);
  };

  return (
    <Button
      variant="contained"
      color="primary"
      onClick={handlePublish}
      style={{ margin: '5px' }}
    >
      {isMissionStarted ? 'Switch to Joystick' : 'Switch to Mission'}
    </Button>
  );
};

export default StartMissionPublisher;
