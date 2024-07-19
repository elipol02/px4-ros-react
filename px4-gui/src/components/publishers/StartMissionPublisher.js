import React, { useEffect, useState } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { Button, Tooltip, Box } from '@mui/material';
import VideogameAssetIcon from '@mui/icons-material/VideogameAsset';
import RouteIcon from '@mui/icons-material/Route';

const StartMissionPublisher = ({ isSmallScreen }) => {
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
    <Box sx={{ display: 'flex', flexDirection: 'row' }}>
      <Tooltip title="Toggle waypoint mode" placement="bottom">
        <Button
          variant="contained"
          color="tertiary"
          onClick={handlePublish}
          sx={{ 
            margin: '5px',
            paddingX: isSmallScreen ? "8px" : "16px",
            minWidth: 0,
          }}
          disabled={isMissionStarted}
        >
          <RouteIcon />
        </Button>
      </Tooltip>
      <Tooltip title="Toggle joystick mode" placement="bottom">
        <Button
          variant="contained"
          color="tertiary"
          onClick={handlePublish}
          sx={{ 
            margin: '5px',
            paddingX: isSmallScreen ? "8px" : "16px",
            minWidth: 0,
          }}
          disabled={!isMissionStarted}
        >
          <VideogameAssetIcon />
        </Button>
      </Tooltip>
    </Box>
  );
};

export default StartMissionPublisher;
