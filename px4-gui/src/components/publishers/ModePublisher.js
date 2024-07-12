import React, { useEffect, useState } from 'react';
import { useROS } from '../ROSConnection';
import ROSLIB from 'roslib';
import { Button, Menu, MenuItem } from '@mui/material';
import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';

const ModePublisher = ({ initialMode }) => {
  const { ros, connected } = useROS();
  const [modePublisher, setModePublisher] = useState(null);
  const [mode, setMode] = useState(initialMode || '');
  const [anchorEl, setAnchorEl] = useState(null);

  useEffect(() => {
    if (connected && ros) {
      const publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/modeset',
        messageType: 'std_msgs/String',
      });
      setModePublisher(publisher);
      console.log('Topic initialized: /modeset');
    }
  }, [connected, ros]);

  const handlePublish = (selectedMode) => {
    if (!connected) {
      console.error('Not connected to ROS');
      return;
    }

    if (!modePublisher) {
      console.error('Topic modePublisher not initialized');
      return;
    }

    const message = new ROSLIB.Message({
      data: selectedMode,
    });

    modePublisher.publish(message);
    console.log('Published to /modeset:', selectedMode);
  };

  const handleButtonClick = (event) => {
    setAnchorEl(event.currentTarget);
  };

  const handleMenuItemClick = (event, selectedMode) => {
    setMode(selectedMode);
    handlePublish(selectedMode);
    setAnchorEl(null);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <div style={{ margin: '5px' }}>
      <Button
        variant="contained"
        size="large"
        onClick={handleButtonClick}
        endIcon={<ArrowDropDownIcon/>}
      >
        {initialMode ? initialMode : 'Select Mode'}
      </Button>
      <Menu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleClose}
        transitionDuration={0}
      >
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'MANUAL')}>MANUAL</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'ALTCTL')}>ALTCTL</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'POSCTL')}>POSCTL</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'AUTO.MISSION')}>AUTO.MISSION</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'AUTO.LOITER')}>AUTO.LOITER</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'AUTO.RTL')}>AUTO.RTL</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'AUTO.TAKEOFF')}>AUTO.TAKEOFF</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'AUTO.LAND')}>AUTO.LAND</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'OFFBOARD')}>OFFBOARD</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'STABILIZED')}>STABILIZED</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'RATTITUDE')}>RATTITUDE</MenuItem>
        <MenuItem onClick={(event) => handleMenuItemClick(event, 'ACRO')}>ACRO</MenuItem>

        {/* Add more modes as needed */}
      </Menu>
    </div>
  );
};

export default ModePublisher;
