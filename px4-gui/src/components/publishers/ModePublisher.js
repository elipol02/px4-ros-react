import React, { useEffect, useState } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { Button, Menu, MenuItem, Tooltip } from '@mui/material';
import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';

const modeDisplayNames = {
  'MANUAL': 'MANUAL',
  'ALTCTL': 'ALTCTL',
  'POSCTL': 'POSCTL',
  'AUTO.MISSION': 'MISSION',
  'AUTO.LOITER': 'LOITER',
  'AUTO.RTL': 'RTL',
  'AUTO.TAKEOFF': 'TAKEOFF',
  'AUTO.LAND': 'LAND',
  'OFFBOARD': 'OFFBOARD',
  'STABILIZED': 'STABILIZED',
  'RATTITUDE': 'RATTITUDE',
  'ACRO': 'ACRO',
};

const ModePublisher = ({ initialMode, isSmallScreen }) => {
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

  const getDisplayMode = (mode) => {
    return modeDisplayNames[mode] || mode;
  };

  return (
    <div style={{ margin: '5px' }}>
      <Tooltip title="Flight Mode" placement="bottom">
        <Button
          variant="contained"
          color="secondary"
          onClick={handleButtonClick}
          endIcon={<ArrowDropDownIcon />}
          sx={{ paddingX: isSmallScreen ? "8px" : "16px" }}
        >
          {getDisplayMode(initialMode)}
        </Button>
      </Tooltip>
      <Menu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleClose}
        transitionDuration={0}
      >
        {Object.keys(modeDisplayNames).map((modeKey) => (
          <MenuItem
            key={modeKey}
            onClick={(event) => handleMenuItemClick(event, modeKey)}
            sx={{ padding: '4px 16px' }}
          >
            {modeDisplayNames[modeKey]}
          </MenuItem>
        ))}
      </Menu>
    </div>
  );
};

export default ModePublisher;
