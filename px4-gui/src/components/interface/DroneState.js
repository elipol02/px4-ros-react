import React from 'react';
import { Typography, Box } from '@mui/material';
import BatteryFullIcon from '@mui/icons-material/BatteryFull';
import SatelliteAltIcon from '@mui/icons-material/SatelliteAlt';

// Define mappings for system_status and landed_state values to text
const SYSTEM_STATUS_TEXT = {
  0: 'NOT READY',
  1: 'BOOT',
  2: 'CALIBRATING',
  3: 'READY',
  4: 'ACTIVE',
  5: 'CRITICAL',
  6: 'EMERGENCY',
  7: 'POWEROFF',
};

const LANDED_STATE_TEXT = {
  0: 'UNDEFINED',
  1: 'LANDED',
  2: 'FLYING',
  3: 'TAKING OFF',
  4: 'LANDING',
};

// React component
const DroneStatus = ({ system_status, landed_state, battery, satellites }) => {
  const isActive = system_status === 4;

  return (
    <Box
        sx= {{
        display: 'flex',
        flexDirection: 'row',
        gap: 1,
        zIndex: 1000,
        alignItems: 'center',
        marginLeft:'10px'
      }} 
    >
      {isActive ? (
        <Typography variant="h8" sx={{ flexGrow: 1, display: 'flex', alignItems: 'center' }}>
            {LANDED_STATE_TEXT[landed_state] || 'Unknown'}
        </Typography>
      ) : (
        <Typography variant="h8" sx={{ flexGrow: 1, display: 'flex', alignItems: 'center' }}>
            {SYSTEM_STATUS_TEXT[system_status] || 'Not Connected'}
        </Typography>
      )}
      <Typography sx={{ display: 'flex', alignItems: 'center' }}>
        <BatteryFullIcon sx={{ marginRight: 0.5 }} />{battery} %
      </Typography>
      <Typography sx={{ display: 'flex', alignItems: 'center' }}>
        <SatelliteAltIcon sx={{ marginRight: 0.5 }} />{satellites}
      </Typography>
    </Box>
  );
};

export default DroneStatus;