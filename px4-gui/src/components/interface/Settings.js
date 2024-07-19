import React, { useState, useContext } from 'react';
import { Box, Typography, Tabs, Tab } from '@mui/material';
import { SettingsContext } from '../utils/SettingsContext';

const TabPanel = (props) => {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`simple-tabpanel-${index}`}
      aria-labelledby={`simple-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ p: 3 }}>
          <Typography>{children}</Typography>
        </Box>
      )}
    </div>
  );
};

const Settings = () => {
  const { settings, setSettings } = useContext(SettingsContext);
  const [value, setValue] = useState(0);

  const handleChange = (event, newValue) => {
    setValue(newValue);
  };

  return (
    <Box
      sx={{
        position: 'absolute',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
        width: '75%',
        height: '60%',
        bgcolor: 'background.paper',
        boxShadow: 24,
        p: 4,
      }}
    >
      <Tabs value={value} onChange={handleChange} aria-label="settings tabs">
        <Tab label="App" />
        <Tab label="Mavros" />
        <Tab label="Drone" />
      </Tabs>
      <TabPanel value={value} index={0}>
        <Typography sx={{ mt: 1 }}>ROS IP:</Typography>
        <Typography sx={{ mt: 1 }}>Data rate:</Typography>
        <Typography sx={{ mt: 1 }}>FC Type:</Typography>
        <Typography sx={{ mt: 1 }}>Frame Type:</Typography>
      </TabPanel>
      <TabPanel value={value} index={1}>
        <Typography sx={{ mt: 1 }}>FC IP:</Typography>
      </TabPanel>
      <TabPanel value={value} index={2}>
        <Typography sx={{ mt: 1 }}>Drone Setting 1:</Typography>
        <Typography sx={{ mt: 1 }}>Drone Setting 2:</Typography>
        <Typography sx={{ mt: 1 }}>Drone Setting 3:</Typography>
      </TabPanel>
    </Box>
  );
};

export default Settings;
