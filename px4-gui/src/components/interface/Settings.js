import React, { useState, useContext } from 'react';
import { Box, Typography, Tabs, Tab, Stack } from '@mui/material';
import { SettingsContext } from '../utils/SettingsContext';
import Params from './Params';

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
            <Tab label="Ros-React" />
            <Tab label="Mavros" />
            <Tab label="Drone" />
        </Tabs>
        <Box sx={{ height: '95%' }}>
            <TabPanel value={value} index={0}>
                <Typography sx={{ mt: 1 }}>ROS IP:</Typography>
                <Typography sx={{ mt: 1 }}>FC Type:</Typography>
                <Typography sx={{ mt: 1 }}>Frame Type:</Typography>
            </TabPanel>
            <TabPanel value={value} index={1}>
                <Typography sx={{ mt: 1 }}>Aggregated Data:</Typography>
                <Typography sx={{ mt: 1 }}>Data rate:</Typography>
                <Typography sx={{ mt: 1 }}>Joystick port:</Typography>
            </TabPanel>
            <TabPanel value={value} index={2}>
                <Stack spacing={2} sx={{ mt: 1 }}>
                    <Params serviceName='mavros_node' />
                </Stack>
            </TabPanel>
            <TabPanel value={value} index={3}>
            <Params serviceName='mavros/param' />
            </TabPanel>
        </Box>
    </Box>
  );
};

export default Settings;
