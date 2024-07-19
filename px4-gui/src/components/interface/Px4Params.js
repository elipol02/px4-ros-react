import React, { useState, useEffect } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { TextField, Button, CircularProgress, Typography, Box, MenuItem } from '@mui/material';

const Px4Params = () => {
  const { ros, connected } = useROS();
  const [params, setParams] = useState([]);
  const [loading, setLoading] = useState(true);
  const [selectedParam, setSelectedParam] = useState('');
  const [paramValue, setParamValue] = useState('');
  const [newValue, setNewValue] = useState('');

  useEffect(() => {
    if (connected && ros) {
      const listService = new ROSLIB.Service({
        ros: ros,
        name: '/mavros_node/list_parameters',
        serviceType: 'rcl_interfaces/srv/ListParameters',
      });

      const request = new ROSLIB.ServiceRequest({});

      listService.callService(request, (result) => {
        console.log('List parameters result:', result);
        if (result && result.result && result.result.names) {
          setParams(result.result.names);
        } else {
          console.error('No parameters found');
        }
        setLoading(false);
      }, (error) => {
        console.error('Error listing parameters: ', error);
        setLoading(false);
      });
    }
  }, [connected, ros]);

  const handleParamChange = (event) => {
    const paramName = event.target.value;
    setSelectedParam(paramName);

    if (paramName) {
      const getService = new ROSLIB.Service({
        ros: ros,
        name: '/mavros_node/get_parameters',
        serviceType: 'rcl_interfaces/srv/GetParameters',
      });

      const request = new ROSLIB.ServiceRequest({
        names: [paramName],
      });

      getService.callService(request, (result) => {
        console.log('Get parameter result:', result);
        if (result && result.values && result.values.length > 0) {
          setParamValue(result.values[0].integer_value || result.values[0].double_value);
        } else {
          console.error('Parameter value not found');
          setParamValue('');
        }
      }, (error) => {
        console.error('Error getting parameter value: ', error);
        setParamValue('');
      });
    } else {
      setParamValue('');
    }
  };

  const handleValueChange = (event) => {
    setNewValue(event.target.value);
  };

  const handleSubmit = () => {
    if (selectedParam && newValue && ros) {
      const setService = new ROSLIB.Service({
        ros: ros,
        name: '/mavros_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters',
      });

      const request = new ROSLIB.ServiceRequest({
        parameters: [{
          name: selectedParam,
          value: {
            type: 'integer', // or 'double' depending on the parameter type
            integer_value: parseInt(newValue, 10),
          },
        }],
      });

      setService.callService(request, (result) => {
        console.log('Set parameter result:', result);
        if (result.successful) {
          alert(`Parameter ${selectedParam} successfully set to ${newValue}`);
        } else {
          alert(`Failed to set parameter ${selectedParam}`);
        }
      }, (error) => {
        console.error('Error setting parameter: ', error);
        alert(`Error setting parameter ${selectedParam}`);
      });
    }
  };

  return (
    <Box sx={{ padding: 2 }}>
      <Typography variant="h4">PX4 Parameters</Typography>
      {loading ? (
        <CircularProgress />
      ) : (
        <Box>
          <TextField
            select
            label="Parameter"
            value={selectedParam}
            onChange={handleParamChange}
            fullWidth
            margin="normal"
          >
            <MenuItem value="">
              <em>None</em>
            </MenuItem>
            {params.map((param, index) => (
              <MenuItem key={index} value={param}>
                {param}
              </MenuItem>
            ))}
          </TextField>
          {selectedParam && (
            <>
              <Typography variant="h6">Current Value: {paramValue}</Typography>
              <TextField
                label="New Value"
                value={newValue}
                onChange={handleValueChange}
                fullWidth
                margin="normal"
              />
              <Button variant="contained" color="primary" onClick={handleSubmit}>
                Set Parameter
              </Button>
            </>
          )}
        </Box>
      )}
    </Box>
  );
};

export default Px4Params;
