import React, { useState, useEffect, useCallback, useMemo } from 'react';
import PropTypes from 'prop-types';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';
import { TextField, Button, CircularProgress, Typography, Box } from '@mui/material';
import { FixedSizeList as List } from 'react-window';

const ParamRow = React.memo(({ param, paramValue, newValue, handleValueChange, handleSubmit, style, editingParam, setEditingParam }) => {
  const onFocus = () => setEditingParam(param);
  const onBlur = () => setEditingParam(null);

  return (
    <Box key={param} sx={{ display: 'flex', alignItems: 'center', marginBottom: 1 }} style={style}>
      <Typography sx={{ flex: 1, fontSize: '0.875rem' }}>
        {param}
      </Typography>
      <Typography sx={{ flex: 1, fontSize: '0.875rem' }}>
        Current Value: {paramValue?.value}
      </Typography>
      <TextField
        label="New Value"
        value={newValue}
        onChange={(e) => handleValueChange(param, e.target.value)}
        sx={{ flex: 1, marginRight: 1 }}
        size="small"
        onFocus={onFocus}
        onBlur={onBlur}
      />
      <Button
        variant="contained"
        color="tertiary"
        size="small"
        onClick={() => handleSubmit(param)}
      >
        Set
      </Button>
    </Box>
  );
});

const Params = ({ serviceName }) => {
  const { ros, connected } = useROS();
  const [params, setParams] = useState([]);
  const [loading, setLoading] = useState(true);
  const [paramValues, setParamValues] = useState({});
  const [newValues, setNewValues] = useState({});
  const [searchQuery, setSearchQuery] = useState('');
  const [editingParam, setEditingParam] = useState(null);

  useEffect(() => {
    if (connected && ros) {
      const listService = new ROSLIB.Service({
        ros: ros,
        name: `/${serviceName}/list_parameters`,
        serviceType: 'rcl_interfaces/srv/ListParameters',
      });

      const request = new ROSLIB.ServiceRequest({});

      listService.callService(request, (result) => {
        if (result && result.result && result.result.names) {
          setParams(result.result.names);
          result.result.names.forEach((param) => {
            getParamValue(param);
          });
        } else {
          console.error('No parameters found');
        }
        setLoading(false);
      }, (error) => {
        console.error('Error listing parameters: ', error);
        setLoading(false);
      });
    }
  }, [connected, ros, serviceName]);

  const getParamValue = useCallback((paramName) => {
    const getService = new ROSLIB.Service({
      ros: ros,
      name: `/${serviceName}/get_parameters`,
      serviceType: 'rcl_interfaces/srv/GetParameters',
    });

    const request = new ROSLIB.ServiceRequest({
      names: [paramName],
    });

    getService.callService(request, (result) => {
      if (result && result.values && result.values.length > 0) {
        const param = result.values[0];
        setParamValues((prevValues) => ({
          ...prevValues,
          [paramName]: {
            value: param.integer_value || param.double_value || param.string_value || '',
            type: param.type,
          },
        }));
      } else {
        console.error('Parameter value not found');
      }
    }, (error) => {
      console.error('Error getting parameter value: ', error);
    });
  }, [ros, serviceName]);

  const handleValueChange = useCallback((paramName, value) => {
    setNewValues((prevValues) => ({
      ...prevValues,
      [paramName]: value,
    }));
  }, []);

  const handleSubmit = useCallback((paramName) => {
    const paramValue = paramValues[paramName];
    const newValue = newValues[paramName];

    if (newValue !== undefined && paramValue && ros) {
      const setService = new ROSLIB.Service({
        ros: ros,
        name: `/${serviceName}/set_parameters`,
        serviceType: 'rcl_interfaces/srv/SetParameters',
      });

      let paramRequest = {
        name: paramName,
        value: {
          type: paramValue.type,
        },
      };

      switch (paramValue.type) {
        case 1: // Boolean
          paramRequest.value.bool_value = newValue === 'true';
          break;
        case 2: // Integer
          paramRequest.value.integer_value = parseInt(newValue, 10);
          break;
        case 3: // Double
          paramRequest.value.double_value = parseFloat(newValue);
          break;
        case 4: // String
          paramRequest.value.string_value = newValue;
          break;
        default:
          alert('Unsupported parameter type');
          return;
      }

      const request = new ROSLIB.ServiceRequest({
        parameters: [paramRequest],
      });

      setService.callService(request, (result) => {
        console.log('Set parameter result:', result);
        if (result.results && result.results.length > 0 && result.results[0].successful) {
          alert(`Parameter ${paramName} successfully set to ${newValue}`);
          getParamValue(paramName); // Update the displayed value
        } else {
          alert(`Failed to set parameter ${paramName}`);
        }
      }, (error) => {
        console.error('Error setting parameter: ', error);
        alert(`Error setting parameter ${paramName}`);
      });
    }
  }, [paramValues, newValues, ros, getParamValue, serviceName]);

  const filteredParams = useMemo(() => params.filter(param => param.toLowerCase().includes(searchQuery.toLowerCase())), [params, searchQuery]);

  const renderRow = useCallback(({ index, style }) => {
    const param = filteredParams[index];
    return (
      <ParamRow
        param={param}
        paramValue={paramValues[param]}
        newValue={newValues[param] || ''}
        handleValueChange={handleValueChange}
        handleSubmit={handleSubmit}
        style={style}
        editingParam={editingParam}
        setEditingParam={setEditingParam}
      />
    );
  }, [filteredParams, paramValues, newValues, handleValueChange, handleSubmit, editingParam]);

  return (
    <Box sx={{ maxHeight: '100%', padding: 1 }}>
      <Typography variant="h5" gutterBottom>
        {serviceName}
      </Typography>
      <TextField
        label="Search Parameters"
        value={searchQuery}
        onChange={(e) => setSearchQuery(e.target.value)}
        sx={{ marginBottom: 2 }}
        fullWidth
      />
      {loading ? (
        <CircularProgress />
      ) : (
        <List
          height={600}
          itemCount={filteredParams.length}
          itemSize={50}
          width="100%"
        >
          {renderRow}
        </List>
      )}
    </Box>
  );
};

Params.propTypes = {
  serviceName: PropTypes.string.isRequired,
};

export default Params;
