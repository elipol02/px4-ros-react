import React, { useEffect, useState } from 'react';
import { useROS } from '../ROSConnection';
import ROSLIB from 'roslib';
import { Box, Button, Card, Typography, Slider, CardContent } from '@mui/material';

const LocationPublisher = ({ clickedCoords }) => {
  const { ros, connected } = useROS();
  const [altitude, setAltitude] = useState(10);
  const [latitude, setLatitude] = useState('');
  const [longitude, setLongitude] = useState('');
  const [locationPublisher, setLocationPublisher] = useState(null);

  useEffect(() => {
    setLatitude(clickedCoords.lat);
    setLongitude(clickedCoords.lng);
  }, [clickedCoords]);

  useEffect(() => {
    if (connected && ros) {
      const publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/flytopoint',
        messageType: 'geometry_msgs/Point',
      });
      setLocationPublisher(publisher);
      console.log('Topic initialized: /flytopoint');
    }
  }, [connected, ros]);

  const handlePublish = () => {
    if (!connected) {
      console.error('Not connected to ROS');
      return;
    }

    if (!locationPublisher) {
      console.error('Topic locationPublisher not initialized');
      return;
    }

    const locationMessage = new ROSLIB.Message({
      x: parseFloat(latitude),
      y: parseFloat(longitude),
      z: parseFloat(altitude),
    });

    locationPublisher.publish(locationMessage);
    console.log('Published to /flytopoint:', locationMessage);
  };

  const handleAltitudeChange = (event, newValue) => {
    setAltitude(newValue);
  };

  return (
    <Box width={200} sx={{ p: 2, my: 2, display: 'flex', flexDirection: 'column', gap: 2 }}>
      <Card style={{ overflow: 'auto', height: '100%', width: '100%' }}>
        <CardContent style={{ padding: '20px' }}>
          <Typography>Fly to Point</Typography>
          <Typography variant="body2" component="p">
            Latitude: {latitude}
          </Typography>
          <Typography variant="body2" component="p">
            Longitude: {longitude}
          </Typography>
          <Typography variant="body2" component="p">
            Altitude: {altitude}m
          </Typography>
          <Slider
            value={altitude}
            min={10}
            max={400}
            step={1}
            onChange={handleAltitudeChange}
            size="small"
            style={{ width: '150px' }}
          />
          <Button variant="contained" color="primary" onClick={handlePublish}>
            Publish
          </Button>
        </CardContent>
      </Card>
    </Box>
  );
};

export default LocationPublisher;
