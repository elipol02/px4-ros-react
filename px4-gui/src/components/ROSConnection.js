// components/ROSConnection.js
import React, { createContext, useContext, useEffect, useState, useCallback } from 'react';
import ROSLIB from 'roslib';

const ROSContext = createContext();

export const useROS = () => useContext(ROSContext);

const ROSConnection = ({ children }) => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  const connectToRos = useCallback(() => {
    const rosConnection = new ROSLIB.Ros({
      url: 'ws://localhost:9090',
    });

    rosConnection.on('connection', () => {
      console.log('Connected to ROS');
      setRos(rosConnection);
      setConnected(true);
    });

    rosConnection.on('error', (error) => {
      console.error('Error connecting to ROS: ', error);
      setConnected(false);
    });

    rosConnection.on('close', () => {
      console.warn('Connection to ROS closed. Attempting to reconnect...');
      setConnected(false);
      // Avoid immediate reconnection to prevent rapid retries
      setTimeout(() => connectToRos(), 5000); // Reconnect after 5 seconds
    });

    return () => {
      if (rosConnection) {
        rosConnection.close();
      }
    };
  }, []);

  useEffect(() => {
    connectToRos();
  }, [connectToRos]);

  return <ROSContext.Provider value={{ ros, connected }}>{children}</ROSContext.Provider>;
};

export default ROSConnection;
