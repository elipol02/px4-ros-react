import React, { createContext, useContext, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

// Create a context for ROS
const ROSContext = createContext();

export const ROSProvider = ({ ros, children }) => {
  const topicsRef = useRef({});

  // Function to get or create a ROS topic subscriber
  const getOrCreateTopic = (topicName, messageType, callback) => {
    if (!topicsRef.current[topicName]) {
      const topic = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType,
      });
      topic.subscribe(callback);
      topicsRef.current[topicName] = { topic, callbacks: [callback] };
    } else {
      topicsRef.current[topicName].callbacks.push(callback);
    }
  };

  // Function to unsubscribe a callback from a ROS topic
  const unsubscribeCallback = (topicName, callback) => {
    if (topicsRef.current[topicName]) {
      topicsRef.current[topicName].callbacks = topicsRef.current[topicName].callbacks.filter(
        (cb) => cb !== callback
      );
      if (topicsRef.current[topicName].callbacks.length === 0) {
        topicsRef.current[topicName].topic.unsubscribe(callback);
        delete topicsRef.current[topicName];
      }
    }
  };

  return (
    <ROSContext.Provider value={{ getOrCreateTopic, unsubscribeCallback }}>
      {children}
    </ROSContext.Provider>
  );
};

export const useROS = () => useContext(ROSContext);
