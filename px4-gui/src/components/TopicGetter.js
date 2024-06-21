import React, { useEffect } from 'react';
import { useROS } from './ROSContext';

const TopicGetter = ({ ros, topicName, keysToDisplay, setField }) => {
  const { getOrCreateTopic, unsubscribeCallback } = useROS();

  useEffect(() => {
    if (!ros || !topicName || !keysToDisplay.length) return;

    const getMessageType = (callback) => {
      ros.getTopicType(topicName, (type) => {
        callback(type);
      });
    };

    const messageCallback = (msg) => {
      const flatMessage = flattenObject(msg);
      const selectedFields = {};

      keysToDisplay.forEach((key) => {
        if (flatMessage[key] !== undefined) {
          selectedFields[key] = flatMessage[key];
        }
      });

      setField(topicName, selectedFields);
    };

    getMessageType((messageType) => {
      getOrCreateTopic(topicName, messageType, messageCallback);
    });

    return () => {
      unsubscribeCallback(topicName, messageCallback);
    };
  }, [ros, topicName, keysToDisplay, setField, getOrCreateTopic, unsubscribeCallback]);

  const flattenObject = (obj, parent = '', res = {}) => {
    for (let key in obj) {
      const propName = parent ? `${parent}.${key}` : key;
      if (typeof obj[key] === 'object' && obj[key] !== null) {
        flattenObject(obj[key], propName, res);
      } else {
        res[propName] = obj[key];
      }
    }
    return res;
  };

  return null;
};

export default TopicGetter;
