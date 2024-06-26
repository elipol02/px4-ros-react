// components/PresetTopicGetter.js
import { useEffect, useRef } from 'react';
import { useROS } from './ROSConnection';
import ROSLIB from 'roslib';

const PresetTopicGetter = ({ topicName, keysToDisplay, newKeyNames, setField }) => {
  const { ros, connected } = useROS();
  const listenerRef = useRef(null);

  useEffect(() => {
    if (!ros || !connected) return;

    const subscribeToTopic = () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }

      ros.getTopics((result) => {
        const topicIndex = result.topics.indexOf(topicName);
        if (topicIndex !== -1) {
          const messageType = result.types[topicIndex];

          listenerRef.current = new ROSLIB.Topic({
            ros,
            name: topicName,
            messageType: messageType,
          });

          listenerRef.current.subscribe((message) => {
            const filteredMessage = keysToDisplay.reduce((acc, key, index) => {
              const newKeyName = newKeyNames[index] || key;
              acc[newKeyName] = message[key];
              return acc;
            }, {});
            setField(filteredMessage);
          });
        } else {
          console.error(`Topic ${topicName} not found`);
        }
      });
    };

    subscribeToTopic();

    return () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }
    };
  }, [connected]); // Only depend on the `connected` state to avoid infinite loops

  return null;
};

export default PresetTopicGetter;
