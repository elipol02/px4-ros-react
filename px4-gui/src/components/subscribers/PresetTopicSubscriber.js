// components/PresetTopicGetter.js
import { useEffect, useRef } from 'react';
import { useROS } from '../ROSConnection';
import ROSLIB from 'roslib';

const PresetTopicSubscriber = ({ topicName, keysToDisplay, newKeyNames, setPresetFields, saveHistory }) => {
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

            setPresetFields((prevFields) => {
              // Create a new object to store the updated fields
              const updatedFields = { ...prevFields };
          
              // Iterate over the keys in the new fields
              for (const key in filteredMessage) {
                if (saveHistory) {
                  // Check if the key exists in the previous fields and if it is an array
                  if (updatedFields[key] && Array.isArray(updatedFields[key])) {
                    // Append the new value to the existing array
                    updatedFields[key] = [...updatedFields[key], filteredMessage[key]];
                  } else {
                    // If the key does not exist, create a new array with the new value
                    updatedFields[key] = [filteredMessage[key]];
                  }
                } else {
                  // If saveHistory is false, set the key to the new value
                  updatedFields[key] = [filteredMessage[key]];
                }
              }
          
              return updatedFields;
            });
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

export default PresetTopicSubscriber;
