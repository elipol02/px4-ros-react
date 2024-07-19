// components/PresetTopicSubscriber.js
import { useEffect, useRef } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';

const AggregatedTopicSubscriber = ({ setPresetFields }) => {
  const { ros, connected } = useROS();
  const listenerRef = useRef(null);
  const maxHistoryLength = 10;

  const appendToFieldHistory = (prevFields, data) => {
    const updatedFields = { ...prevFields };

    for (const key in data) {
      if (updatedFields[key] && Array.isArray(updatedFields[key])) {
        updatedFields[key] = [...updatedFields[key], data[key]].slice(-maxHistoryLength);
      } else {
        updatedFields[key] = [data[key]];
      }
    }

    return updatedFields;
  };

  useEffect(() => {
    if (!ros || !connected) return;

    const subscribeToTopic = () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }

      const topicName = '/aggregated_data';

      ros.getTopics((result) => {
        const topicIndex = result.topics.indexOf(topicName);
        if (topicIndex !== -1) {
          const messageType = result.types[topicIndex];

          listenerRef.current = new ROSLIB.Topic({
            ros,
            name: topicName,
            messageType: messageType,
            throttle_rate: 100,
          });

          listenerRef.current.subscribe((message) => {
            const data = JSON.parse(message.data);

            setPresetFields((prevFields) => appendToFieldHistory(prevFields, data));
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

export default AggregatedTopicSubscriber;
