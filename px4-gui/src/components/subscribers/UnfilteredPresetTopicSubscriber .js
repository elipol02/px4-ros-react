import { useEffect, useRef } from 'react';
import { useROS } from '../ROSConnection';
import ROSLIB from 'roslib';

const UnfilteredPresetTopicSubscriber = ({ topicName, setField }) => {
  const { ros, connected } = useROS();
  const listenerRef = useRef(null);

  useEffect(() => {
    if (!ros || !connected) return;

    let intervalId;

    const subscribeToTopic = () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }

      const checkAndSubscribe = () => {
        ros.getTopics((result) => {
          const topicIndex = result.topics.indexOf(topicName);
          if (topicIndex !== -1) {
            clearInterval(intervalId); // Clear the interval if the topic is found

            const messageType = result.types[topicIndex];

            listenerRef.current = new ROSLIB.Topic({
              ros,
              name: topicName,
              messageType: messageType,
            });

            listenerRef.current.subscribe((message) => {
              setField(message);
              console.log(message);
            });
          } else {
            console.error(`Topic ${topicName} not found, retrying...`);
          }
        });
      };

      // Check for the topic every 0.5 seconds
      intervalId = setInterval(checkAndSubscribe, 500);
    };

    subscribeToTopic();

    return () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }
      clearInterval(intervalId); // Clear the interval on cleanup
    };
  }, [connected]); // Only depend on the `connected` state to avoid infinite loops

  return null;
};

export default UnfilteredPresetTopicSubscriber;
