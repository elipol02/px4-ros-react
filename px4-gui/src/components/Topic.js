import { useEffect, useRef } from 'react';
import { useROS } from './ROSConnection';
import ROSLIB from 'roslib';

const Topic = ({ topic, onNewMessage, onSubscriptionReady }) => {
  const { ros, connected } = useROS();
  const listenerRef = useRef(null);

  useEffect(() => {
    if (!ros || !connected) return;

    const getMessageType = async () => {
      ros.getTopics((result) => {
        const topicIndex = result.topics.indexOf(topic);
        if (topicIndex !== -1) {
          const messageType = result.types[topicIndex];

          listenerRef.current = new ROSLIB.Topic({
            ros,
            name: topic,
            messageType: messageType,
          });

          listenerRef.current.subscribe((message) => {
            onNewMessage(topic, message);
          });

          onSubscriptionReady(listenerRef.current);
        } else {
          console.error(`Topic ${topic} not found`);
        }
      });
    };

    getMessageType();

    return () => {
      if (listenerRef.current) {
        listenerRef.current.unsubscribe();
      }
    };
  }, []); // Include all dependencies

  return null;
};

export default Topic;
