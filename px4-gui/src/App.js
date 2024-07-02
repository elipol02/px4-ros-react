// App.js
import React, { useState, useCallback } from 'react';
import { Container, Box, Grid } from '@mui/material';
import ROSConnection from './components/ROSConnection';
import Topic from './components/Topic';
import Dropdown from './components/Dropdown';
import TopicDisplay from './components/TopicDisplay';
import PresetGUI from './components/PresetGUI';
import PresetTopicDisplay from './components/PresetTopicDisplay'
import DroneMap from'./components/DroneMap'
import LocationPublisher from './components/LocationPublisher';
import Renderer from './components/Renderer';

const App = () => {
  const [selectedTopics, setSelectedTopics] = useState([]);
  const [messages, setMessages] = useState({});
  const [listeners, setListeners] = useState({});
  const [presetFields, setPresetFields] = useState({});
  const [clickedCoords, setClickedCoords] = useState('');
  const [clickedLocations, setClickedLocations] = useState([]);

  const setField = (fields) => {
      setPresetFields((prevFields) => ({
        ...prevFields,
        ...fields,
      }));
    };

  const handleAddTopic = (topic) => {
    if (!selectedTopics.includes(topic)) {
      setSelectedTopics([...selectedTopics, topic]);
      setMessages((prevMessages) => ({ ...prevMessages, [topic]: [] }));
    }
  };

  const handleRemoveTopic = (topic) => {
    setSelectedTopics(selectedTopics.filter((t) => t !== topic));
    setMessages((prevMessages) => {
      const newMessages = { ...prevMessages };
      delete newMessages[topic];
      return newMessages;
    });

    if (listeners[topic]) {
      listeners[topic].unsubscribe();
      setListeners((prevListeners) => {
        const newListeners = { ...prevListeners };
        delete newListeners[topic];
        return newListeners;
      });
    }
  };

  const handleNewMessage = useCallback((topic, message) => {
    setMessages((prevMessages) => ({
      ...prevMessages,
      [topic]: [message],
    }));
  }, []);

  const handleSubscriptionReady = useCallback((topic, listener) => {
    setListeners((prevListeners) => ({
      ...prevListeners,
      [topic]: listener,
    }));
  }, []);

  return (
    <ROSConnection>
      <Container>
        <PresetGUI
            setField={setField}
        />
        <Grid container item>
          <PresetTopicDisplay
            topicName="/mavros/state"
            fields={presetFields}
          />
          <LocationPublisher
            clickedCoords={clickedCoords}
          /> 
        </Grid>
        <DroneMap
          latitude={presetFields['latitude']}
          longitude={presetFields['longitude']}
          angle={presetFields['compass']}
          altitude={presetFields['local_altitude']}
          setClickedCoords={setClickedCoords}
          clickedLocations={clickedLocations}
          setClickedLocations={setClickedLocations}

        />
        <Box sx={{ my: 4 }}>
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <Dropdown addTopic={handleAddTopic} />
            </Grid>
            {selectedTopics.map((topic) => (
              <React.Fragment key={topic}>
                <Topic
                  topic={topic}
                  onNewMessage={handleNewMessage}
                  onSubscriptionReady={(listener) => handleSubscriptionReady(topic, listener)}
                />
                <Grid item>
                  <TopicDisplay topic={topic} messages={messages[topic]} onRemove={handleRemoveTopic} />
                </Grid>
              </React.Fragment>
            ))}
          </Grid>
        </Box>
      </Container>
    </ROSConnection>
  );
};

export default App;