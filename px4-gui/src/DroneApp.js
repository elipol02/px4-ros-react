// App.js
import React, { useState, useCallback } from 'react';
import { Container, Box, Grid, Card } from '@mui/material';
import ROSConnection from './components/ROSConnection';
import DropdownTopicSubscriber from './components/subscribers/DropdownTopicSubscriber';
import Dropdown from './components/interface/Dropdown';
import TopicDisplay from './components/interface/TopicDisplay';
import PresetSubscribers from './components/testing / not used/PresetSubscribers';
import PresetTopicDisplay from './components/interface/PresetTopicDisplay';
import ArmDisarmPublisher from './components/publishers/ArmDisarmPublisher';
import ModePublisher from './components/publishers/ModePublisher';
import PointPublisher from './components/publishers/PointPublisher';
import StartMissionPublisher from './components/publishers/StartMissionPublisher';
import DroneMessages from'./components/interface/DroneMessages';
import UnfilteredPresetTopicSubscriber  from './components/subscribers/UnfilteredPresetTopicSubscriber ';
import MapComponent from'./components/interface/MapComponent';
import Waypoints from'./components/interface/Waypoints';
import Px4Params from './components/interface/Params';
import DroneState from './components/interface/DroneState';
import DistanceFromHome from './components/interface/DistanceFromHome';
import DroneSpeed from './components/interface/DroneSpeed';
import DroneAttitude from './components/interface/DroneAttitude';

const App = () => {
  const [selectedTopics, setSelectedTopics] = useState([]);
  const [messages, setMessages] = useState({});
  const [listeners, setListeners] = useState({});
  const [presetFields, setPresetFields] = useState({});
  const [points, setPoints] = useState([]);
  const [editPoints, setEditPoints] = useState([]);
  const [pointsMessage, setPointsMessage] = useState([]);
  const [defaultElevation, setDefaultElevation] = useState(10);

  const getLastItem = (key) => {
    if (presetFields[key] && Array.isArray(presetFields[key])) {
      const array = presetFields[key];
      return array[array.length - 1];
    }
    return null; // Return null if the key does not exist or is not an array
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
    setMessages((prevMessages) => {
      const newMessages = [...(prevMessages[topic] || []), message];
      return {
        ...prevMessages,
        [topic]: newMessages.slice(-10),
      };
    });
  }, []);

  const handleSubscriptionReady = useCallback((topic, listener) => {
    setListeners((prevListeners) => ({
      ...prevListeners,
      [topic]: listener,
    }));
  }, []);

  return (
    <ROSConnection>
      <PresetSubscribers
            setPresetFields={setPresetFields}
        />
      <UnfilteredPresetTopicSubscriber topicName={'/setpoints'} setField={setPointsMessage}/>
      <PointPublisher editPoints={editPoints}/>
      <Container>
        <Grid container item>
          <PresetTopicDisplay
            fields={presetFields}
            getLastItem={getLastItem}
          />
          <Box width={200} sx={{ p: 2, my: 2, display: 'flex', flexDirection: 'column', gap: 2 }}>
            <Card style={{ overflow: 'auto', height: '100%', width: '100%' }}>
              <ArmDisarmPublisher 
                armed={getLastItem('armed')}
              />
              <ModePublisher
                initialMode={getLastItem('mode')}
              />
              <StartMissionPublisher/>
              <DroneState
                system_status={getLastItem('system_status')}
                landed_state={getLastItem('landed_state')}
              />
              <DistanceFromHome
                homeX={getLastItem('homeX')}
                homeY={getLastItem('homeY')}
                Xpos={getLastItem('Xpos')}
                Ypos={getLastItem('Ypos')}
              />
              <DroneSpeed
                x={getLastItem('Xvel')}
                y={getLastItem('Yvel')}
                z={getLastItem('Zvel')}
              />
              <DroneAttitude
                quatX={getLastItem('quatX')}
                quatY={getLastItem('quatY')}
                quatZ={getLastItem('quatZ')}
                quatW={getLastItem('quatW')}
              />
            </Card>
          </Box>
          <DroneMessages severities={presetFields['severity']} messages={presetFields['message']}/>
        </Grid>
        <Waypoints
          setPoints={setPoints}
          points={points}
          setEditPoints={setEditPoints}
          pointsMessage={pointsMessage}
          defaultElevation={defaultElevation}
          setDefaultElevation={setDefaultElevation}
        />
        <MapComponent
          latitude={getLastItem('latitude')}
          longitude={getLastItem('longitude')}
          angle={getLastItem('compass hdg')}
          points={points}
          setEditPoints={setEditPoints}
          defaultElevation={defaultElevation}
        />
        <Box sx={{ my: 4 }}>
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <Dropdown addTopic={handleAddTopic} />
            </Grid>
            {selectedTopics.map((topic) => (
              <React.Fragment key={topic}>
                <DropdownTopicSubscriber
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