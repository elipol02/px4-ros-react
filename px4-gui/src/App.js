import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import { ROSProvider } from './components/ROSContext';
import { Container, Typography, Grid } from '@mui/material';
import TopicGetter from './components/TopicGetter';
import TopicDisplay from './components/TopicDisplay';
import TopicDropdown from './components/TopicDropdown';
import TopicViewer from './components/TopicViewer';
import DroneMap from './components/DroneMap';

const App = () => {
  // State to store the ROS connection instance
  const [ros, setRos] = useState(null);
  // State to store the list of selected topics
  const [selectedTopics, setSelectedTopics] = useState([]);
  // State to store the types of topics
  const [topicTypes, setTopicTypes] = useState({});

  // Initialize ROS connection and fetch topics and their types
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosInstance.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    rosInstance.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    setRos(rosInstance);

    // Fetch list of available topics
    const topicsClient = new ROSLIB.Service({
      ros: rosInstance,
      name: '/rosapi/topics',
      serviceType: 'rosapi/Topics'
    });

    const request = new ROSLIB.ServiceRequest();

    topicsClient.callService(request, (result) => {
        result.topics.forEach((topic) => {
          const typesClient = new ROSLIB.Service({
            ros: rosInstance,
            name: '/rosapi/topic_type',
            serviceType: 'rosapi/TopicType'
          });

          const typeRequest = new ROSLIB.ServiceRequest({ topic });
          typesClient.callService(typeRequest, (typeResult) => {
            setTopicTypes((prevTypes) => ({
              ...prevTypes,
              [topic]: typeResult.type
            }));
          });
        });
      });

  }, []);

  const [fields, setFields] = useState({});
  const [latitude, setLatitude] = useState(null);
  const [longitude, setLongitude] = useState(null);
  const [angle, setAngle] = useState(90);

  const setField = (topicName, keyValuePairs) => {
    setFields((prevFields) => ({
      ...prevFields,
      [topicName]: {
        ...prevFields[topicName],
        ...keyValuePairs,
      },
    }));
  };

  // Handle selecting a topic from the dropdown
  const handleSelectTopic = (topic) => {
    if (!selectedTopics.includes(topic)) {
      setSelectedTopics([...selectedTopics, topic]);
    }
  };

  // Handle removing a selected topic
  const handleRemoveTopic = (topic) => {
    setSelectedTopics(selectedTopics.filter(t => t !== topic));
  };

  useEffect(() => {
    if (fields['/mavros/global_position/global']) {
      setLatitude(fields['/mavros/global_position/global'].latitude || 0);
      setLongitude(fields['/mavros/global_position/global'].longitude || 0);
    }
    if (fields['/mavros/global_position/compass_hdg']) {
      setAngle(fields['/mavros/global_position/compass_hdg'].data || 0);
    }
  }, [fields]);

  return (
    <Container>
      <Typography variant="h4" component="h1" gutterBottom>
        PX4 Drone Control
      </Typography>
      {ros && (
        <ROSProvider ros={ros}>
          <div>
            {/* Custom Topic Viewer for a preset topic */}
            <TopicGetter
              ros={ros}
              topicName="/mavros/state"
              keysToDisplay={['connected', 'armed', 'mode', 'system_status']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/extended_state"
              keysToDisplay={['landed_state']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/sys_status"
              keysToDisplay={['battery_remaining']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/global_position/raw/satellites"
              keysToDisplay={['data']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/altitude"
              keysToDisplay={['local']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/global_position/global"
              keysToDisplay={['latitude', 'longitude']}
              setField={setField}
            />
            <TopicGetter
              ros={ros}
              topicName="/mavros/global_position/compass_hdg"
              keysToDisplay={['data']}
              setField={setField}
            />
            <TopicDisplay fields={fields} />
          </div>
        </ROSProvider>
      )}
      <DroneMap
        latitude={latitude}
        longitude={longitude}
        angle={angle}
      />
      {/* Dropdown to select and add topics */}
      {ros && (
        <TopicDropdown ros={ros} onSelectTopic={handleSelectTopic} topicTypes={topicTypes} />
      )}
      {/* Display a TopicViewer for each selected topic */}
      <Grid container spacing={2}>
        {ros &&
          selectedTopics.map((topic) => (
            <Grid item xs={12} key={topic}>
              <TopicViewer
                ros={ros}
                topicName={topic}
                messageType={topicTypes[topic]}
                onRemove={() => handleRemoveTopic(topic)}
              />
            </Grid>
          ))}
      </Grid>
    </Container>
  );
};

export default App;
