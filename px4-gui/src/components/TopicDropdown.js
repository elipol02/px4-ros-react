import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import { FormControl, InputLabel, Select, MenuItem, Button } from '@mui/material';

const TopicDropdown = ({ ros, onSelectTopic, topicTypes }) => {
  // State to store the list of available topics
  const [topics, setTopics] = useState([]);
  // State to store the currently selected topic from the dropdown
  const [selectedTopic, setSelectedTopic] = useState('');

  // Fetch the list of topics when the component mounts
  useEffect(() => {
    const topicsClient = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/topics',
      serviceType: 'rosapi/Topics'
    });

    const request = new ROSLIB.ServiceRequest();

    topicsClient.callService(request, (result) => {
      setTopics(result.topics);
    });
  }, [ros]);

  // Handler for changing the selected topic in the dropdown
  const handleChange = (event) => {
    setSelectedTopic(event.target.value);
  };

  // Handler for adding the selected topic to the list of topics to view
  const handleAddTopic = () => {
    if (selectedTopic && topicTypes[selectedTopic]) {
      onSelectTopic(selectedTopic);
    }
  };

  return (
    <div>
      <FormControl fullWidth variant="outlined" margin="normal">
        <InputLabel id="topic-select-label">Select Topic</InputLabel>
        <Select
          labelId="topic-select-label"
          id="topic-select"
          value={selectedTopic}
          onChange={handleChange}
          label="Select Topic"
        >
          {topics.map((topic, index) => (
            <MenuItem key={index} value={topic}>
              {topic}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
      <Button variant="contained" color="primary" onClick={handleAddTopic} style={{ marginBottom:"10px" }}>
        Add Topic
      </Button>
    </div>
  );
};

export default TopicDropdown;
