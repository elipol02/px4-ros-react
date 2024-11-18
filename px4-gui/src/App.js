import React, { useState, useCallback } from 'react';
import { AppBar, Toolbar, Typography, Button, Box, Grid, Tooltip, Modal, useMediaQuery } from '@mui/material';
import { createTheme, ThemeProvider } from '@mui/material/styles';
import { SettingsProvider } from './components/utils/SettingsContext';
import LocationOnIcon from '@mui/icons-material/LocationOn';
import BuildCircleIcon from '@mui/icons-material/BuildCircle';
import AnnouncementIcon from '@mui/icons-material/Announcement';
import SettingsIcon from '@mui/icons-material/Settings';
import ROSConnection from './components/utils/ROSConnection';
import DropdownTopicSubscriber from './components/subscribers/DropdownTopicSubscriber';
import UnfilteredPresetTopicSubscriber  from './components/subscribers/UnfilteredPresetTopicSubscriber ';
import AggregatedTopicSubscriber from './components/subscribers/AggregatedTopicSubscriber';
import ArmDisarmPublisher from './components/publishers/ArmDisarmPublisher';
import ModePublisher from './components/publishers/ModePublisher';
import PointPublisher from './components/publishers/PointPublisher';
import StartMissionPublisher from './components/publishers/StartMissionPublisher';
import DroneMessages from'./components/interface/DroneMessages';
import MapComponent from'./components/interface/MapComponent';
import Waypoints from'./components/interface/Waypoints';
import DroneState from './components/interface/DroneState';
import DistanceFromHome from './components/interface/DistanceFromHome';
import DroneSpeed from './components/interface/DroneSpeed';
import DroneAttitude from './components/interface/DroneAttitude';
import Settings from './components/interface/Settings';
import Dropdown from './components/interface/Dropdown';
import TopicDisplay from './components/interface/TopicDisplay';
import VideoPlayer from './components/interface/VideoPlayer';

function DroneControlApp() {
  const [showWaypointEditor, setShowWaypointEditor] = useState(false);
  const [showDebugArea, setShowDebugArea] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [selectedTopics, setSelectedTopics] = useState([]);
  const [messages, setMessages] = useState({});
  const [listeners, setListeners] = useState({});
  const [presetFields, setPresetFields] = useState({});
  const [points, setPoints] = useState([]);
  const [editPoints, setEditPoints] = useState([]);
  const [pointsMessage, setPointsMessage] = useState([]);
  const [defaultElevation, setDefaultElevation] = useState(10);
  const [lastMessageSeverity, setLastMessageSeverity] = useState(null);
  const [lastMessage, setLastMessage] = useState(null);
  const [showMessages, setShowMessages] = useState(false);

  const toggleWaypointEditor = () => setShowWaypointEditor(!showWaypointEditor);
  const toggleDebugArea = () => setShowDebugArea(!showDebugArea);
  const toggleSettings = () => setShowSettings(!showSettings);
  const handleShowMessages = () => setShowMessages(!showMessages);

  const getLastItem = (key) => {
    if (presetFields[key] && Array.isArray(presetFields[key])) {
      const array = presetFields[key];
      return array[array.length - 1];
    }
    return null;
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

  const altitude = getLastItem('altitude');

  const theme = createTheme({
    palette: {
      primary: {
        main: '#32006e',
      },
      secondary: {
        main: '#ffffff',
      },
      tertiary: {
        main: '#e8e3d3',
      },
      dark: {
        main: '#000000',
      }
    },
  });

  const isSmallScreen = useMediaQuery(theme.breakpoints.down('sm'));
  const isMediumScreen = useMediaQuery(theme.breakpoints.between('sm', 'md'));
  const isLargeScreen = useMediaQuery(theme.breakpoints.up('md'));

  return (
    <ThemeProvider theme={theme}>
        <SettingsProvider>
            <ROSConnection>
                <AggregatedTopicSubscriber
                    setPresetFields={setPresetFields}
                />
                <UnfilteredPresetTopicSubscriber topicName={'/setpoints'} setField={setPointsMessage}/>
                <PointPublisher editPoints={editPoints}/>
                <Box sx={{ height: '100vh', width: '100vw', overflow: 'hidden', position: 'relative' }}>
                    <AppBar position="static" sx={{ backgroundColor: 'white', color: 'black', height: '64px' }}>
                        <Toolbar>
                            <Typography variant={isSmallScreen ? "p" : "h6"} sx={{ marginRight: isSmallScreen ? 1 : 2 }}>
                                MAVROS-JS
                            </Typography>
                            <Button 
                                variant="outline" 
                                onClick={handleShowMessages} 
                                sx={{ 
                                marginRight: 0.25, 
                                width: '36px', 
                                minWidth: 'auto',
                                flexGrow: isSmallScreen ? 1 : 0
                                }}
                            >
                                <AnnouncementIcon/>
                            </Button>
                            {!isSmallScreen &&
                                <>
                                    <Typography sx={{ flexGrow: 1, display: 'flex', alignItems: 'center' }}>
                                        {lastMessage && 
                                            <>{lastMessageSeverity}:{lastMessage}</>
                                        }
                                    </Typography>
                                    <DroneState
                                        system_status={getLastItem('system_status')}
                                        landed_state={getLastItem('landed_state')}
                                        battery={getLastItem('battery')}
                                        satellites={getLastItem('satellites')}
                                    />
                                </>
                            }
                            <ArmDisarmPublisher 
                                armed={getLastItem('armed')}
                                isSmallScreen={isSmallScreen}
                            />
                            <ModePublisher
                                initialMode={getLastItem('mode')}
                                isSmallScreen={isSmallScreen}
                            />
                            <StartMissionPublisher
                                isSmallScreen={isSmallScreen}
                            />
                        </Toolbar>
                    </AppBar>

                    <Box sx={{ display: 'flex', flexDirection: 'column', height: 'calc(100% - 64px)', width: '100%' }}>
                        <Box sx={{ flexGrow: 1, backgroundColor: 'black', position: 'relative'}}>
                            <MapComponent
                                latitude={getLastItem('latitude')}
                                longitude={getLastItem('longitude')}
                                angle={getLastItem('compass_hdg')}
                                points={points}
                                setEditPoints={setEditPoints}
                                defaultElevation={defaultElevation}
                            />
                            <Box
                                sx={{
                                position: 'absolute',
                                top: (isSmallScreen && showMessages) ? 148 : 16,
                                right: 16,
                                backgroundColor: 'rgba(255, 255, 255, 0.8)',
                                padding: 1,
                                borderRadius: 1,
                                width: 175
                                }}
                            >   
                                {isSmallScreen &&
                                    <DroneState
                                        system_status={getLastItem('system_status')}
                                        landed_state={getLastItem('landed_state')}
                                        battery={getLastItem('battery')}
                                        satellites={getLastItem('satellites')}
                                    />
                                }
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
                                <Typography>
                                    Altitude: {altitude !== null && <>{altitude.toFixed(3)}</>} m
                                </Typography>
                            </Box>

                            <Box
                                sx={{
                                position: 'absolute',
                                bottom: 16,
                                left: 16,
                                }}
                            >
                                <VideoPlayer />
                            </Box>
                        </Box>
                        <Box
                                sx={{
                                position: 'absolute',
                                top: isSmallScreen ? 80 : 64,
                                left: !isSmallScreen && 196,
                                right: isSmallScreen && 16,
                                borderRadius: 1,
                                }}
                            >
                            <DroneMessages 
                                severities={presetFields['severity']} 
                                messages={presetFields['message']}
                                seconds={presetFields['seconds']}
                                nanoseconds={presetFields['nanoseconds']}
                                setLastMessage={setLastMessage}
                                setLastMessageSeverity={setLastMessageSeverity}
                                showMessages={showMessages}
                                isSmallScreen={isSmallScreen}
                            />
                        </Box>

                        <Box sx={{ display: 'flex', width: '100%' }}>
                            <Waypoints
                                setPoints={setPoints}
                                points={points}
                                setEditPoints={setEditPoints}
                                pointsMessage={pointsMessage}
                                defaultElevation={defaultElevation}
                                setDefaultElevation={setDefaultElevation}
                                showWaypointEditor={showWaypointEditor}
                            />
                        </Box>
                        <Box sx={{ display: 'flex', width: '100%' }}>
                        {showDebugArea && (
                            <Box sx={{ 
                                flexShrink: 0, 
                                width: '100%', 
                                borderTop: 1, 
                                borderColor: 'divider', 
                                backgroundColor: 'white', 
                                maxHeight: '50vh',
                                overflow: 'auto',
                                }}
                            >
                                <Grid container>
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
                        )}
                        </Box>
                    </Box>

                    <Box sx={{ position: 'absolute', top: 80, left: 16, display: 'flex', flexDirection: 'column', gap: 1.5 }}>
                        <Tooltip title="Waypoint Editor" placement="right">
                            <Button color="tertiary" size="small" variant="contained" onClick={toggleWaypointEditor}>
                                <LocationOnIcon fontSize="small"/>
                            </Button>
                        </Tooltip>
                        <Tooltip title="View ROS topics" placement="right">
                            <Button color="tertiary" size="small" variant="contained" onClick={toggleDebugArea}>
                                <BuildCircleIcon fontSize="small"/>
                            </Button>
                        </Tooltip>
                        <Tooltip title="Settings" placement='right'>
                            <Button color="tertiary" size="small" variant="contained" onClick={toggleSettings}>
                                <SettingsIcon fontSize="small"/>
                            </Button>
                        </Tooltip>
                    </Box>
                    <Modal
                        open={showSettings}
                        onClose={toggleSettings}
                        aria-labelledby="modal-modal-title"
                        aria-describedby="modal-modal-description"
                    >
                        <Settings/>
                    </Modal>
                </Box>
            </ROSConnection>
        </SettingsProvider>
    </ThemeProvider>
  );
}

export default DroneControlApp;
