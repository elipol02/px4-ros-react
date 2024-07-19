import React, { useState } from 'react';
import PresetTopicSubscriber from './PresetTopicSubscriber';
import { Box } from '@mui/material';

const PresetGUI= ({ setPresetFields }) => {

    return (
        <Box>
            <PresetTopicSubscriber
                topicName="/mavros/state"
                keysToDisplay={['connected', 'armed', 'mode', 'system_status']}
                newKeyNames={['connected', 'armed', 'mode', 'system_status']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/extended_state"
                keysToDisplay={['landed_state']}
                newKeyNames={['landed_state']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/sys_status"
                keysToDisplay={['battery_remaining']}
                newKeyNames={['battery']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/global_position/raw/satellites"
                keysToDisplay={['data']}
                newKeyNames={['satellites']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/altitude"
                keysToDisplay={['relative']}
                newKeyNames={['altitude']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/global_position/global"
                keysToDisplay={['latitude', 'longitude']}
                newKeyNames={['latitude', 'longitude']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/global_position/compass_hdg"
                keysToDisplay={['data']}
                newKeyNames={['compass hdg']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/statustext/recv"
                keysToDisplay={['severity', 'text']}
                newKeyNames={['severity', 'message']}
                setPresetFields={setPresetFields}
                saveHistory={true}
            />
            <PresetTopicSubscriber
                topicName="/mavros/local_position/velocity_local"
                keysToDisplay={['twist.linear.x', 'twist.linear.y', 'twist.linear.z']}
                newKeyNames={['Xvel', 'Yvel', 'Zvel']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/local_position/pose"
                keysToDisplay={['pose.position.x', 'pose.position.y', 'pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']}
                newKeyNames={['Xpos', 'Ypos', 'quatX', 'quatY', 'quatZ', 'quatW']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/home_position/home"
                keysToDisplay={['geo.latitude', 'geo.longitude', 'geo.altitude', 'position.x', 'position.y']}
                newKeyNames={['homeLat', 'homeLon', 'homeAlt', 'homeX', 'homeY']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
        </Box>
    );
};

export default PresetGUI;