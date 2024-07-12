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
                newKeyNames={['battery_remaining (%)']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/global_position/raw/satellites"
                keysToDisplay={['data']}
                newKeyNames={['satellite count']}
                setPresetFields={setPresetFields}
                saveHistory={false}
            />
            <PresetTopicSubscriber
                topicName="/mavros/altitude"
                keysToDisplay={['relative']}
                newKeyNames={['relative_altitude (m)']}
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
        </Box>
    );
};

export default PresetGUI;