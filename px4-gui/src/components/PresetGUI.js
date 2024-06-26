import React, { useState } from 'react';
import PresetTopicGetter from './PresetTopicGetter';
import PresetTopicDisplay from './PresetTopicDisplay';
import { Box } from '@mui/material';

const PresetGUI= ({ setField }) => {

    return (
        <Box>
            <PresetTopicGetter
                topicName="/mavros/state"
                keysToDisplay={['connected', 'armed', 'mode', 'system_status']}
                newKeyNames={['connected', 'armed', 'mode', 'system_status']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/extended_state"
                keysToDisplay={['landed_state']}
                newKeyNames={['landed_state']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/sys_status"
                keysToDisplay={['battery_remaining']}
                newKeyNames={['battery_remaining']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/global_position/raw/satellites"
                keysToDisplay={['data']}
                newKeyNames={['satellites']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/altitude"
                keysToDisplay={['local']}
                newKeyNames={['local_altitude']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/global_position/global"
                keysToDisplay={['latitude', 'longitude']}
                newKeyNames={['latitude', 'longitude']}
                setField={setField}
            />
            <PresetTopicGetter
                topicName="/mavros/global_position/compass_hdg"
                keysToDisplay={['data']}
                newKeyNames={['compass']}
                setField={setField}
            />
        </Box>
    );
};

export default PresetGUI;