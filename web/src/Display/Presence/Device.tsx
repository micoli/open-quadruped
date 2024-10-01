import React from 'react';

import useEspHomePresenceHook from './EspHomePresenceHook.tsx'

type DisplayMode = 'large' | 'compact'

const ValueDisplay = ({displayMode, attribute, value}: {
    displayMode: DisplayMode,
    attribute: String,
    value: String
}) => {
    if (displayMode === 'large') {
        return <>{`${attribute}`.replace('sensor-', '').replace('_', ' ')}: {value}</>
    }
    return <>{`${attribute}`.replace('sensor-', '').replace('_', ' ')}: {value}</>
}

export default ({displayMode, deviceName, hostIp}: {
    displayMode: DisplayMode,
    deviceName: string,
    hostIp: string
}) => {
    const {
        engineeringModeState,
        switchEngineeringMode,
        connectionError,
        connectionState,
        espHomeState
    } = useEspHomePresenceHook(hostIp);

    return <div>
        <ul>
            {Object.keys(espHomeState).sort().map((attribute) => {
                const data = espHomeState[attribute];
                return <li><ValueDisplay displayMode={displayMode} attribute={attribute} value={data}/></li>
            })}
        </ul>
        <div>
            <button onClick={() => {
                switchEngineeringMode(!engineeringModeState)
            }}>
                {displayMode == 'large' && `Switch light Sensor `}{engineeringModeState ? 'OFF' : 'ON'}
            </button>
        </div>
        {connectionError && <h3>{deviceName}: {connectionState}</h3>}
    </div>;
}
