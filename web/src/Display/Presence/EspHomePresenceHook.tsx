import {useState, useEffect} from 'react';

export default (hostIp: string) => {
    const [connectionState, setConnectionState] = useState("unknown");
    const [connectionError, setConnectionError] = useState<string|null>(null);
    const [engineeringModeState, setEngineeringModeState] = useState(false);
    const [espHomeState, setEspHomeState] = useState<any>({});

    useEffect(() => {
        const evtSource = new EventSource(`http://${hostIp}/events`);
        setConnectionState('Opening connection');
        evtSource.addEventListener("state", (message) => {
            const rawData = JSON.parse(message.data);
            if (`${rawData.id}` === 'switch-engineering_mode') {
                setEngineeringModeState(rawData.value)
                return;
            }
            if (!`${rawData.id}`.match(/^sensor-/)) {
                return;
            }
            espHomeState[rawData.id] = rawData.value
            setEspHomeState({...espHomeState})
        });

        evtSource.onopen = () => {
            setConnectionState('Connection opened');
        }

        evtSource.onerror = (e) => {
            setConnectionState('ERROR!' + JSON.stringify(e));
            setConnectionError('ERROR!' + JSON.stringify(e));
        }

        return () => {
            evtSource.close();
        }
    }, []);

    const switchEngineeringMode = (value: boolean) => {
        fetch(`http://${hostIp}/switch/engineering_mode/${value ? 'turn_on' : 'turn_off'}`, {
            method: "POST",
            body: "true"
        });
    }
    return {
        engineeringModeState,
        switchEngineeringMode,
        connectionError,
        connectionState,
        espHomeState
    }
}
