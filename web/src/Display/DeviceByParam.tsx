import {Link, useParams} from "react-router-dom";
import * as React from 'react';

import {usePersistingPreferences} from "../Context/PreferencesContext.tsx";
import Device from "./Presence/Device.tsx";
import {useRoot} from '../Components/RootHook.tsx'

export default () => {
    const params = useParams();
    const {preferences} = usePersistingPreferences();
    const {root} = useRoot();
    const hosts = preferences.hosts
        .filter(({name}) => params.deviceName === name)
        .map(({name, hostIp}) => <>
                <Link to={`${root}/`}>Home</Link>
                <Device displayMode={'large'} deviceName={name} hostIp={hostIp}/>
            </>
        )

    if (hosts.length === 0) {
        return <h1>Error, unknown hosts "{params.deviceName}" in preferences</h1>
    }

    return hosts[0];
}
