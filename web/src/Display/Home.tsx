import * as React from 'react';

import {usePersistingPreferences} from "../Context/PreferencesContext.tsx";
import Device from "./Presence/Device.tsx";
import {Grid} from "@mantine/core";
import {Link} from "react-router-dom";
import {useRoot} from '../Components/RootHook.tsx'

export default () => {
    const {preferences} = usePersistingPreferences();
    const {root} = useRoot();
    return <Grid>
        {preferences.hosts.map(({hostIp, name}) => <Grid.Col key={hostIp} span={6}>
                <Link to={`${root}/device/${name}`}>Detail</Link>
                <Device displayMode={'compact'} deviceName={name} hostIp={hostIp}/>
            </Grid.Col>
        )}
    </Grid>
}
