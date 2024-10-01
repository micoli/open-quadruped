import React from 'react';
import {useState} from 'react';
import {Autocomplete, Button, TextInput} from "@mantine/core";
import {
    IconLayoutGridAdd,
    IconTrash,
    IconDeviceFloppy,
} from '@tabler/icons-react';
import * as allIcons from "tabler-icons-react";

import {PERSISTING_PREFERENCES_ACTIONS, usePersistingPreferences} from "../Context/PreferencesContext.tsx";

const DynamicIcon = ({iconName}: { iconName: string }) => {
    // @ts-ignore
    const IconToBeUsed = allIcons[iconName] ?? allIcons['Unlink'];
    return <IconToBeUsed/>
}

const allIconNames = Object.keys(allIcons);

const Settings = () => {
    const {preferences, dispatchPersistingPreferences} = usePersistingPreferences();
    const [hosts, setHosts] = useState(preferences.hosts)

    return <>
        <h1>Settings</h1>
        <Button onClick={
            () => {
                hosts.push({hostIp: '', name: '', icon: 'Unlink'})
                setHosts([...hosts])
            }
        }><IconLayoutGridAdd/></Button>
        <table>
            <thead>
            <tr>
                <td>
                    Host
                </td>
                <td>
                    Name
                </td>
                <td>
                    Icon
                </td>
                <td>
                    &nbsp;
                </td>
                <td>
                    &nbsp;
                </td>
            </tr>
            </thead>
            <tbody>
            {hosts.map((host, index) =>
                <tr key={index}>
                    <td>
                        <TextInput
                            value={host.hostIp}
                            placeholder="192.168.X.X, 10.X.X.X"
                            onChange={(event) => {
                                hosts[index].hostIp = event.currentTarget.value;
                                setHosts([...hosts])
                            }}
                        />
                    </td>
                    <td>
                        <TextInput
                            value={host.name}
                            placeholder="bedroom1, kitchen"
                            onChange={(event) => {
                                hosts[index].name = event.currentTarget.value;
                                setHosts([...hosts])
                            }}
                        />
                    </td>
                    <td>
                        <Autocomplete
                            placeholder="Unlink"
                            limit={10}
                            value={host.icon}
                            data={allIconNames}
                            onChange={(iconName) => {
                                hosts[index].icon = iconName;
                                setHosts([...hosts])
                            }}
                        />
                    </td>
                    <td>
                        <DynamicIcon iconName={host.icon}/>
                        {/*
                            <TextInput
                            value={host.icon}
                            placeholder="Unlink"
                            onChange={(event) => {
                                hosts[index].icon = event.currentTarget.value;
                                setHosts([...hosts])
                            }}
                        />
                        */}
                    </td>
                    <td><Button onClick={
                        () => {
                            hosts.splice(index, 1)
                            setHosts([...hosts])
                        }
                    }><IconTrash/></Button></td>
                </tr>
            )}
            </tbody>
        </table>
        <Button onClick={
            () => {
                console.log('save', hosts)
                dispatchPersistingPreferences({type: PERSISTING_PREFERENCES_ACTIONS.SAVE_HOSTS, hosts})
            }
        }><IconDeviceFloppy/></Button>

    </>
}
export default Settings;
