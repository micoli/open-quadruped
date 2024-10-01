import * as React from 'react';
import {BrowserRouter, Route, Routes} from "react-router-dom";

import Settings from "./Display/Settings.tsx";
import Home from "./Display/Home.tsx";
import App from "./App.tsx";
import DeviceByParam from "./Display/DeviceByParam.tsx";
import {useRoot} from './Components/RootHook.tsx'

export default () => {
    const {root} = useRoot();

    return <BrowserRouter>
        <Routes>
            <Route path="/" element={<App/>}>
                <Route index element={<Home/>}/>
                <Route path={`${root}/`} element={<Home/>}/>
                <Route path={`${root}/device/:deviceName`} element={<DeviceByParam/>}/>
                <Route path={`${root}/settings`} element={<Settings/>}/>
            </Route>
        </Routes>
    </BrowserRouter>
};
