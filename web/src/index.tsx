import './Styles/style.css'
import '@mantine/core/styles.css';

import * as React from 'react';
import * as ReactDOM from "react-dom/client";
import {MantineProvider} from '@mantine/core';

import ErrorBoundary from "./Components/ErrorBoundary.tsx";
import ApplicationRoutes from "./ApplicationRoutes.tsx";
import {PersistingPreferencesProvider} from "./Context/PreferencesContext.tsx";


ReactDOM
    .createRoot(document.getElementById('root')!)
    .render(
        <React.StrictMode>
            <ErrorBoundary>
                <PersistingPreferencesProvider>
                    <MantineProvider>
                        <ApplicationRoutes/>
                    </MantineProvider>
                </PersistingPreferencesProvider>
            </ErrorBoundary>
        </React.StrictMode>
    );
