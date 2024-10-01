import * as React from 'react';
import {
    createContext, useContext, useReducer
} from 'react';

const LOCALSTORAGE_KEY = 'PREFERENCES';

export const PERSISTING_PREFERENCES_ACTIONS = {
    TOGGLE_DARK_MODE: 'TOGGLE_DARK_MODE',
    NAVBAR_OPENED: 'NAVBAR_OPENED',
    SAVE_HOSTS: 'SAVE_HOSTS'
};

export interface PersistingPreferencesContextData {
    darkMode: boolean
    openedNavbar: boolean
    hosts: { hostIp: string, name: string, icon: string }[]
}

const saveLocalStoragePreferences = ({
                                         darkMode,
                                         openedNavbar,
                                         hosts
                                     }: PersistingPreferencesContextData) => {
    console.log(LOCALSTORAGE_KEY, JSON.stringify({
        darkMode,
        openedNavbar,
        hosts
    }))
    localStorage.setItem(LOCALSTORAGE_KEY, JSON.stringify({
        darkMode,
        openedNavbar,
        hosts
    }));
};

const getLocalStoragePreferences = (): PersistingPreferencesContextData => {
    const defaultValue = {
        darkMode: window.matchMedia('(prefers-color-scheme: dark)').matches,
        openedNavbar: true,
        hosts: [],
    };
    try {
        return {
            ...defaultValue,
            ...JSON.parse(localStorage.getItem(LOCALSTORAGE_KEY) ?? '{}')
        };
    } catch (error) {
        return defaultValue;
    }
};

const reducer = (previousState: PersistingPreferencesContextData, action: any): PersistingPreferencesContextData => {
    switch (action.type) {
        case PERSISTING_PREFERENCES_ACTIONS.TOGGLE_DARK_MODE:
            saveLocalStoragePreferences({
                darkMode: !previousState.darkMode,
                openedNavbar: previousState.openedNavbar,
                hosts: previousState.hosts
            });
            return {
                ...previousState,
                darkMode: !previousState.darkMode
            };
        case PERSISTING_PREFERENCES_ACTIONS.NAVBAR_OPENED:
            saveLocalStoragePreferences({
                darkMode: previousState.darkMode,
                openedNavbar: action.openedNavbar,
                hosts: previousState.hosts
            });
            return {
                ...previousState,
                openedNavbar: action.openedNavbar
            };
        case PERSISTING_PREFERENCES_ACTIONS.SAVE_HOSTS:
            saveLocalStoragePreferences({
                darkMode: previousState.darkMode,
                openedNavbar: previousState.openedNavbar,
                hosts: action.hosts
            });
            return {
                ...previousState,
                hosts: action.hosts
            };
        default:
            return previousState;
    }
};

const PersistingPreferencesContext = createContext<{
    preferences: PersistingPreferencesContextData
    dispatchPersistingPreferences: React.Dispatch<any>
}>({
    preferences: getLocalStoragePreferences(),
    dispatchPersistingPreferences: (): void => {
    }
});

export const usePersistingPreferences = (): {
    preferences: PersistingPreferencesContextData
    dispatchPersistingPreferences: React.Dispatch<any>
} => useContext(PersistingPreferencesContext);

export const PersistingPreferencesProvider = ({children}: any) => {
    const [state, dispatchPersistingPreferences] = useReducer(reducer, getLocalStoragePreferences());

    return (
        <PersistingPreferencesContext.Provider value={{
            preferences: state,
            dispatchPersistingPreferences
        }}>
            {children}
        </PersistingPreferencesContext.Provider>
    );
};
