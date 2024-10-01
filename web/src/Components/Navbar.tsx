import React from 'react';
import classes from '../Styles/Navbar.module.css';

import * as allIcons from "tabler-icons-react";
import {Link, useLocation} from "react-router-dom";
import {Tooltip, UnstyledButton, Stack,} from '@mantine/core';

import {useRoot} from './RootHook.tsx'
import {usePersistingPreferences} from "../Context/PreferencesContext.tsx";

interface NavbarLinkProps {
    icon: string;
    path: string;
    label: string;
    active?: boolean;

    onClick?(): void;
}

function NavbarLink(link: NavbarLinkProps) {
    // @ts-ignore
    const IconToBeUsed = allIcons[link.icon] ?? allIcons['Unlink'];

    return (
        <Tooltip label={link.label} position="right" transitionProps={{duration: 0}}>
            <UnstyledButton onClick={link.onClick} className={classes.link} data-active={link.active || undefined}>
                <Link to={link.path}>
                    <IconToBeUsed className={classes.linkIcon}/>
                    {link.label}
                </Link>
            </UnstyledButton>
        </Tooltip>
    );
}

export function Navbar({closeMenu}: { closeMenu: () => void }) {
    const {pathname} = useLocation()
    const {root} = useRoot()
    const {preferences} = usePersistingPreferences();

    const links = preferences.hosts.map(({name, icon}) => (
        <NavbarLink
            key={name}
            path={`${root}/device/${name}`}
            icon={icon}
            label={name}
            active={pathname === `${root}/device/${name}`}
            onClick={closeMenu}
        />
    ));

    return (
        <nav>
            <Stack justify="center" gap={0}>
                <NavbarLink
                    icon={"Home"}
                    label="Home"
                    path={`${root}/`}
                    active={pathname === '/'}
                    onClick={closeMenu}
                />
            </Stack>

            <Stack justify="center" gap={0}>
                {links}
            </Stack>

            <Stack justify="center" gap={0}>
                <NavbarLink
                    icon={"Settings"}
                    label="Settings"
                    path={`${root}/settings`}
                    active={pathname === `${root}/settings`}
                    onClick={closeMenu}
                />
            </Stack>
        </nav>
    );
}
